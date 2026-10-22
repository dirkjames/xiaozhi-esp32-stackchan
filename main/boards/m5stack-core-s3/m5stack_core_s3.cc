#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"
#include "mcp_server.h"
#include "SCSCL.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include "esp_video.h"

#define TAG "M5StackCoreS3Board"

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};

class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

class M5StackCoreS3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_;
    Aw9523* aw9523_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    EspVideo* camera_;
    esp_timer_handle_t touchpad_timer_;
    PowerSaveTimer* power_save_timer_;
    SCSCL servo_;           // StackChan 转头舵机 (FTServo SCSCL 协议)
    bool servo_ok_ = false;
    bool pitch_ok_ = false;  // 俯仰(点头)舵机 id=2 是否在总线上

    // StackChan 底座 PY32 IO 扩展器 (I2C 0x6F): 通道0 = VM EN, 舵机电源使能。
    // 与官方固件比对定位到的丢失代码 (官方 hal_io_expander.cpp: 
    // 舵机初始化前必做: 等PY32慢启动→ch0设输出+上拉→输出高→延时200ms)。
    // 缺了这一步舵机电源轨不开, 总线上舵机永远不应答。
    void InitializeStackChanBase() {
        ESP_LOGI(TAG, "Init PY32 IO expander (servo power)");
        i2c_device_config_t py32_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6F,
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t py32 = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &py32_cfg, &py32) != ESP_OK) {
            ESP_LOGE(TAG, "PY32 add device failed");
            return;
        }
        auto w8 = [&](uint8_t reg, uint8_t val) -> bool {
            uint8_t buf[2] = {reg, val};
            return i2c_master_transmit(py32, buf, sizeof(buf), 1000) == ESP_OK;
        };
        auto r8 = [&](uint8_t reg, uint8_t* val) -> bool {
            return i2c_master_transmit_receive(py32, &reg, 1, val, 1, 1000) == ESP_OK;
        };
        auto bit_on = [&](uint8_t reg, uint8_t mask) -> bool {
            uint8_t v = 0;
            if (!r8(reg, &v)) return false;
            v |= mask;
            return w8(reg, v);
        };
        auto bit_off = [&](uint8_t reg, uint8_t mask) -> bool {
            uint8_t v = 0;
            if (!r8(reg, &v)) return false;
            v &= ~mask;
            return w8(reg, v);
        };
        // PY32 可能慢启动, 官方逻辑: 每200ms探测版本寄存器, 最多1.2s
        uint8_t ver = 0;
        for (int i = 0; i < 6; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (r8(0x02, &ver) && ver != 0x00 && ver != 0xFF) break;
            ver = 0;
        }
        if (ver == 0) {
            ESP_LOGE(TAG, "PY32 IO expander not found (version read failed)");
            return;
        }
        ESP_LOGI(TAG, "PY32 version: 0x%02X", ver);
        // ch0 = VM EN: 输出 + 上拉 + 输出高 (官方 setServoPowerEnabled(true))
        bool ok = bit_on(0x03, 0x01)    // GPIO_M_L: ch0 output
               && bit_off(0x0B, 0x01)   // GPIO_PD_L: pull-down off
               && bit_on(0x09, 0x01)    // GPIO_PU_L: pull-up on
               && bit_on(0x05, 0x01);   // GPIO_O_L: high
        ESP_LOGI(TAG, "Servo power (VM EN) %s", ok ? "enabled" : "FAILED");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    void InitializeServo() {
        // 官方 StackChan 同款: SCSCL 总线 1Mbps, UART1, TX=G6 RX=G7, yaw 舵机 id=1
        // (引脚与参数取自实测通过的 firmware_capture 工程, 2026-10-16 定案)
        servo_ok_ = servo_.begin(UART_NUM_1, 1000000, 6, 7);
        // 自检: 用 SCSCL 驱动发正确的 READ Position 帧, 舵机有电才会应答
        int pos = servo_.ReadPos(1);
        ESP_LOGI(TAG, "Servo self-test id=1: %s (pos=%d)", pos >= 0 ? "RESPONDED" : "no response", pos);
        // 俯仰(点头)舵机 id=2 探测 (官方 StackChan: id=2, 零位 620)
        int pos2 = servo_.ReadPos(2);
        pitch_ok_ = (pos2 >= 0);
        ESP_LOGI(TAG, "Servo self-test id=2 (pitch): %s (pos=%d)", pitch_ok_ ? "RESPONDED" : "no response", pos2);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.servo.set_angle",
            "转动 StackChan 头部。angle 为相对正前方的角度：正数向左转，负数向右转，0 回正，范围 -60 到 60",
            PropertyList({
                Property("angle", kPropertyTypeInteger, -60, 60)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (!servo_ok_) {
                    throw std::runtime_error("servo bus not ready");
                }
                int angle = properties["angle"].value<int>();
                // 官方 StackChan 映射: raw = 460 + angle*3.2 (零位 460=正前, 实测镜像已含在此式中, 正=左)
                int raw = 460 + (int)(angle * 3.2f);
                if (raw < 0) raw = 0;
                if (raw > 1000) raw = 1000;
                servo_.EnableTorque(1, 1);
                servo_.WritePos(1, (uint16_t)raw, 20, 0);
                int pos = servo_.ReadPos(1);  // 回读验证舵机是否真的执行
                ESP_LOGI(TAG, "Servo set_angle(%d) -> raw %d, readback=%d", angle, raw, pos);
                return true;
            });
        // 俯仰(点头)工具: 仅当 id=2 舵机在总线上时注册 (官方 StackChan: id=2, 零位 620, raw 限幅 30~870)
        if (pitch_ok_) {
            mcp_server.AddTool(
                "self.servo.set_pitch",
                "让 StackChan 头部上下点头。angle 为相对水平的角度：正数低头，负数抬头，0 回正，范围 -30 到 30",
                PropertyList({
                    Property("angle", kPropertyTypeInteger, -30, 30)
                }),
                [this](const PropertyList& properties) -> ReturnValue {
                    int angle = properties["angle"].value<int>();
                    // 官方 StackChan 俯仰: 零位 620, raw 限幅 30~870 (符号待实测, 反了改一处符号)
                    int raw = 620 + (int)(angle * 3.2f);
                    if (raw < 30) raw = 30;
                    if (raw > 870) raw = 870;
                    servo_.EnableTorque(2, 1);
                    servo_.WritePos(2, (uint16_t)raw, 20, 0);
                    int pos = servo_.ReadPos(2);
                    ESP_LOGI(TAG, "Servo set_pitch(%d) -> raw %d, readback=%d", angle, raw, pos);
                    return true;
                });
        }
    }


    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500;  // 触摸时长阈值，超过500ms视为长按
        
        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        
        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched) {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        } 
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;
            
            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    EnterWifiConfigMode();
                    return;
                }
                app.ToggleChatState();
            }
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                M5StackCoreS3Board* board = (M5StackCoreS3Board*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

     void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
        camera_->SetHMirror(false);
    }

public:
    M5StackCoreS3Board() {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeAw9523();
        I2cDetect();
        InitializeSpi();
        InitializeIli9342Display();
        InitializeCamera();
        InitializeFt6336TouchPad();
        InitializeStackChanBase();  // 官方固件同款: 先开舵机电源(VM EN)再初始化舵机
        InitializeServo();
        InitializeTools();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackCoreS3Board);
