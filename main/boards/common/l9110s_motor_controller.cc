#include "l9110s_motor_controller.h"

#include <algorithm>
#include <esp_log.h>
#include <esp_random.h>

#define TAG "L9110S"

namespace {
constexpr ledc_mode_t kMotorLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kMotorLedcTimer = LEDC_TIMER_2;
constexpr ledc_timer_bit_t kMotorLedcResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kMotorLedcMaxDuty = (1 << 10) - 1;
constexpr uint32_t kMotorLedcFrequency = 20000;
constexpr ledc_channel_t kMotorIn1Channel = LEDC_CHANNEL_2;
constexpr ledc_channel_t kMotorIn2Channel = LEDC_CHANNEL_3;
}  // namespace

L9110sMotorController::L9110sMotorController(gpio_num_t in1_gpio, gpio_num_t in2_gpio)
    : in1_gpio_(in1_gpio), in2_gpio_(in2_gpio) {
    if (in1_gpio_ == GPIO_NUM_NC || in2_gpio_ == GPIO_NUM_NC) {
        return;
    }

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kMotorLedcMode;
    timer_config.duty_resolution = kMotorLedcResolution;
    timer_config.timer_num = kMotorLedcTimer;
    timer_config.freq_hz = kMotorLedcFrequency;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t in1_channel = {};
    in1_channel.gpio_num = in1_gpio_;
    in1_channel.speed_mode = kMotorLedcMode;
    in1_channel.channel = kMotorIn1Channel;
    in1_channel.intr_type = LEDC_INTR_DISABLE;
    in1_channel.timer_sel = kMotorLedcTimer;
    in1_channel.duty = 0;
    in1_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&in1_channel));

    ledc_channel_config_t in2_channel = {};
    in2_channel.gpio_num = in2_gpio_;
    in2_channel.speed_mode = kMotorLedcMode;
    in2_channel.channel = kMotorIn2Channel;
    in2_channel.intr_type = LEDC_INTR_DISABLE;
    in2_channel.timer_sel = kMotorLedcTimer;
    in2_channel.duty = 0;
    in2_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&in2_channel));

    Stop();

    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool(
        "self.motor.get_state",
        "Get the DC motor state and speed. Use only when the user asks about motor status, not before movement commands.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return GetStateJson();
        });

    mcp_server.AddTool(
        "self.motor.move_forward",
        "Silently move the DC motor forward on GPIO12/GPIO13. Use this directly when the user says move forward, go forward, forward, come here, come to me, come closer, or come. Do not call get_state first. Do not say anything before or after calling this tool.",
        PropertyList({
            Property("speed", kPropertyTypeInteger, 70, 0, 100),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            MoveForward(properties["speed"].value<int>());
            return std::string{};
        });

    mcp_server.AddTool(
        "self.motor.move_backward",
        "Silently move the DC motor backward on GPIO12/GPIO13. Use this directly when the user says move backward, go backward, reverse, back, go away, move away, or back up. Do not call get_state first. Do not say anything before or after calling this tool.",
        PropertyList({
            Property("speed", kPropertyTypeInteger, 70, 0, 100),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            MoveBackward(properties["speed"].value<int>());
            return std::string{};
        });

    mcp_server.AddTool(
        "self.motor.stop",
        "Silently stop the DC motor on GPIO12/GPIO13. Use this directly when the user says stop, stop motor, or halt. Do not call get_state first. Do not say anything before or after calling this tool.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            Stop();
            return std::string{};
        });

    mcp_server.AddTool(
        "self.motor.dance",
        "Silently run dance mode on the DC motor using random forward and backward movements on GPIO12/GPIO13. Use this directly when the user says let's dance, dance, do a dance, or dance mode. Do not call get_state first. Do not say anything before or after calling this tool.",
        PropertyList({
            Property("duration_seconds", kPropertyTypeInteger, 8, 2, 30),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            Dance(properties["duration_seconds"].value<int>());
            return std::string{};
        });
}

void L9110sMotorController::MoveForward(int speed) {
    speed_ = ClampSpeed(speed);
    direction_ = speed_ == 0 ? Direction::kStopped : Direction::kForward;
    SetOutputs(SpeedToDuty(speed_), 0);
    ESP_LOGI(TAG, "Motor forward, speed=%d", speed_);
}

void L9110sMotorController::MoveBackward(int speed) {
    speed_ = ClampSpeed(speed);
    direction_ = speed_ == 0 ? Direction::kStopped : Direction::kBackward;
    SetOutputs(0, SpeedToDuty(speed_));
    ESP_LOGI(TAG, "Motor backward, speed=%d", speed_);
}

bool L9110sMotorController::Dance(int duration_seconds) {
    duration_seconds = ClampDurationSeconds(duration_seconds);
    if (dance_running_) {
        ESP_LOGW(TAG, "Dance mode is already running");
        return false;
    }

    dance_running_ = true;
    dance_duration_seconds_ = duration_seconds;
    auto result = xTaskCreate(
        [](void* arg) {
            auto self = static_cast<L9110sMotorController*>(arg);
            self->DanceTask(self->dance_duration_seconds_);
        },
        "MotorDance",
        2048,
        this,
        tskIDLE_PRIORITY + 1,
        &dance_task_handle_);

    if (result != pdPASS) {
        dance_running_ = false;
        dance_task_handle_ = nullptr;
        ESP_LOGE(TAG, "Failed to start dance task");
        return false;
    }

    ESP_LOGI(TAG, "Dance mode started for %d seconds", duration_seconds);
    return true;
}

void L9110sMotorController::Stop() {
    dance_running_ = false;
    speed_ = 0;
    direction_ = Direction::kStopped;
    SetOutputs(0, 0);
    if (dance_task_handle_ != nullptr && xTaskGetCurrentTaskHandle() != dance_task_handle_) {
        vTaskDelete(dance_task_handle_);
        dance_task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "Motor stopped");
}

std::string L9110sMotorController::GetStateJson() const {
    return std::string("{\"direction\":\"") + DirectionName() + "\",\"speed\":" + std::to_string(speed_) +
        ",\"dancing\":" + (dance_running_ ? "true" : "false") + "}";
}

int L9110sMotorController::ClampSpeed(int speed) const {
    return std::clamp(speed, 0, 100);
}

int L9110sMotorController::ClampDurationSeconds(int duration_seconds) const {
    return std::clamp(duration_seconds, 2, 30);
}

uint32_t L9110sMotorController::SpeedToDuty(int speed) const {
    return static_cast<uint32_t>(ClampSpeed(speed) * kMotorLedcMaxDuty / 100);
}

void L9110sMotorController::SetOutputs(uint32_t in1_duty, uint32_t in2_duty) {
    ledc_set_duty(kMotorLedcMode, kMotorIn1Channel, in1_duty);
    ledc_update_duty(kMotorLedcMode, kMotorIn1Channel);
    ledc_set_duty(kMotorLedcMode, kMotorIn2Channel, in2_duty);
    ledc_update_duty(kMotorLedcMode, kMotorIn2Channel);
}

void L9110sMotorController::DanceTask(int duration_seconds) {
    auto end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration_seconds * 1000);

    while (dance_running_ && xTaskGetTickCount() < end_tick) {
        int speed = 55 + (esp_random() % 46);
        int direction = esp_random() % 2;

        if (direction == 0) {
            MoveForward(speed);
        } else {
            MoveBackward(speed);
        }

        vTaskDelay(pdMS_TO_TICKS(2000 + (esp_random() % 1001)));
    }

    speed_ = 0;
    direction_ = Direction::kStopped;
    SetOutputs(0, 0);
    dance_running_ = false;
    dance_task_handle_ = nullptr;
    ESP_LOGI(TAG, "Dance mode finished");
    vTaskDelete(nullptr);
}

const char* L9110sMotorController::DirectionName() const {
    switch (direction_) {
        case Direction::kForward:
            return "forward";
        case Direction::kBackward:
            return "backward";
        case Direction::kStopped:
        default:
            return "stopped";
    }
}
