#ifndef L9110S_MOTOR_CONTROLLER_H_
#define L9110S_MOTOR_CONTROLLER_H_

#include "mcp_server.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>

class L9110sMotorController {
public:
    L9110sMotorController(gpio_num_t in1_gpio, gpio_num_t in2_gpio);

    void MoveForward(int speed);
    void MoveBackward(int speed);
    bool Dance(int duration_seconds);
    void Stop();
    std::string GetStateJson() const;

private:
    enum class Direction {
        kStopped,
        kForward,
        kBackward,
    };

    gpio_num_t in1_gpio_;
    gpio_num_t in2_gpio_;
    Direction direction_ = Direction::kStopped;
    int speed_ = 0;
    bool dance_running_ = false;
    int dance_duration_seconds_ = 8;
    TaskHandle_t dance_task_handle_ = nullptr;

    int ClampSpeed(int speed) const;
    int ClampDurationSeconds(int duration_seconds) const;
    uint32_t SpeedToDuty(int speed) const;
    void SetOutputs(uint32_t in1_duty, uint32_t in2_duty);
    void DanceTask(int duration_seconds);
    const char* DirectionName() const;
};

#endif  // L9110S_MOTOR_CONTROLLER_H_
