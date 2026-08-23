#pragma once

#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "MCPWMDriver.hpp"
#include "ADCOneshot.hpp"
#include "Controller.hpp"

#include "AS5047P.hpp"
#include "DRV8323.hpp"

struct FOCTaskConfig
{
    gpio_num_t cDRV8323_CS;
    gpio_num_t cAS5047P_CS;
    gpio_num_t cSPI0_CLK;
    gpio_num_t cSPI0_PICO;
    gpio_num_t cSPI0_POCI;
};


class FOCTask {
public:
    FOCTask(FOCTaskConfig &config);
    void begin();

    void update();

private:
    FOCTaskConfig _config;

    SPIBase _spi;
    MCPWMDriver _pwm;
    AS5047P _encoder;
    DRV8323 _drv;
    Controller _controller;

    Output _out;

    float _elPosOffset = 0.0f;
};

