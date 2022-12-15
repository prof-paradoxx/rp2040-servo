/**
 * @file servo.c
 * @author Professor Paradox
 * @brief Source file with function implementation to control servo
 * motor using Raspberry Pi Pico
 * @version 0.1
 * @date 2022-12-15
 * 
 * MIT License
 *
 * Copyright (c) 2022 Professor Paradox
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

/* Pico includes */
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

/* Library includes */
#include "servo.h"


/* Private function prototypes */
static float map_val_float(uint16_t x, uint16_t in_min, uint16_t in_max,
    float out_min, float out_max);
static uint32_t configureServoPWM(servo_t *servo);


/**
 * @brief Function to map the input between provided constraints
 * 
 * @param x Value to be mapped
 * @param in_min Minimum value of x
 * @param in_max Maximum value of x
 * @param out_min Minimum value of output (float)
 * @param out_max Maximum value of output (float)
 * @return float Mapped value of x
 */
static float map_val_float(uint16_t x, uint16_t in_min, uint16_t in_max,
    float out_min, float out_max)
{
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min))
        + out_min; 
}


/**
 * @brief Function to configure the servo PWM at 50Hz frequency
 * and set the slice number and channel number for servo PWM GPIO
 * 
 * @param servo Object of the servo_t structure
 * @return uint32_t 
 */
static uint32_t configureServoPWM(servo_t *servo)
{
    uint32_t clockFreq, clockDivider;
    uint8_t pin = servo->pin;

    gpio_set_function(pin, GPIO_FUNC_PWM);

    clockFreq = clock_get_hz(clk_sys);
    clockDivider = clockFreq / PWM_FREQ_HZ / 4096 +
        (clockFreq % (PWM_FREQ_HZ * 4096) != 0);

    if (clockDivider / 16 == 0)
    {
        clockDivider = 16;
    }

    servo->wrapPoint = clockFreq * 16 / clockDivider / PWM_FREQ_HZ - 1;

    servo->sliceNum = pwm_gpio_to_slice_num(pin);
    servo->channelNum = pwm_gpio_to_channel(pin);

    pwm_set_clkdiv_int_frac(servo->sliceNum, clockDivider / 16,
        clockDivider & 0xF);
    pwm_set_wrap(servo->sliceNum, servo->wrapPoint);

    pwm_set_enabled(servo->sliceNum, true);
}


void servoWrite(servo_t *servo, uint8_t angle)
{
    float dutyCycle = map_val_float(angle, 0, 180, 2.5, 12.5);
    pwm_set_chan_level(servo->sliceNum, servo->channelNum,
        servo->wrapPoint * dutyCycle / 100);
}


void servoAttach(servo_t *servo, uint8_t pin)
{
    servo->channelNum = 0;
    servo->pin = pin;
    servo->sliceNum = 0;
    servo->wrapPoint = 0;

    configureServoPWM(servo);
    sleep_ms(100);
    servoWrite(servo, 0);
}