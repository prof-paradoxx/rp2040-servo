/**
 * @file servo.h
 * @author Professor Paradox
 * @brief Header file for servo motor library for RP2040 based Raspberry
 * Pi Pico
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

#ifndef _SERVO_H_
#define _SERVO_H_

/* Standard C includes */
#include <stdint.h>

/* Macro definitions */
#define PWM_FREQ_HZ                         50


/**
 * @brief Structure to hold Servo motor parameters
 * 
 */
typedef struct
{
    uint8_t pin;
    uint8_t sliceNum;
    uint8_t channelNum;
    uint32_t wrapPoint;
}servo_t;


/**
 * @brief Function to initialize servo GPIO with 50Hz PWM frequency
 * and 0 deg angle
 * 
 * @param servo Object of servo_t structure
 * @param pin Pin at which servo PWM wire is connected to Pico
 */
void servoAttach(servo_t *servo, uint8_t pin);


/**
 * @brief Function to move the servo to the specified angle
 * 
 * @param servo Object of servo_t structure
 * @param angle Angle at which servo is to be moved (0 to 180 deg)
 */
void servoWrite(servo_t *servo, uint8_t angle);

#endif