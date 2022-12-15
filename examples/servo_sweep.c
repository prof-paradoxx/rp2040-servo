/**
 * @file servo_sweep.c
 * @author Professor Paradox
 * @brief Example program which sweeps the servo between 0-180 degrees
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

/* Library includes */
#include "servo.h"

/* Macro definitions */
#define SERVO_PIN                   2

int main()
{
    servo_t myServo;

    /* Initialize servo */
    servoAttach(&myServo, SERVO_PIN);
    
    while(1)
    {
        for (uint8_t i = 0; i < 180; i++)
        {
            servoWrite(&myServo, i);
            sleep_ms(10);
        }
        
        sleep_ms(100);

        for (uint8_t i = 180; i > 0; i--)
        {
            servoWrite(&myServo, i);
            sleep_ms(10);
        }

        sleep_ms(100);
    }
}