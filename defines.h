/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32Fxxx
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef DEFINES_H
#define DEFINES_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif


/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOB
#define HD44780_RS_PIN				GPIO_PIN_0
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOB
#define HD44780_E_PIN				GPIO_PIN_2
#endif
/* Data pins */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOA
#define HD44780_D4_PIN				GPIO_PIN_8
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOA
#define HD44780_D5_PIN				GPIO_PIN_7
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOA
#define HD44780_D6_PIN				GPIO_PIN_6
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOA
#define HD44780_D7_PIN				GPIO_PIN_15
#endif





/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif




