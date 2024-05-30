/*
 * FreeRTOS STM32 Reference Integration
 *
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "hw_defs.h"

static GPIOInterruptCallback_t volatile xGpioCallbacks[ 16 ] = { NULL };
static void * volatile xGpioCallbackContext[ 16 ] = { NULL };

__attribute__( ( optimize( "O0" ) ) ) void prvGetRegistersFromStack( uint32_t * pulFaultStackAddress )
{
    /* These are volatile to try and prevent the compiler/linker optimising them
     * away as the variables never actually get used.  If the debugger won't show the
     * values of the variables, make them global my moving their declaration outside
     * of this function. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */


    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];
#pragma GCC diagnostic pop

    /* When the following line is hit, the variables contain the register values. */
    for( ; ; )
    {
        __NOP();
    }
}

/*
 * @brief Register a callback function for a given gpio.
 * @param usGpioPinMask The target gpio pin's bitmask
 * @param pvCallback Callback function pointer
 * @param pvContext User provided context pointer
 */
void GPIO_EXTI_Register_Callback( uint16_t usGpioPinMask,
                                  GPIOInterruptCallback_t pvCallback,
                                  void * pvContext )
{
    uint32_t ulIndex = POSITION_VAL( usGpioPinMask );

    configASSERT( ulIndex < 16 );

    xGpioCallbacks[ ulIndex ] = pvCallback;
    xGpioCallbackContext[ ulIndex ] = pvContext;
}

/**
 * @brief  EXTI line rising detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Falling_Callback( uint16_t usGpioPinMask )
{
    ( void ) usGpioPinMask;
}

/**
 * @brief  EXTI line rising detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Rising_Callback( uint16_t usGpioPinMask )
{
    uint32_t ulIndex = POSITION_VAL( usGpioPinMask );

    if( xGpioCallbacks[ ulIndex ] != NULL )
    {
        ( *( xGpioCallbacks[ ulIndex ] ) )( xGpioCallbackContext[ ulIndex ] );
    }
}
