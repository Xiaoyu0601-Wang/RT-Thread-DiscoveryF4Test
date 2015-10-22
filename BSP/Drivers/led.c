/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
#include <rtdevice.h>
#include <rtthread.h>
#include <rtconfig.h>
#include <stm32f4xx.h>
#include "led.h"
#include "board.h"

// led define

#define LED1_rcc                    RCC_AHB1Periph_GPIOG
#define LED1_gpio                   GPIOG
#define LED1_pin                   (GPIO_Pin_13)

#define LED2_rcc                    RCC_AHB1Periph_GPIOG
#define LED2_gpio                   GPIOG
#define LED2_pin                    (GPIO_Pin_14)

void rt_hw_led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(LED1_rcc|LED2_rcc,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = LED1_pin;
    GPIO_Init(LED1_gpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED2_pin;
    GPIO_Init(LED2_gpio, &GPIO_InitStructure);
}
/******************************************************************/
void rt_hw_led_on(rt_uint32_t n)
{
    switch (n)
    {
    case 1:
        GPIO_SetBits(LED1_gpio, LED1_pin);
        break;
    case 2:
        GPIO_SetBits(LED2_gpio, LED2_pin);
        break;
    default:
        break;
    }
}

void rt_hw_led_off(rt_uint32_t n)
{
    switch (n)
    {
    case 1:
        GPIO_ResetBits(LED1_gpio, LED1_pin);
        break;
    case 2:
        GPIO_ResetBits(LED2_gpio, LED2_pin);
        break;
    default:
        break;
    }
}

void rt_hw_led_toogle(rt_uint32_t n)
{
    switch (n)
    {
    case 1:
        LED1_gpio->ODR ^=  LED1_pin;
        break;
    case 2:
        LED2_gpio->ODR ^=  LED2_pin;
        break;
    default:
        break;
    }
}
/******************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>
static rt_uint8_t led_inited = 0;
void led(rt_uint32_t led, rt_uint32_t value)
{
    /* init led configuration if it's not inited. */
    if (!led_inited)
    {
        rt_hw_led_init();
        led_inited = 1;
    }

    if ( led == 1 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(1);
            break;
        case 1:
            rt_hw_led_on(1);
            break;
        default:
            break;
        }
    }

    if ( led == 2 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(2);
            break;
        case 1:
            rt_hw_led_on(2);
            break;
        default:
            break;
        }
    }
}
FINSH_FUNCTION_EXPORT(led, set led[1 - 2] on[1] or off[0].)
#endif
/************************************Application*******************************/
void led_thread_entry(void* parameter)
{
    unsigned int count=0;
	
		rt_hw_led_init();//不能放到Hardware初始化里面
	
    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
#endif
        count++;
        rt_hw_led_toogle(1);
        rt_thread_delay( RT_TICK_PER_SECOND/5 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_toogle(1);
        rt_thread_delay( RT_TICK_PER_SECOND/5 );
    }
}
