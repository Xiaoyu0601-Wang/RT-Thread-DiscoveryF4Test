/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>

/********************************Application_Head_File**************************/
#include "led.h"
#include "drv_hwtimer.h"
#include "mpu6500_spi.h"
/********************************Application_Static_Variable********************/

/********************************Application_Initial****************************/
void rt_init_thread_entry(void* parameter)
{

		MPU_TimerInit();
		MPU_DevInit();
}

int rt_application_init()
{
		rt_thread_t tid;
	
//		timer_static_init();
	
    tid = rt_thread_create("platform",
        rt_init_thread_entry, RT_NULL,
        512, 29, 2);/* LED2 */
    if (tid != RT_NULL)
				rt_thread_startup(tid);
	
    tid = rt_thread_create("led",
        led_thread_entry, RT_NULL,
        256, 30, 2);/* LED1 */
    if (tid != RT_NULL)
				rt_thread_startup(tid);
		
    return 0;
}

/*@}*/
