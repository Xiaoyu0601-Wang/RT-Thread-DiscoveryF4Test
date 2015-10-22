/*
 * File      : cpuusage.h
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

#ifndef __CPUUSAGE_H__
#define __CPUUSAGE_H__

#include <rtthread.h>


void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
void cpu_usage_init(void);


#endif
