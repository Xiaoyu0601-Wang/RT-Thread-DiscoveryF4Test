/*
 * File      : mpu6500_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-10-11     Wang         the first version
 */
 /*
 * SPI1_MOSI: PA7
 * 
 * 
 *
 * .
*/
#include <rtconfig.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "mpu6500_spi.h"

/* SPI GPIO define. */
#define SPI1_GPIO            GPIOA
#define SPI1_GPIO_CS         GPIO_Pin_4 //CS0: PA4  Chip Select
#define SPI1_GPIO_SCK        GPIO_Pin_5 //SPI1_SCK : PA5
#define SPI1_SCK_PIN_SOURCE  GPIO_PinSource5
#define SPI1_GPIO_MISO       GPIO_Pin_6 //SPI1_MISO: PA6
#define SPI1_MISO_PIN_SOURCE GPIO_PinSource6
#define SPI1_GPIO_MOSI       GPIO_Pin_7 //SPI1_MOSI: PA7
#define SPI1_MOSI_PIN_SOURCE GPIO_PinSource7

static rt_err_t spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message);

/************************************Variable****************************************/
static rt_int16_t AccData[3];//
static rt_int16_t AccRawData[3];//
static rt_int16_t GyroData[3];//
static rt_int16_t GyroRawData[3];//
static rt_int16_t GyroLastRawData[3];
static rt_int16_t TemData;

rt_int16_t AccZero_Offset[3] = {0};//
rt_int16_t GyroZero_Offset[3] = {0};

static struct rt_spi_ops stm32_spi_ops =
{
    spi_configure,
    spi_xfer
};

#ifdef USING_SPI1
static struct rt_spi_device spi1_device;
static struct stm32_spi_bus stm32_spi1_bus;
static struct stm32_spi_cs spi1_cs;
#define MPU_SPI "spi1"
#define MPU_PORT  SPI1      
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct stm32_spi_bus stm32_spi_bus_2;
#endif /* #ifdef USING_SPI2 */

#ifdef USING_SPI3
static struct stm32_spi_bus stm32_spi_bus_3;
#endif /* #ifdef USING_SPI3 */

//------------------ DMA ------------------
#ifdef SPI_USE_DMA
static uint8_t dummy = 0xFF;
#endif

#ifdef SPI_USE_DMA
static void DMA_Configuration(struct stm32_spi_bus * stm32_spi_bus, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(stm32_spi_bus->DMA_Channel_RX_FLAG_TC
                  | stm32_spi_bus->DMA_Channel_RX_FLAG_TE
                  | stm32_spi_bus->DMA_Channel_TX_FLAG_TC
                  | stm32_spi_bus->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Cmd(stm32_spi_bus->DMA_Channel_RX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) recv_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) (&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->DMA_Channel_RX, &DMA_InitStructure);

    DMA_Cmd(stm32_spi_bus->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Cmd(stm32_spi_bus->DMA_Channel_TX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&dummy);;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->DMA_Channel_TX, &DMA_InitStructure);

    DMA_Cmd(stm32_spi_bus->DMA_Channel_TX, ENABLE);
}
#endif

rt_inline uint16_t get_spi_BaudRatePrescaler(rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler;

    /* STM32F40x SPI MAX 18Mhz */
    if(max_hz >= SystemCoreClock/7*3/2 && SystemCoreClock/7*3/2 <= 18000000)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if(max_hz >= SystemCoreClock/7*3/4)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if(max_hz >= SystemCoreClock/7*3/8)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if(max_hz >= SystemCoreClock/7*3/16)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if(max_hz >= SystemCoreClock/7*3/32)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if(max_hz >= SystemCoreClock/7*3/64)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if(max_hz >= SystemCoreClock/7*3/128)
    {
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
        /* min prescaler 256 */
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }

    return SPI_BaudRatePrescaler;
}

static rt_err_t spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
    SPI_InitTypeDef SPI_InitStructure;

    SPI_StructInit(&SPI_InitStructure);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    }
    else if(configuration->data_width <= 16)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    SPI_InitStructure.SPI_BaudRatePrescaler = get_spi_BaudRatePrescaler(configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;

    /* init SPI */
    SPI_I2S_DeInit(stm32_spi_bus->SPI);
    SPI_Init(stm32_spi_bus->SPI, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(stm32_spi_bus->SPI, ENABLE);
    SPI_CalculateCRC(stm32_spi_bus->SPI, DISABLE);

    return RT_EOK;
};

static rt_uint32_t spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    SPI_TypeDef * SPI = stm32_spi_bus->SPI;
    struct stm32_spi_cs * stm32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if(message->cs_take)
    {
        GPIO_ResetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }

#ifdef SPI_USE_DMA
    if(message->length > 32)
    {
        if(config->data_width <= 8)
        {
            DMA_Configuration(stm32_spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
            while (DMA_GetFlagStatus(stm32_spi_bus->DMA_Channel_RX_FLAG_TC) == RESET
                    || DMA_GetFlagStatus(stm32_spi_bus->DMA_Channel_TX_FLAG_TC) == RESET);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
        }
//        rt_memcpy(buffer,_spi_flash_buffer,DMA_BUFFER_SIZE);
//        buffer += DMA_BUFFER_SIZE;
    }
    else
#endif
    {
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if(message->cs_release)
    {
        GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }

    return message->length;
};

/** \brief init and register stm32 spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t stm32_spi_register(SPI_TypeDef * SPI,
                            struct stm32_spi_bus * stm32_spi,
                            const char * spi_bus_name)
{
//    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//RCC_APB2Periph_AFIO

    if(SPI == SPI1)
    {
    	stm32_spi->SPI = SPI1;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        stm32_spi->DMA_Channel_RX = DMA1_Channel2;
        stm32_spi->DMA_Channel_TX = DMA1_Channel3;
        stm32_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC2;
        stm32_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_TE2;
        stm32_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC3;
        stm32_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_TE3;
#endif
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    }
    else if(SPI == SPI2)
    {
        stm32_spi->SPI = SPI2;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        stm32_spi->DMA_Channel_RX = DMA1_Channel4;
        stm32_spi->DMA_Channel_TX = DMA1_Channel5;
        stm32_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC4;
        stm32_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_TE4;
        stm32_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC5;
        stm32_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_TE5;
#endif
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    }
    else if(SPI == SPI3)
    {
    	stm32_spi->SPI = SPI3;
#ifdef SPI_USE_DMA
        /* Enable the DMA2 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

        stm32_spi->DMA_Channel_RX = DMA2_Channel1;
        stm32_spi->DMA_Channel_TX = DMA2_Channel2;
        stm32_spi->DMA_Channel_RX_FLAG_TC = DMA2_FLAG_TC1;
        stm32_spi->DMA_Channel_RX_FLAG_TE = DMA2_FLAG_TE1;
        stm32_spi->DMA_Channel_TX_FLAG_TC = DMA2_FLAG_TC2;
        stm32_spi->DMA_Channel_TX_FLAG_TE = DMA2_FLAG_TE2;
#endif
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    }
    else
    {
        return RT_ENOSYS;
    }

    return rt_spi_bus_register(&stm32_spi->parent, spi_bus_name, &stm32_spi_ops);
}
//int rt_hw_spi_init(void)
//{
//#ifdef RT_USING_SPI1
//    /* register spi bus */
//		static struct stm32_spi_bus stm32_spi;
//		static struct rt_spi_device spi_device;
//		static struct stm32_spi_cs  spi_cs;
//		GPIO_InitTypeDef GPIO_InitStructure;

//		/* Enable GPIO clock */
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);

//		stm32_spi_register(SPI1, &stm32_spi, "spi1");

//    /* attach cs */
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

//		/* spi21: PG10 */
//		spi_cs.GPIOx = GPIOA;
//		spi_cs.GPIO_Pin = GPIO_Pin_4;

//		GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
//		GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
//		GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

//		rt_spi_bus_attach_device(&spi_device, "spi10", "spi1", (void*)&spi_cs);
//		return 0;
//#endif /* RT_USING_SPI1 */
//}
int rt_hw_spi_init(void)
{
#ifdef RT_USING_SPI1
		struct rt_spi_configuration config;
    GPIO_InitTypeDef GPIO_InitStructure;
		
		config.max_hz = 9000000;
		config.data_width = 8;
		config.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
		spi1_device.config = config;
	
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = SPI1_GPIO_SCK
																	| SPI1_GPIO_MISO | SPI1_GPIO_MOSI;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
		
		/* Connect alternate function */
		GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);
		
    /* register spi bus */	
		stm32_spi_register(MPU_PORT, &stm32_spi1_bus, MPU_SPI);

    /* attach cs */
		spi1_cs.GPIOx = SPI1_GPIO;
		spi1_cs.GPIO_Pin = SPI1_GPIO_CS;

		GPIO_InitStructure.GPIO_Pin = spi1_cs.GPIO_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_SetBits(spi1_cs.GPIOx, spi1_cs.GPIO_Pin);
		GPIO_Init(spi1_cs.GPIOx, &GPIO_InitStructure);

		rt_spi_bus_attach_device(&spi1_device, "spi1mpu", MPU_SPI, (void*)&spi1_cs);
#endif /* RT_USING_SPI1 */
    return 0;
}
/****************************************MPU_Device********************************/
void Zero_Offset_Config(void){
		short int Count;
		int sum[3];
		for(Count=1;Count<=500;Count++)
		{
				MPU_ReadData(AccRawData, GyroRawData, TemData);
				sum[0] += AccRawData[0];
				sum[1] += AccRawData[1];
				sum[2] += GyroRawData[2];
				rt_thread_delay(1);
		}
		AccZero_Offset[0] = (short int)(sum[0]/Count);
		AccZero_Offset[1] = (short int)(sum[1]/Count);
		GyroZero_Offset[2] = (short int)(sum[2]/Count);
}
rt_uint8_t SPI_ReadByte( SPI_TypeDef* SPIx )
{
    while((SPIx->SR & SPI_I2S_FLAG_TXE) == (u16)RESET);
    SPIx->DR = 0xFF;
    while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (u16)RESET);

    return SPIx->DR;
}
void MPU_ReadBuf(rt_uint8_t ReadAddr, rt_uint8_t *ReadBuf, u8 Bytes)
{
    rt_uint8_t i = 0;

		GPIO_ResetBits(SPI1_GPIO, SPI1_GPIO_CS);
    //The first bit of the first byte:Read(1) Write(0)
    ReadAddr|=0x80;
		rt_spi_transfer(&spi1_device, &ReadAddr, RT_NULL, 1);

    for(i=0; i<Bytes; i++)
        ReadBuf[i] = SPI_ReadByte(MPU_PORT);
	
		GPIO_SetBits(SPI1_GPIO, SPI1_GPIO_CS);
}
void MPU_ReadData(rt_int16_t *accdata, rt_int16_t *gyrodata, rt_int16_t temdata)
{
		rt_uint8_t bufAR[14];
		MPU_ReadBuf(0x3B , bufAR , 14);
	
    rt_enter_critical();
		accdata[0] = (bufAR[0] << 8) | bufAR[1];
		accdata[1] = (bufAR[2] << 8) | bufAR[3];
		accdata[2] = (bufAR[4] << 8) | bufAR[5];
		temdata = ((bufAR[6] << 8) | bufAR[7]);
		gyrodata[0] = (bufAR[8] << 8) | bufAR[9];
		gyrodata[1] = (bufAR[10] << 8) | bufAR[11];
		gyrodata[2] = (bufAR[12] << 8) | bufAR[13];

		rt_exit_critical();
}
int MPU_DevInit(void)
{
    rt_err_t err;
		rt_device_t dev = RT_NULL;
		rt_uint8_t i;

		rt_uint8_t Reg_Addr[8]={0x19, //SMPLRT_DIV
                    0x1A, //CONFIG 
                    0x1B, //GYRO_CONFIG
                    0x1C, //ACCEL_CONFIG
                    0x1D, //ACCEL_CONFIG 2
                    0x6A, //USER_CTRL
                    0x6B, //PWR_MGMT_1
                    0x6C};//PWR_MGMT_2
                    
    rt_uint8_t Reg_Val[8]={0x00, //SAMPLE_RATE
                    0x00, //Gyro Fs = 32KHz
			              0x00,//csn change ;Gyro Full Scale = +-250dps// 0x1B, //Gyro Full Scale = +-2000dps
                    0x00, //Acc Full Scale = +-2G
                    0x00, //Acc Rate = 4KHz
                    0x10, //SPI mode only
                    0x01, //PLL if ready, else use the Internal oscillator
                    0x00};//All sensors on
										
//    if ((dev = rt_device_find(MPU_SPI)) == RT_NULL)
//    {
//        rt_kprintf("No Device: %s\n", MPU_SPI);
//        return -1;
//    }
//		
//		if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
//    {
//        rt_kprintf("Open %s Fail\n", MPU_SPI);
//        return -1;
//    }
		rt_spi_take(&spi1_device);
		rt_spi_take_bus(&spi1_device);
										
		for(i=0;i<8;i++)
    {
        rt_spi_send_then_send(&spi1_device, &Reg_Addr[i], 1, &Reg_Val[i], 1); 
        rt_thread_delay(1);
    }
		rt_thread_delay(100);
		Zero_Offset_Config();
		return err;
}
