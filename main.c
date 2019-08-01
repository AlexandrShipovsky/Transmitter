/**
  ******************************************************************************
  * @file    main.c 
  * @author  Shipovsky
  * @version V
  * @date    00-April-2019
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "UART/UART.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_crc.h"
#include <stdio.h>
<<<<<<< HEAD

//HOME

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */


/** @addtogroup USART_Printf
  * @{
  */

=======
#include "protV/protV.h"
// Пин LED
#define LEDpin GPIO_Pin_13
>>>>>>> Первый вариант
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
protVstructure prot;  // Структура протокола
uint8_t bufON[12];
uint8_t buf[12];
/* Private function prototypes -----------------------------------------------*/
void Delay_ms(uint32_t ms);
void DMA_ini(void);
ErrorStatus RCC_ini(void);
/* Private functions ---------------------------------------------------------*/
// Программная задержка
void Delay_ms(uint32_t ms)
{
  for (uint32_t i = 0; i < ms; i++)
  {
  }
}
// Настройки тактирования
ErrorStatus RCC_ini(void)
{
  RCC_DeInit();              //Сброс настроек
  RCC_HSEConfig(RCC_HSE_ON); //Включаем внешний кварцевый резонатор
  ErrorStatus HSEStartUpStatus;
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); /* Ждем пока HSE будет готов */
  if (HSEStartUpStatus == ERROR)
  {
    return HSEStartUpStatus;
  }
  /* HCLK = SYSCLK */ /* Смотри на схеме AHB Prescaler. Частота не делится (RCC_SYSCLK_Div1) */
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  /* PCLK2 = HCLK */ /* Смотри на схеме APB2 Prescaler. Частота не делится (RCC_HCLK_Div1)  */
  RCC_PCLK2Config(RCC_HCLK_Div1);
  /* PCLK1 = HCLK/2 */ /* Смотри на схеме APB1 Prescaler. Частота делится на 2 (RCC_HCLK_Div2)
	потому что на выходе APB1 должно быть не более 36МГц (смотри схему тактирования) */
  RCC_PCLK1Config(RCC_HCLK_Div2);
  /* PLLCLK = 8MHz * 9 = 72 MHz */
  /* Указываем PLL от куда брать частоту (RCC_PLLSource_HSE_Div1) и на сколько ее умножать (RCC_PLLMul_9) */
  /* PLL может брать частоту с кварца как есть (RCC_PLLSource_HSE_Div1) или поделенную на 2 (RCC_PLLSource_HSE_Div2). Смотри схему */
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  /* Включаем PLL */
  RCC_PLLCmd(ENABLE);
  /* Ждем пока PLL будет готов */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  };
  /* Переключаем системное тактирование на PLL */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  /* Ждем пока переключиться */
  while (RCC_GetSYSCLKSource() != 0x08)
  {
  };
  return HSEStartUpStatus;
}
// Настройка DMA
void DMA_ini(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel5); // Сброс настроек DMA
  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   // Адрес данных UART
  DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&buf[0];               // Адрес буфера памяти
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                      // Направление передачи от переферии в память
  DMA_InitStruct.DMA_BufferSize = sizeof(buf);                         // Размер буфера
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // Отключить инкремент адреса данных переферии
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // Включить инкремент адреса памяти
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Байт данных для переферии
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // Байт данных для памяти
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         // Циклический режим работы
  DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;                   // Приоритет
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;                            // Отключить режим "из памяти в память"
  DMA_Init(DMA1_Channel5, &DMA_InitStruct);

  DMA_Cmd(DMA1_Channel5, ENABLE); // Включить DMA канал 5
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  ErrorStatus RCCStatus;
  RCCStatus = RCC_ini();
  UART_Init();
  DMA_ini();
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

  /*!< At this stage the microcontroller setting
     */

  // Настройка LED
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef PIN_INIT;
  PIN_INIT.GPIO_Pin = LEDpin;
  PIN_INIT.GPIO_Speed = GPIO_Speed_10MHz;
  PIN_INIT.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOC, &PIN_INIT);
  if (RCCStatus) // Проверка настроек тактирования
  {
    //Send_UART_Str(USART1, "I'm ready!\n\rRCC SUCCESS\n\r");
    //printf("I'm ready!\n\rRCC SUCCESS\n\r");
  }
  else
  {
    //Send_UART_Str(USART1, "I'm ready!\n\rRCC ERROR\n\r");
    //printf("I'm ready!\n\rRCC ERROR\n\r");
  }

  uint32_t crc;    // Переменная для хранения CRC
  WordToByte word; // Объединение для расчета CRC

  while (1)
  {
    GPIO_ResetBits(GPIOC, LEDpin);
    bufON[0] = 0xAA;
    bufON[1] = 0x63;
    bufON[2] = 0x73;
    bufON[3] = 0x63;

    word.byte[3] = bufON[0];
    word.byte[2] = bufON[1];
    word.byte[1] = bufON[2];
    word.byte[0] = bufON[3];

    CRC_ResetDR();
    CRC_CalcCRC(word.word);
    crc = CRC_GetCRC();
    word.word = crc;
    bufON[4] = word.byte[3];
    bufON[5] = word.byte[2];
    bufON[6] = word.byte[1];
    bufON[7] = word.byte[0];

    c_form(NumbOfErr, ProtLength-1); // Будем исправлять 2 ошибки, в буфере длиной ProtLength - 1 байт (за вычетом стартового)
    c_code(&bufON[1]);     // Тепрь buf длиной 12 содержит 4 кодовых байт+8 информационных
    Send_UART_Str(USART1, bufON);
    
    
    Delay_ms(10000000);
    
    GPIO_SetBits(GPIOC, LEDpin);
    bufON[0] = 0xAA;
    bufON[1] = 0x77;
    bufON[2] = 0x63;
    bufON[3] = 0x63;

    word.byte[3] = bufON[0];
    word.byte[2] = bufON[1];
    word.byte[1] = bufON[2];
    word.byte[0] = bufON[3];

    CRC_ResetDR();
    CRC_CalcCRC(word.word);
    crc = CRC_GetCRC();
    word.word = crc;
    bufON[4] = word.byte[3];
    bufON[5] = word.byte[2];
    bufON[6] = word.byte[1];
    bufON[7] = word.byte[0];

    c_form(NumbOfErr, ProtLength-1); // Будем исправлять 2 ошибки, в буфере длиной ProtLength - 1 байт (за вычетом стартового)
    c_code(&bufON[1]);     // Тепрь buf длиной 12 содержит 4 кодовых байт+8 информационных
    Send_UART_Str(USART1, bufON);

    Delay_ms(10000000);
    
  } // END_WHILE
} // END_MAIN

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 Biruch *****END OF FILE****/