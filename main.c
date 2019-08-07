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
#include "protV/protV.h"
// Пин LED
#define LEDpin GPIO_Pin_13

#define RelayON_1 ((uint8_t)(0xFF << 4)) // Левые 4 бита = 1
#define RelayON_2 ((uint8_t)(0xFF >> 4)) // Правые 4 бита = 1
#define RelayOFF ((uint8_t)(0x00))
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
protVstructure prot; // Структура протокола
uint8_t buf[12];     // Буфер для передачи
/* Private function prototypes -----------------------------------------------*/
void SendPKG(protVstructure *prot, uint8_t *buf);
void Delay_ms(uint32_t ms);
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
// Процедура отправляет пакет в USART1
void SendPKG(protVstructure *prot, uint8_t *buf)
{
  WordToByte WordProt; // Объединение для расчета CRC
                       // Объединяем 4 байта в слово
  WordProt.byte[3] = StartByte;
  WordProt.byte[2] = prot->fst;
  WordProt.byte[1] = prot->snd;
  WordProt.byte[0] = prot->trd;
  // Вычисляем аппаратно контрольную сумму стартового байта и 3 информационных байт
  CRC_ResetDR();
  CRC_CalcCRC(WordProt.word);
  prot->crc = CRC_GetCRC();
  // Заполняем буфер и кодируем пакет
  UnitBuf(prot, buf);
  // Отправляет в UART
  Send_UART_Str(USART1, buf);
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

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

  /*!< At this stage the microcontroller setting
     */

  // Настройка LED (PC13)
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

  uint32_t i = 0;
  protVstructure prot; // Структура протокола
  while (1)
  {
    Delay_ms(10);
    USART_SendData(USART1, 255);
    if (i == 10000)
    {
      GPIO_ResetBits(GPIOC, LEDpin);
      // Заполняем структуру с данными
      prot.fst = RelayON_1;
      prot.snd = RelayON_1 | RelayON_2;
      prot.trd = RelayOFF;
      // Отправляем пакет
      SendPKG(&prot, buf);
    }

    if (i == 5000)
    {
      GPIO_SetBits(GPIOC, LEDpin);
      // Заполняем структуру с данными
      prot.fst = RelayOFF;
      prot.snd = RelayON_1 | RelayON_2;
      prot.trd = RelayOFF;
      // Отправляем пакет
      SendPKG(&prot, buf);
    }

    i++;
    if (i > 10000)
    {
      i = 0;
    }

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