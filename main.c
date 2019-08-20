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
// Состояния реле
#define RelayON_1 ((uint8_t)(0xFF << 6)) // 0x11000000
#define RelayON_2 ((uint8_t)(0xFF >> 6)) // 0x00000011
#define RelayOFF ((uint8_t)(0x00))       // 0x00
// Выводы кнопок
#define but32 GPIO_Pin_0
#define but31 GPIO_Pin_1
#define but22 GPIO_Pin_2
#define but21 GPIO_Pin_3
#define but12 GPIO_Pin_4
#define but11 GPIO_Pin_5
// Порт кнопок
#define ButPORT GPIOA
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
protVstructure prot;     // Структура протокола
uint8_t buf[ProtLength]; // Буфер для передачи
/* Private function prototypes -----------------------------------------------*/
void SendPKG(protVstructure *prot, uint8_t *buf);
void Delay_cpu(uint32_t tick);
void Delay_ustim(uint16_t us);
void Delay_mstim(uint16_t ms);
void Delay_sectim(uint16_t sec);
ErrorStatus RCC_ini(void);
uint8_t ReadButtons(uint8_t button1, uint8_t button2);
void Buttons_ini(void);
void LED_ini(void);
void TimDelay_ini(void);
/* Private functions ---------------------------------------------------------*/
// Инициализация LED
void LED_ini(void)
{
  GPIO_InitTypeDef PIN_INIT;
  PIN_INIT.GPIO_Pin = LEDpin;
  PIN_INIT.GPIO_Speed = GPIO_Speed_10MHz;
  PIN_INIT.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOC, &PIN_INIT);
}
// Инициализация кнопок
void Buttons_ini(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = but11 | but12 | but21 | but22 | but31 | but32;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(ButPORT, &GPIO_InitStructure);
}
// Инициализация таймера
void TimDelay_ini(void)
{
  // TIMER4
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseStructInit(&TIMER_InitStructure);
  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 72;
  TIMER_InitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
  //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}
// Функция проверяет состояния двухкнопок и объединяет их в байт
uint8_t ReadButtons(uint8_t button1, uint8_t button2)
{
  uint8_t byte = RelayOFF;
  if (button1 == 0)
  {
    Delay_sectim(2); // Проверяем еще раз через 2 секунды
    if (button1 == 0)
    {
      byte |= RelayON_1;
      GPIO_ResetBits(GPIOC, LEDpin);
    }
  }
  if (button2 == 0)
  {
    Delay_sectim(2); // Проверяем еще раз через 2 секунды
    if (button2 == 0)
    {
      byte |= RelayON_2;
      GPIO_SetBits(GPIOC, LEDpin);
    }
  }
  return byte;
}
// Программная задержка
void Delay_cpu(uint32_t tick)
{
  for (uint32_t i = 0; i < tick; i++)
  {
  }
}
// Задержка на таймере tim4
void Delay_ustim(uint16_t us)
{
  TIM_SetCounter(TIM4, 0);
  while (us != TIM_GetCounter(TIM4))
  {
  }
}
void Delay_mstim(uint16_t ms)
{
  uint16_t i;
  for (i = 0; i < ms; i++)
  {
    Delay_ustim(1000);
  }
}
void Delay_sectim(uint16_t sec)
{
  uint16_t i;
  for (i = 0; i < sec; i++)
  {
    Delay_mstim(1000);
  }
}
// Настройки тактирования
ErrorStatus RCC_ini(void)
{

  RCC_DeInit(); //Сброс настроек
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

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
// Процедура считает контрольную сумму полезных данных+стартовый байт отправляет пакет в USART1
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
  for (uint8_t it = 0; it < 100; it++)
  {
    USART_SendData(USART1, 0xFF);
    USART_SendData(USART1, 0xFF);
    USART_SendData(USART1, 0xFF);
  }
  // Отправляет в UART
  Send_UART_Str(USART1, buf, ProtLength);
  Send_UART_Str(USART1, buf, ProtLength);
  Send_UART_Str(USART1, buf, ProtLength);
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  // Инициализация
  ErrorStatus RCCStatus;
  RCCStatus = RCC_ini();
  UART_Init();
  Buttons_ini();
  LED_ini();
  TimDelay_ini();

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
  while (1)
  {
    Delay_mstim(70);

    // Заполняем структуру с данными
    prot.fst = ReadButtons(GPIO_ReadInputDataBit(ButPORT, but11), GPIO_ReadInputDataBit(ButPORT, but12));
    prot.snd = ReadButtons(GPIO_ReadInputDataBit(ButPORT, but21), GPIO_ReadInputDataBit(ButPORT, but22));
    prot.trd = ReadButtons(GPIO_ReadInputDataBit(ButPORT, but31), GPIO_ReadInputDataBit(ButPORT, but32));
    // Отправляем пакет
    SendPKG(&prot, buf);

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