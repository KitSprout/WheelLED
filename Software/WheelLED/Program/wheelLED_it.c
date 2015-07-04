/*====================================================================================================*/
/*====================================================================================================*/
#include "stm32f0_system.h"
#include "wheelLED.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define USARTx USART1
void USART1_IRQHandler( void )
{
  uint8_t recvByte = 0;

  if((USARTx->ISR & USART_FLAG_TXE) != RESET) {
    switch(WheelLED_Mode) {
      case WLED_MODE_RUN: {
        recvByte = (uint8_t)(USARTx->RDR & (uint16_t)0x00FF);
        USARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((USARTx->ISR & USART_FLAG_TXE) == RESET);
        WheelLED_RecvData(&recvByte);
        break;
      }
      case WLED_MODE_SET: {
        recvByte = (uint8_t)(USARTx->RDR & (uint16_t)0x00FF);
        WheelLED_RecvData(&recvByte);
        USARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((USARTx->ISR & USART_FLAG_TXE) == RESET);
        break;
      }
      case WLED_MODE_IAP: {
        recvByte = (uint8_t)(USARTx->RDR & (uint16_t)0x00FF);
        USARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((USARTx->ISR & USART_FLAG_TXE) == RESET);
        break;
      }
      default: {
        
      }
    }
  }
  USARTx->ICR = USART_IT_RXNE;
}
/*====================================================================================================*/
/*====================================================================================================*/
void NMI_Handler( void ) { while(1); }
void HardFault_Handler( void ) { while(1); }
void SVC_Handler( void ) { while(1); }
void PendSV_Handler( void ) { while(1); }
/*====================================================================================================*/
/*====================================================================================================*/
//void SysTick_Handler( void );
//void WWDG_IRQHandler( void );
//void RTC_IRQHandler( void );
//void FLASH_IRQHandler( void );
//void RCC_IRQHandler( void );
//void EXTI0_1_IRQHandler( void );
//void EXTI2_3_IRQHandler( void );
//void EXTI4_15_IRQHandler( void );
//void DMA1_Channel1_IRQHandler( void );
//void DMA1_Channel2_3_IRQHandler( void );
//void DMA1_Channel4_5_IRQHandler( void );
//void ADC1_IRQHandler( void );
//void TIM1_BRK_UP_TRG_COM_IRQHandler( void );
//void TIM1_CC_IRQHandler( void );
//void TIM3_IRQHandler( void );
//void TIM14_IRQHandler( void );
//void TIM15_IRQHandler( void );
//void TIM16_IRQHandler( void );
//void TIM17_IRQHandler( void );
//void I2C1_IRQHandler( void );
//void I2C2_IRQHandler( void );
//void SPI1_IRQHandler( void );
//void SPI2_IRQHandler( void );
//void USART1_IRQHandler( void );
//void USART2_IRQHandler( void );
/*====================================================================================================*/
/*====================================================================================================*/
