/* 昇圧・キック・電源出力をLowへ固定し、電源基板を安全な更新状態にする。 */
#include "board_io.h"
#include "stm32f303xc.h"
#include <stdint.h>
#define PIN(n)(UINT32_C(1)<<(n))
void board_io_init_safe(void){
 RCC->APB1RSTR|=RCC_APB1RSTR_TIM2RST|RCC_APB1RSTR_TIM3RST|RCC_APB1RSTR_TIM4RST;RCC->APB1RSTR&=~(RCC_APB1RSTR_TIM2RST|RCC_APB1RSTR_TIM3RST|RCC_APB1RSTR_TIM4RST);RCC->APB1ENR&=~(RCC_APB1ENR_TIM2EN|RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM4EN);
 RCC->AHBENR|=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;
 const uint32_t ao=PIN(3)|PIN(4)|PIN(6)|PIN(7)|PIN(15),bo=PIN(2)|PIN(6)|PIN(7)|PIN(10)|PIN(11),co=PIN(13)|PIN(14)|PIN(15);
 GPIOA->BSRR=ao<<16U;GPIOB->BSRR=bo<<16U;GPIOC->BSRR=co<<16U;
 for(uint32_t pin=0;pin<16U;pin++){if((ao&PIN(pin))!=0U)GPIOA->MODER=(GPIOA->MODER&~(UINT32_C(3)<<(pin*2U)))|(UINT32_C(1)<<(pin*2U));if((bo&PIN(pin))!=0U)GPIOB->MODER=(GPIOB->MODER&~(UINT32_C(3)<<(pin*2U)))|(UINT32_C(1)<<(pin*2U));if((co&PIN(pin))!=0U)GPIOC->MODER=(GPIOC->MODER&~(UINT32_C(3)<<(pin*2U)))|(UINT32_C(1)<<(pin*2U));}
 GPIOB->MODER&=~((UINT32_C(3)<<16U)|(UINT32_C(3)<<18U));GPIOB->PUPDR=(GPIOB->PUPDR&~((UINT32_C(3)<<16U)|(UINT32_C(3)<<18U)))|(UINT32_C(1)<<16U)|(UINT32_C(1)<<18U);
 GPIOA->MODER=(GPIOA->MODER&~((UINT32_C(3)<<22U)|(UINT32_C(3)<<24U)))|(UINT32_C(2)<<22U)|(UINT32_C(2)<<24U);GPIOA->AFR[1]=(GPIOA->AFR[1]&~((UINT32_C(0xF)<<12U)|(UINT32_C(0xF)<<16U)))|(UINT32_C(9)<<12U)|(UINT32_C(9)<<16U);
}
void board_status_set_validating(bool enabled){(void)enabled;}void board_status_set_invalid(bool enabled){(void)enabled;}uint8_t board_update_node_id(void){return 100U;}
