#include "exti.h"

#define EXTI_BASE_ADDRESS    (PERIPH_BASE + 0x0400UL + 0x00000UL)

typedef enum {
  EXTI_MODE_IT = 0x00U, /*!< Interrupt Mode */
  EXTI_MODE_EVT = 0x01U  /*!< Event Mode      */
}EXTI_ModeTypeDef;


HAL_StatusTypeDef HAL_EXTI_Init(EXTI_ConfigTypeDef *hexti) {
  uint32_t tmp = 0x00000000;

  /* Check the EXTI handle allocation */
  if (hexti == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_EXTI_LINE(hexti->Line));
  assert_param(IS_EXTI_MODE(hexti->Mode));
  assert_param(IS_EXTI_TRIGGER(hexti->Trigger));

  /* Configure EXTI Mode */
  tmp |= hexti->Mode;

  /* Configure EXTI Trigger */
  tmp |= hexti->Trigger;

  /* Configure EXTI Line */
  SET_BIT(*((volatile uint32_t *)EXTI_BASE_ADDRESS + ((hexti->Line) >> 5U)), (hexti->Line) & EXTI_PIN_MASK);

  /* Configure Rising Falling edge */
  MODIFY_REG(EXTI->RTSR, hexti->Line, (hexti->Trigger & EXTI_TRIGGER_RISING) ? hexti->Line : 0U);
  MODIFY_REG(EXTI->FTSR, hexti->Line, (hexti->Trigger & EXTI_TRIGGER_FALLING) ? hexti->Line : 0U);

  /* Configure EXTI Mode register */
  MODIFY_REG(EXTI->IMR, hexti->Line, (hexti->Mode == EXTI_MODE_IT) ? hexti->Line : 0U);
  MODIFY_REG(EXTI->EMR, hexti->Line, (hexti->Mode == EXTI_MODE_EVENT) ? hexti->Line : 0U);

  return HAL_OK;
}

void MPU6050_EXTI_Init(void) {
	EXTI_ConfigTypeDef EXTI_InitStruct = {0};
	
	EXTI_InitStruct.Line = EXTI_LINE_0;
	EXTI_InitStruct.GPIOSel = EXTI_GPIOA;
	EXTI_InitStruct.Mode = EXTI_MODE_INTERRUPT;
	EXTI_InitStruct.Trigger = EXTI_TRIGGER_FALLING;  // ÏÂ½µÑØ´¥·¢
	
	HAL_EXTI_Init(&EXTI_InitStruct);
}