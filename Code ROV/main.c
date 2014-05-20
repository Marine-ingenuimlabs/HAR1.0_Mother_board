/**
  ******************************************************************************
  * @file    TIM_PWM_Output/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "arm_math.h"
#include "communication_lib.h"
#include <stm32f4xx_usart.h> 


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

ROV_Struct ROV ;
union_float ff;
CanRxMsg CANMSG;
CanTxMsg CanTx;
int tt = 0;

/*
float32_t b = 35; //thruster angle
float32_t d = 342.64; //in millimeters
float32_t gamma = 175; //in millimeters
float32_t val1,val2,val3,val4,val5;
float32_t thruster_matrix_vector[36];
arm_matrix_instance_f32 thruster_matrix; 
arm_matrix_instance_f32 force_moment_matrix;
float32_t force_moment_vector[] = { 1 , 0 , 0 , 0 , 0 , 0};
*/

//__IO uint8_t UserButtonPressed = 0x00;


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/



//initialises USART1 used for debug
void init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}





/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  NVIC_SetPriority(SysTick_IRQn, 0x5); //configure systick priority
  
  /*
  val1 = (1/(4*cos(b)));
  val2 = (1/(4*sin(b)));
  val3 = 0.5;
  val4 = (1/(2*gamma));
  val5 = (1/(2*d*cos(b)));
  thruster_matrix_vector[0] = val1;
  thruster_matrix_vector[1] = val2;
  thruster_matrix_vector[2] = 0;
  thruster_matrix_vector[3] = 0;
  thruster_matrix_vector[4] = 0;
  thruster_matrix_vector[5] = val5;
  thruster_matrix_vector[6] = val1;
  thruster_matrix_vector[7] = -val2;
  thruster_matrix_vector[8] = 0;
  thruster_matrix_vector[9] = 0;
  thruster_matrix_vector[10] = 0;
  thruster_matrix_vector[11] = -val5;
  thruster_matrix_vector[12] = val1;
  thruster_matrix_vector[13] = val2;
  thruster_matrix_vector[14] = 0;
  thruster_matrix_vector[15] = 0;
  thruster_matrix_vector[16] = 0;
  thruster_matrix_vector[17] = -val5;
  thruster_matrix_vector[18] = val1;
  thruster_matrix_vector[19] = -val2;
  thruster_matrix_vector[20] = 0;
  thruster_matrix_vector[21] = 0;
  thruster_matrix_vector[22] = 0;
  thruster_matrix_vector[23] = val5;
  thruster_matrix_vector[24] = 0;
  thruster_matrix_vector[25] = 0;
  thruster_matrix_vector[26] = val3;
  thruster_matrix_vector[27] = val4;
  thruster_matrix_vector[28] = 0;
  thruster_matrix_vector[29] = 0;
  thruster_matrix_vector[30] = 0;
  thruster_matrix_vector[31] = 0;
  thruster_matrix_vector[32] = val3;
  thruster_matrix_vector[33] = val4;
  thruster_matrix_vector[34] = 0;
  thruster_matrix_vector[35] = 0;
    
  arm_mat_init_f32(&thruster_matrix, 6, 6,thruster_matrix_vector);
  arm_mat_init_f32(&thruster_matrix, 6, 6,thruster_matrix_vector);
  b = thruster_matrix.pData[26];
 */
 
  ROV_Init(&ROV);
  FuncCallbackTable_INIT(); //initialization of functions pointers table
  
  SysTick_Config(SystemCoreClock / 10000);
 
   ROV.propulsion[0].speed_command.integer16 = 0;
   ROV.propulsion[1].speed_command.integer16 = 0;
   ROV.propulsion[2].speed_command.integer16 = 0;
   ROV.propulsion[3].speed_command.integer16 = 0;
   THRUSTER_update(ROV.propulsion);

    ROV.rov_state.is_streaming_enabled = 0;

    

 ROV.measurement_unit_sensors.AHRS.Euler_Angle.x_value.floating_number = 5.3;
 ROV.measurement_unit_sensors.AHRS.Euler_Angle.y_value.floating_number = 9.6;
 ROV.measurement_unit_sensors.AHRS.Euler_Angle.z_value.floating_number = 7.1;
 ROV.identifiers_table[80].State = 1;
 ROV.identifiers_table[81].State = 0;
 ROV.identifiers_table[82].State = 0;
 

  ROV.light.right.integer16 = 1500;
  
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  //STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 

  
   while (1)
   { 
 /*   if (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
  {
     UserButtonPressed=0;
      // Waiting User Button is released 
      while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
      {} 
        CanTx.DLC = 4;
  CanTx.Data[0] = 0x9A ;
  CanTx.Data[1] = 0x99 ;
  CanTx.Data[2] = 0xA9 ;
  CanTx.Data[3] = 0x40;
  //CanTx.Data[4] = 0x40;

  CanTx.IDE = CAN_Id_Extended;
  CanTx.ExtId = 0x50;
  CAN_Transmit(CAN1, &CanTx);
     
    }
     */
   //ROV.propulsion[0].speed_command.integer16 = 0;
   //ROV.propulsion[1].speed_command.integer16 = 0;
   //THRUSTER_update(ROV.propulsion);
    // ROV_Stream_VAR(ROV);
    
    // Delay(0x1FFFFFF);
  }
  
}

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */



#ifdef  USE_FULL_ASSERT

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

  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
