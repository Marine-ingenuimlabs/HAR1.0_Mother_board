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
//union_float ff;
//CanRxMsg CANMSG;
//CanTxMsg CanTx;

//__IO uint8_t UserButtonPressed = 0x00;


/* Private function prototypes -----------------------------------------------*/
void init_USART1(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx,__IO char *s);
char* conv_f2c(float f);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    ROV_Init(&ROV);
    FuncCallbackTable_INIT(); //initialization of functions pointers table
    NVIC_SetPriority(SysTick_IRQn, 0x5); //configure systick priority
    SysTick_Config(SystemCoreClock / 20000);
    ROV_ControlMatrix_Init(&ROV); // Matrix Init 
    ROV_coldStart_Init(&ROV);
    ROV.rov_state.is_streaming_enabled = 0;
    ROV.rov_state.is_computer_connected = 0;
    init_USART1(115200);
    
    /**This section is used to configure the debug reporting uart*/
    
    ROV.light.right.integer16 = 1500;
  
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  //STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 

  /* Initialization of ROV thruster config matrix used for debug only */
  
  
  /*float32_t thruster_matrix_vector[36]={
  val1, val2,0   ,0    ,0, val5,
  val1,-val2,0   ,0    ,0,-val5,
  val1, val2,0   ,0    ,0,-val5,
  val1,-val2,0   ,0    ,0, val5,
  0   ,0    ,val3, val4,0,    0,
  0   ,0    ,val3, val4,0,    0 
  };*/

   while (1)
   { 
     
   
      
     
     Sensor_DataUpdate_50Hz(&ROV.measurement_unit_sensors,&ROV.aio.buffers.frame_50Hz,&ROV.rov_state);
     Sensor_DataUpdate_10Hz(&ROV.measurement_unit_sensors,&ROV.aio.buffers.frame_10Hz,&ROV.rov_state);      
     
     //ROV_Routine(&ROV,joy_vector);
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

void USART_puts(USART_TypeDef* USARTx,__IO char *s)
{

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}
char* conv_f2c(float f)
{
  
    static int pint;
    static int pfract;
    static char string[7];
    static int sign;
    
    sign=(int)f;
    pint =(int)f;
    pfract= (int)((f - pint)*1000);
    pint = abs(pint);
    pfract = abs(pfract);
    
    
    
     if (sign < 0)
    {
      sprintf(string,"-%03d.%03d",pint,pfract);
    }  
    else
    {
      sprintf(string,"%03d.%03d",pint,pfract);
    }
    return string;
  
}

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
