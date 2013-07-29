#include "functions.h"

/*************************** X5 Code ********************************************/
void PinOutput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK){

  RCC_AHB1PeriphClockCmd(CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(PORT, &GPIO_InitStructure);
}

void PinInput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK){

  RCC_AHB1PeriphClockCmd(CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(PORT, &GPIO_InitStructure);
}

void Mux_Init(){
  PinOutput(MUX_0_PIN, MUX_0_PORT, MUX_0_CLK);
  GPIO_ResetBits(MUX_0_PORT, MUX_0_PIN);
  PinOutput(MUX_1_PIN, MUX_1_PORT, MUX_1_CLK);
  GPIO_ResetBits(MUX_1_PORT, MUX_1_PIN);
}

void TurnerSelectInit(){
  PinOutput(TURNER_SEL_0_PIN, TURNER_SEL_0_PORT, TURNER_SEL_0_CLK);
  GPIO_ResetBits(TURNER_SEL_0_PORT, TURNER_SEL_0_PIN);
  PinOutput(TURNER_SEL_1_PIN, TURNER_SEL_1_PORT, TURNER_SEL_1_CLK);
  GPIO_ResetBits(TURNER_SEL_1_PORT, TURNER_SEL_1_PIN);
}

void TurnerDirInit(){
  PinOutput(TURNER_DIR_PIN, TURNER_DIR_PORT, TURNER_DIR_CLK);
}

void WaterInit(){
  PinInput(WATER_0_PIN, WATER_0_PORT, WATER_0_CLK);
  PinInput(WATER_1_PIN, WATER_1_PORT, WATER_1_CLK);
  PinInput(WATER_2_PIN, WATER_2_PORT, WATER_2_CLK);
  PinInput(WATER_3_PIN, WATER_3_PORT, WATER_3_CLK);
}

void FuseInit(){
  PinInput(FUSE_DECT_PIN, FUSE_DECT_PORT, FUSE_DECT_CLK);
}

void TurnerPWMInit(){
  /* GPIOD clock enable */
  RCC_APB2PeriphClockCmd(TURNER_PWM_TIM_RCC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = TURNER_PWM_1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(TURNER_PWM_1_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(TURNER_PWM_1_PORT, TURNER_PWM_1_SOURCE, TURNER_PWM_TIM_AF);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TURNER_PWM_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(TURNER_PWM_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TURNER_PWM_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TURNER_PWM_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TURNER_PWM_TIM, ENABLE);
  TURNER_PWM_TIM->CCR1 = 0;
}

void NVIC_Config(void){

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void LedInit(){
  PinOutput(LED_0_PIN, LED_0_PORT, LED_0_CLK);
  PinOutput(LED_1_PIN, LED_1_PORT, LED_1_CLK);
  PinOutput(LED_2_PIN, LED_2_PORT, LED_2_CLK);
  PinOutput(LED_3_PIN, LED_3_PORT, LED_3_CLK);
}

void Usart2Init(){ //fix this 

  BufferInit(&U2Rx);
  BufferInit(&U2Tx);

  NVIC_Config();
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART2);
  
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART2, &USART_InitStructure);
  
  USART_Cmd(USART2, ENABLE);
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

void Usart6Init(){ //fix this 
  PinOutput(USART6_ENABLE_PIN, USART6_ENABLE_PORT, USART6_ENABLE_CLK);
  PinOutput(USART6_DISABLE_PIN, USART6_DISABLE_PORT, USART6_DISABLE_CLK);
  
  BufferInit(&U6Rx);
  BufferInit(&U6Tx);

  NVIC_Config();
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
  
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART6, &USART_InitStructure);
  
  USART_Cmd(USART6, ENABLE);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
}

void MUXControl(volatile uint8_t M0, volatile uint8_t M1){
  if(M0 == 1){
    GPIO_SetBits(MUX_0_PORT, MUX_0_PIN);
  }else{
    GPIO_ResetBits(MUX_0_PORT, MUX_0_PIN);
  }

  if(M1 == 1){
    GPIO_SetBits(MUX_1_PORT, MUX_1_PIN);
  }else{
    GPIO_ResetBits(MUX_1_PORT, MUX_1_PIN);
  }
}

void delay(int i){    //100 000 is about 1/4th a second, 100 000 000 is about 2.5 sec
  while (i-- > 0) {
    asm("nop");       /* This stops it optimising code out */
  }
}

uint16_t GenChecksum(uint16_t address, uint16_t command){
  return (address+command)&0xFF;
}

void UpdateAddress(uint16_t OldAddress, uint16_t NewAddress){
  //Getting things set up  
  int OldAddress1 = ((OldAddress & 0xF0) >> 4);
  if(OldAddress1 >= 0x0A){
      OldAddress1 += 0x37;
  }else{
      OldAddress1 += 0x30;
  }

  int OldAddress2 = ((OldAddress & 0x0F)) ;
  if(OldAddress2 >= 0x0A){
      OldAddress2 += 0x37;
  }else{
      OldAddress2 += 0x30;
  } 
  
  int NewAddress1 = ((NewAddress & 0xF0) >> 4);
  if(NewAddress1 >= 0x0A){
      NewAddress1 += 0x37;
  }else{
      NewAddress1 += 0x30;
  }

  int NewAddress2 = ((NewAddress & 0x0F)) ;
  if(NewAddress2 >= 0x0A){
      NewAddress2 += 0x37;
  }else{
      NewAddress2 += 0x30;
  }  
  
  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //Old address byte
  USART_SendData(USART6, OldAddress1);
  delay(3000);
  
  USART_SendData(USART6, OldAddress2);
  delay(3000);

  //Change address command
  USART_SendData(USART6, 0x30);//0
  delay(3000);
  
  USART_SendData(USART6, 0x42);//B
  delay(3000);
  
  //New Address byte
  USART_SendData(USART6, NewAddress1);
  delay(3000);
  
  USART_SendData(USART6, NewAddress1);
  delay(3000);
  
  //checksum byte - Same as new address
  USART_SendData(USART6, NewAddress1);
  delay(3000);
  
  USART_SendData(USART6, NewAddress1);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}

void MotorControl(uint16_t Address, uint16_t Speed){
  //Getting things set up  
  int Address1 = ((Address & 0xF0) >> 4);
  if(Address1 >= 0x0A){
      Address1 += 0x37;
  }else{
      Address1 += 0x30;
  }

  int Address2 = ((Address & 0x0F)) ;
  if(Address2 >= 0x0A){
      Address2 += 0x37;
  }else{
      Address2 += 0x30;
  }  
  
  int Speed1 = ((Speed & 0xF0) >> 4);
  if(Speed1 >= 0x0A){
      Speed1 += 0x37;
  }else{
      Speed1 += 0x30;
  }

  int Speed2 = ((Speed & 0x0F));
  if(Speed2 >= 0x0A){
      Speed2 += 0x37;
  }else{
      Speed2 += 0x30;
  }

  int Checksum = GenChecksum(Address, Speed);
  int Checksum1 = ((Checksum & 0xF0) >> 4);
  if(Checksum1 >= 0x0A){
      Checksum1 += 0x37;
  }else{
      Checksum1 += 0x30;
  }

  int Checksum2 = ((Checksum & 0x0F));
  if(Checksum2 >= 0x0A){
      Checksum2 += 0x37;
  }else{
      Checksum2 += 0x30;
  }
  
  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //address byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
  
  //speed byte
  USART_SendData(USART6, Speed1);
  delay(3000);
  
  USART_SendData(USART6, Speed2);
  delay(3000);
  
  //nothing byte
  USART_SendData(USART6, 0x30);
  delay(3000);
  
  USART_SendData(USART6, 0x30);
  delay(3000);
     
  //checksum byte
  USART_SendData(USART6, Checksum1);
  delay(3000);
  
  USART_SendData(USART6, Checksum2);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}

void RequestStatus(uint16_t Address){
  //Getting things set up  
  int Address1 = ((Address & 0xF0) >> 4);
  if(Address1 >= 0x0A){
      Address1 += 0x37;
  }else{
      Address1 += 0x30;
  }

  int Address2 = ((Address & 0x0F)) ;
  if(Address2 >= 0x0A){
      Address2 += 0x37;
  }else{
      Address2 += 0x30;
  }  
  
  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //address byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
     
  //checksum byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}



/****************** Usart & Buffer Functions ******************/
void Usart2Put(uint8_t ch){
  //put char to the buffer
  BufferPut(&U2Tx, ch);
  //enable Transmit Data Register empty interrupt
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

uint8_t Usart2Get(void){
  uint8_t ch;
  //check if buffer is empty
  while (BufferIsEmpty(U2Rx) ==SUCCESS);
  BufferGet(&U2Rx, &ch);
  return ch;
}

void Usart6Put(uint8_t ch){
  //put char to the buffer
  BufferPut(&U6Tx, ch);
  //enable Transmit Data Register empty interrupt
  USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

uint8_t Usart6Get(void){
  uint8_t ch;
  //check if buffer is empty
  while (BufferIsEmpty(U6Rx) ==SUCCESS);
  BufferGet(&U6Rx, &ch);
  return ch;
}

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
void BufferInit(__IO FIFO_TypeDef *buffer){

  buffer->count = 0;//0 bytes in buffer
  buffer->in = 0;//index points to start
  buffer->out = 0;//index points to start
}

ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch){
  if ( ch == 0x12){
    buffer->in=0;
  }
  //if(buffer->count==USARTBUFFSIZE)
  //  return ERROR;//buffer full
  buffer->buff[buffer->in++]=ch;
  //buffer->count++;
  if(buffer->in==USARTBUFFSIZE)
    buffer->in=0;//start from beginning
  return SUCCESS;
}

ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch){
  if(buffer->count==0)
    return ERROR;//buffer empty
  *ch=buffer->buff[buffer->out++];
  buffer->count--;
  if(buffer->out==USARTBUFFSIZE)
    buffer->out=0;//start from beginning
  return SUCCESS;
}

ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer){
  if(buffer.count==0)
    return SUCCESS;//buffer full
  return ERROR;
}


/****************** Function that initializes all pin types (I think) ******************/
void PinInit(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK, const uint8_t CH, const uint8_t SOURCE, TIM_TypeDef* TIM, const uint32_t TIM_RCC, const uint8_t TIM_AF, int INPUT){  //8 inputs to function
	//enable RCC on gpio pin NOT TIMER THINGS
	//AKA need to do any GPIO at all
	//Gotta do RCC stuff first.
	if(TIM == TIM14){
		RCC_APB1PeriphClockCmd(TIM_RCC, ENABLE);

	}else if(TIM != NULL){
		if(TIM_RCC == RCC_APB2Periph_TIM9 | TIM_RCC == RCC_APB2Periph_TIM8 ){ //timer 9 is special.
			RCC_APB2PeriphClockCmd(CLK, ENABLE);

		}else{
			
			RCC_APB1PeriphClockCmd(CLK, ENABLE);
		}
	}
	
	RCC_AHB1PeriphClockCmd(CLK, ENABLE);

	//Start initinalizing GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  PIN;
	
	//dynamic gpio stuff
	if( INPUT == 1 ){ //1 for out,2 for AF,  0 or else for input

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	}else if( INPUT == 2){ // for things that need Alternate outputs. aka pierphials

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	}else{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	}
	
	//pretty sure always the same gpio stuff
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//I think the same in all cases
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//Same as above
	
	//stuff for pwm NOTE* Remember to exclude other timers for other periphials
	if (TIM == TIM8){ //this part for LED init

    uint16_t PrescalerValue = (uint16_t) (SystemCoreClock /8400000) - 1;   //work on this

		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;
		
		//this part
		TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
		TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
		TIM_TimeBaseInitStruct.TIM_Period =2000-1;   // 0..999
		TIM_TimeBaseInitStruct.TIM_Prescaler = PrescalerValue; // Div 240
        TIM_TimeBaseInit( TIM, &TIM_TimeBaseInitStruct );
		//to this part need to be fixed to generate the correct pwm frequency
	
        TIM_OCStructInit( &TIM_OCInitStruct );
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; //has nothing to do with channel
		// Initial duty cycle equals 0%. Value can range from zero to 1000.
		TIM_OCInitStruct.TIM_Pulse = 500; // 0 .. 1000 (0=Always Off, 1000=Always On)

		if(CH == 1){
			TIM_OC1Init( TIM, &TIM_OCInitStruct ); //add in IF statement here
		}else if(CH == 2){
			TIM_OC2Init( TIM, &TIM_OCInitStruct );
		}else if(CH == 3){
			TIM_OC3Init( TIM, &TIM_OCInitStruct );
		}
        	TIM_Cmd( TIM, ENABLE );
	}else if (TIM != 0 && TIM != TIM14){ //then not a pin that has a timer!

    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 45000) - 1;   //work on this

		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;
		
		//this part
		TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
		TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
		TIM_TimeBaseInitStruct.TIM_Period =2000 - 1;   // 0..999
		TIM_TimeBaseInitStruct.TIM_Prescaler = PrescalerValue; // Div 240
        TIM_TimeBaseInit( TIM, &TIM_TimeBaseInitStruct );
		//to this part need to be fixed to generate the correct pwm frequency
	
        TIM_OCStructInit( &TIM_OCInitStruct );
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; //has nothing to do with channel
		// Initial duty cycle equals 0%. Value can range from zero to 1000.
		TIM_OCInitStruct.TIM_Pulse = 500; // 0 .. 1000 (0=Always Off, 1000=Always On)

		if(CH == 1){
			TIM_OC1Init( TIM, &TIM_OCInitStruct ); //add in IF statement here
		}else if(CH == 2){
			TIM_OC2Init( TIM, &TIM_OCInitStruct );
		}else if(CH == 3){
			TIM_OC3Init( TIM, &TIM_OCInitStruct );
		}
        	TIM_Cmd( TIM, ENABLE );
	}
	
	
	if( INPUT == 2){
			GPIO_PinAFConfig(PORT, SOURCE, TIM_AF); //changes for every pin
	}
	
	//initinalize the GPIO's
	GPIO_Init(PORT, &GPIO_InitStructure);
	
}
