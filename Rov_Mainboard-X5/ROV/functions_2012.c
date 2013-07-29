#include "functions.h"


//Output for 1, AF output for 2, input for else

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
	if( INPUT == 1 ){ //1 for input,2 for AF,  0 or else for output

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

/****************** Actual device functions that initinalize pins. ******************/
//change motor init to set to 0 pwm output
void Motor1Init(){
	//Non-PWM pins
	PinOutput(A_1_DIR_PIN, A_1_DIR_PORT, A_1_DIR_CLK);//output
	PinOutput(A_1_RST_PIN, A_1_RST_PORT, A_1_RST_CLK);//output

  //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(A_1_RST_PORT, A_1_RST_PIN);
  //pwm stuff below
  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB2PeriphClockCmd(MOTOR1_TIM_RCC, ENABLE); //APB2 not APB1

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(A_1_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = A_1_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(A_1_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(A_1_PWM_PORT, GPIO_PinSource5, MOTOR1_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR1_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR1_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(MOTOR1_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(MOTOR1_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR1_TIM, ENABLE);
	MotorSet(1,1,0);
}

void Motor2Init(){
	//Non-PWM pins
	PinInit(B_1_DIR_PIN, B_1_DIR_PORT, B_1_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_1_RST_PIN, B_1_RST_PORT, B_1_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_1_FAULT_1_PIN, B_1_FAULT_1_PORT, B_1_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(B_1_FAULT_2_PIN, B_1_FAULT_2_PORT, B_1_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
   //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(B_1_RST_PORT, B_1_RST_PIN);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB2PeriphClockCmd(MOTOR1_TIM_RCC, ENABLE); //APB2 not APB1

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(B_1_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = B_1_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_1_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(B_1_PWM_PORT, GPIO_PinSource6, MOTOR1_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR1_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR1_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(MOTOR1_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(MOTOR1_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR1_TIM, ENABLE);
MotorSet(2,1,0);

}

void Motor3Init(){
//Non-PWM pins
	PinInit(A_2_DIR_PIN, A_2_DIR_PORT, A_2_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_2_RST_PIN, A_2_RST_PORT, A_2_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_2_FAULT_1_PIN, A_2_FAULT_1_PORT, A_2_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(A_2_FAULT_2_PIN, A_2_FAULT_2_PORT, A_2_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(A_2_RST_PORT, A_2_RST_PIN);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR2_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(A_2_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = A_2_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(A_2_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(A_2_PWM_PORT, GPIO_PinSource0, MOTOR2_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR2_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR2_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(MOTOR2_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(MOTOR2_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR2_TIM, ENABLE);
MotorSet(3,1,0);
}

void Motor4Init(){
//Non-PWM pins
	PinInit(B_2_DIR_PIN, B_2_DIR_PORT, B_2_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_2_RST_PIN, B_2_RST_PORT, B_2_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_2_FAULT_1_PIN, B_2_FAULT_1_PORT, B_2_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(B_2_FAULT_2_PIN, B_2_FAULT_2_PORT, B_2_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(B_2_RST_PORT, B_2_RST_PIN);

 /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR2_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(B_2_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = B_2_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_2_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(B_2_PWM_PORT, GPIO_PinSource1, MOTOR2_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR2_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR2_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(MOTOR2_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(MOTOR2_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR2_TIM, ENABLE);
  MotorSet(4,1,0);
}

void Motor5Init(){
//Non-PWM pins
	PinInit(A_3_DIR_PIN, A_3_DIR_PORT, A_3_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_3_RST_PIN, A_3_RST_PORT, A_3_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_3_FAULT_1_PIN, A_3_FAULT_1_PORT, A_3_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(A_3_FAULT_2_PIN, A_3_FAULT_2_PORT, A_3_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

  //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(A_3_RST_PORT, A_3_RST_PIN);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR3_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(A_3_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = A_3_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(A_3_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(A_3_PWM_PORT, GPIO_PinSource6, MOTOR3_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR3_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR3_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(MOTOR3_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(MOTOR3_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR3_TIM, ENABLE);
  MotorSet(5,1,0);
}

void Motor6Init(){
//Non-PWM pins
	PinInit(B_3_DIR_PIN, B_3_DIR_PORT, B_3_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_3_RST_PIN, B_3_RST_PORT, B_3_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_3_FAULT_1_PIN, B_3_FAULT_1_PORT, B_3_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(B_3_FAULT_2_PIN, B_3_FAULT_2_PORT, B_3_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

	//Pull reset LOW to DISABLE Output
  GPIO_ResetBits(B_3_RST_PORT, B_3_RST_PIN);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR3_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(B_3_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = B_3_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_3_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(B_3_PWM_PORT, GPIO_PinSource7, MOTOR3_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR3_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR3_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(MOTOR3_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(MOTOR3_TIM, TIM_OCPreload_Enable);
  
  /* TIMx enable counter */
  TIM_Cmd(MOTOR3_TIM, ENABLE);
  MotorSet(6,1,0);
}

void Motor7Init(){
//Non-PWM pins
	PinInit(A_5_DIR_PIN, A_5_DIR_PORT, A_5_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_5_RST_PIN, A_5_RST_PORT, A_5_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_5_FAULT_1_PIN, A_5_FAULT_1_PORT, A_5_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(A_5_FAULT_2_PIN, A_5_FAULT_2_PORT, A_5_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(A_5_RST_PORT, A_5_RST_PIN);

 /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR5_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(A_5_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = A_5_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(A_5_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(A_5_PWM_PORT, GPIO_PinSource14, MOTOR5_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR5_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR5_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC3Init(MOTOR5_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTOR5_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR5_TIM, ENABLE);
  MotorSet(7,1,0);
}

void Motor8Init(){
//Non-PWM pins
	PinInit(B_5_DIR_PIN, B_5_DIR_PORT, B_5_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_5_RST_PIN, B_5_RST_PORT, B_5_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_5_FAULT_1_PIN, B_5_FAULT_1_PORT, B_5_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(B_5_FAULT_2_PIN, B_5_FAULT_2_PORT, B_5_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(B_5_RST_PORT, B_5_RST_PIN);

 /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(MOTOR5_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(B_5_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = B_5_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_5_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(B_5_PWM_PORT, GPIO_PinSource15, MOTOR5_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR5_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR5_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC4Init(MOTOR5_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTOR5_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR5_TIM, ENABLE);
MotorSet(8,1,0);
}
	
void Motor9Init(){
//Non-PWM pins
	PinInit(A_4_DIR_PIN, A_4_DIR_PORT, A_4_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_4_RST_PIN, A_4_RST_PORT, A_4_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(A_4_FAULT_1_PIN, A_4_FAULT_1_PORT, A_4_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(A_4_FAULT_2_PIN, A_4_FAULT_2_PORT, A_4_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(A_4_RST_PORT, A_4_RST_PIN);

 /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB2PeriphClockCmd(MOTOR4_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(A_4_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = A_4_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_4_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(A_4_PWM_PORT, GPIO_PinSource9, MOTOR4_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR4_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR4_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(MOTOR4_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(MOTOR4_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR4_TIM, ENABLE);
  TIM_CtrlPWMOutputs(MOTOR4_TIM, ENABLE); //VERY IMPORTANT THING NEEDED FOR PWM OUTPUT ON TIM8 AND TIM1
  MotorSet(9,1,0);
}

void Motor10Init(){
//Non-PWM pins
	PinInit(B_4_DIR_PIN, B_4_DIR_PORT, B_4_DIR_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_4_RST_PIN, B_4_RST_PORT, B_4_RST_CLK, NULL, NULL, NULL, NULL, NULL, 1);//output
	PinInit(B_4_FAULT_1_PIN, B_4_FAULT_1_PORT, B_4_FAULT_1_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input
	PinInit(B_4_FAULT_2_PIN, B_4_FAULT_2_PORT, B_4_FAULT_2_CLK, NULL, NULL, NULL, NULL, NULL, 0);//input

 //Pull reset LOW to DISABLE Output
  GPIO_ResetBits(B_4_RST_PORT, B_4_RST_PIN);

 /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB2PeriphClockCmd(MOTOR4_TIM_RCC, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(B_4_PWM_CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = B_4_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(B_4_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(B_4_PWM_PORT, GPIO_PinSource11, MOTOR4_TIM_AF);//need pinsource not just pin

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
  TIM_TimeBaseInit(MOTOR4_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(MOTOR4_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(MOTOR4_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(MOTOR4_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(MOTOR4_TIM, ENABLE);
  TIM_CtrlPWMOutputs(MOTOR4_TIM, ENABLE); //VERY IMPORTANT THING NEEDED FOR PWM OUTPUT ON TIM8 AND TIM1
  MotorSet(10,1,0);
}	


void LedInit(){ //done
		
  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIM8 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM8 pins to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    uint16_t Period;

    Period = 10000000 / 2000; // 2 KHz for 1MHz prescaled
		   //1000000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 1000000) / 2) - 1; // Get clock to 1 MHz on STM32F4

  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  /* Enable TIM8 Preload register on ARR */
  //TIM_ARRPreloadConfig(TIM8, ENABLE);

  /* TIM PWM1 Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

  /* TIM8 enable counter */
  TIM_Cmd(TIM8, ENABLE);
  TIM_CtrlPWMOutputs(TIM8, ENABLE); //VERY IMPORTANT THING NEEDED FOR PWM OUTPUT ON TIM8 AND TIM1

  LedControl(0,0,0);
}

void Mux_1Init(){ //done
	PinInit(MUX_1_PIN, MUX_1_PORT, MUX_1_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	GPIO_SetBits(MUX_1_PORT, MUX_1_PIN);
}

void Mux_2Init(){ //done
	PinInit(MUX_2_PIN, MUX_2_PORT, MUX_2_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	GPIO_SetBits(MUX_2_PORT, MUX_2_PIN);
}

void InductionInit(){ //done
	PinInit(INDUCTION_PIN, INDUCTION_PORT, INDUCTION_CLK, NULL, NULL, NULL, NULL, NULL, 0);
}

void NVIC_Config(void){//done

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UsartInit(){ //done

	
	BufferInit(&U3Rx);
	BufferInit(&U3Tx);

	NVIC_Config();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART3, &USART_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	
}

void I2C_1Init(){} //lol doesnt work

void I2C_2Init(){} //lol doesnt work

void HeartInit(){ //works
	PinInit(HEART_BEAT_PIN, HEART_BEAT_PORT, HEART_BEAT_CLK, NULL, NULL, NULL, NULL, NULL, 1);
}

void TxLEDInit(){ //works
	PinInit(COM_TX_PIN, COM_TX_PORT, COM_TX_CLK, NULL, NULL, NULL, NULL, NULL, 1);
}

void RxLEDInit(){ //works
	PinInit(COM_RX_PIN, COM_RX_PORT, COM_RX_CLK, NULL, NULL, NULL, NULL, NULL, 1);
}

void ShiftInit(){ //done
	PinInit(SER_EN_PIN, SER_EN_PORT, SER_EN_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	PinInit(SER_IN_PIN, SER_IN_PORT, SER_IN_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	PinInit(SER_CLR_PIN, SER_CLR_PORT, SER_CLR_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	/*
	RCC_AHB1PeriphClockCmd(SER_CLR_CLK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  SER_CLR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//I think the same in all cases
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(SER_CLR_PORT, &GPIO_InitStructure);
	*/
	
	
	PinInit(SER_CLK_PIN, SER_CLK_PORT, SER_CLK_CLK, NULL, NULL, NULL, NULL, NULL, 1);
	GPIO_SetBits(SER_EN_PORT, SER_EN_PIN);
	GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
	GPIO_ResetBits(SER_CLR_PORT, SER_CLR_PIN);
	GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
	
	ShiftControl(0,0,1,0);
}





/****************** Control functions ******************/

void delay(int i){ //100 000 is about 1/4th a second													//100 000 000 is about 2.5 sec
	while (i-- > 0) {
		asm("nop");													/* This stops it optimising code out */
	}
}

//works when the buffer stuff works...
void ShiftControl( volatile uint8_t manip, volatile uint8_t tape, volatile uint8_t patch, volatile uint8_t lift){
	
	uint16_t d = 10;
	//printf("%d %d %d %d\n", manip, tape, patch, lift);
	//USART_SendData(USART3, manip);
	//USART_SendData(USART3, tape);
	//USART_SendData(USART3, patch);
	//USART_SendData(USART3, lift);
	//USART_SendData(USART3, 0x91);
	
	//1 and 8 - lift bag
	//2 and 7 - patch rack
	//3 and 6 - tape
	//4 and 5 - manip
	
	/*
	int i=0;
	for(i=0; i<16;i++){
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}
	*/
	
	if(lift == 0){
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}	
	
	if(patch == 0){
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}
	
	if(tape == 0){
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}
	
	if(manip == 0){
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}		

//side 2
	
	if(manip == 0){
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}

	if(tape == 0){
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}
	
	if(patch == 0){
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}	

	if(lift == 0){
		GPIO_SetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}else{
		GPIO_ResetBits(SER_IN_PORT, SER_IN_PIN);
		delay(d);
		
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	}	
		GPIO_SetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
		GPIO_ResetBits(SER_CLK_PORT, SER_CLK_PIN);
		delay(d);
	delay(1000000);
}

void MotorSet(int Motor, int Dir, int Speed){


  //Motor 1A aka motor 1
	if(Motor == 1){
		if (Dir ==1){
			//forward
			GPIO_SetBits(A_1_DIR_PORT, A_1_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(A_1_DIR_PORT, A_1_DIR_PIN);
		}
		MOTOR1_TIM->CCR1 = Speed;
	}
	
	//Motor 1B aka motor 2
	if(Motor == 2){
		if (Dir ==1){
			//forward
			GPIO_SetBits(B_1_DIR_PORT, B_1_DIR_PIN);
		}
		else{
			//Backward
			GPIO_ResetBits(B_1_DIR_PORT, B_1_DIR_PIN);
		}
		MOTOR1_TIM->CCR2 = Speed;
	}
	
	//Motor 2A aka motor 3
	if(Motor == 3){
		if (Dir ==1){
			//forward
			GPIO_SetBits(A_2_DIR_PORT, A_2_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(A_2_DIR_PORT, A_2_DIR_PIN);
		}
		MOTOR2_TIM->CCR1 = Speed;
	}
	
	//Motor 2B aka motor 4
	if(Motor == 4){
		if (Dir ==1){
			//forward
			GPIO_SetBits(B_2_DIR_PORT, B_2_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(B_2_DIR_PORT, B_2_DIR_PIN);
		}
		MOTOR2_TIM->CCR2 = Speed;
	}
	
	//Motor 3A aka motor 5
	if(Motor == 5){
		if (Dir ==1){
			//forward
			GPIO_SetBits(A_3_DIR_PORT, A_3_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(A_3_DIR_PORT, A_3_DIR_PIN);
		}
		MOTOR3_TIM->CCR1 = Speed;
	}
	
	//Motor 3B aka motor 6
	if(Motor == 6){
		if (Dir ==1){
			//forward
			GPIO_SetBits(B_3_DIR_PORT, B_3_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(B_3_DIR_PORT, B_3_DIR_PIN);
		}
		MOTOR3_TIM->CCR2 = Speed;
	}
	
	//Motor 5A aka motor 7
	if(Motor == 7){
		if (Dir ==1){
			//forward
			GPIO_SetBits(A_5_DIR_PORT, A_5_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(A_5_DIR_PORT, A_5_DIR_PIN);
		}
		MOTOR5_TIM->CCR3 = Speed;
	}
	
	//Motor 5B aka motor 8
	if(Motor == 8){
		if (Dir ==1){
			//forward
			GPIO_SetBits(B_5_DIR_PORT, B_5_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(B_5_DIR_PORT, B_5_DIR_PIN);
		}
		MOTOR5_TIM->CCR4 = Speed;
	}
	
	//Motor 4A aka motor 9
	if(Motor == 9){
		if (Dir ==1){
			//forward
			GPIO_SetBits(A_4_DIR_PORT, A_4_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(A_4_DIR_PORT, A_4_DIR_PIN);
		}
		MOTOR4_TIM->CCR1 = Speed;
	}
	
	//Motor 4B aka motor 10
	if(Motor == 10){
		if (Dir ==1){
			//forward
			GPIO_SetBits(B_4_DIR_PORT, B_4_DIR_PIN);
		}
		else{
		    //reverse
			GPIO_ResetBits(B_4_DIR_PORT, B_4_DIR_PIN);
		}
		MOTOR4_TIM->CCR2 = Speed;
	}
}

void MotorControl( volatile uint8_t M3, volatile uint8_t M8, volatile uint8_t M1, volatile uint8_t M6, volatile uint8_t M2, volatile uint8_t M5, volatile uint8_t M4, volatile uint8_t M7, volatile uint8_t M9, volatile uint8_t M10){
	uint16_t period = 	10000 / 2000;

	
	//-100 = 156
	//-25 = 231
	
	if(M1<=100){
		MotorSet(1, 0, period * M1 );
	}else if(M1>=156){
		MotorSet(1, 1, period * ( 100 - (M1 -156)));
	}
	
	if(M2<=100){
		MotorSet(2, 0, period * M2 );
	}else if(M2>=156){
		MotorSet(2, 1, period * ( 100 - (M2 -156)));
	}
	
	if(M3<=100){
		MotorSet(3, 0, period * M3);
	}else if(M3>=156){
		MotorSet(3, 1, period * ( 100 - (M3 -156)));
	}
	
	if(M4<=100){
		MotorSet(4, 0, period * M4);
	}else if(M4>=156){
		MotorSet(4, 1, period * ( 100 - (M4 -156)));
	}
	
	if(M5<=100){
		MotorSet(5, 0, period * M5);
	}else if(M5>=156){
		MotorSet(5, 1, period * ( 100 - (M5 -156)));
	}
	
	if(M6<=100){
		MotorSet(6, 0, period * M6);
	}else if(M6>=156){
		MotorSet(6, 1, period * ( 100 - (M6 -156)));
	}
	
	if(M7<=100){
		MotorSet(7, 0, period * M7);
	}else if(M7>=156){
		MotorSet(7, 1, period * ( 100 - (M7 -156)));
	}
	
	if(M8<=100){
		MotorSet(8, 0, period * M8);
	}else if(M8>=156){
		MotorSet(8, 1, period * ( 100 - (M8 -156)));
	}
	
	if(M9<=100){
		MotorSet(9, 0, period * M9 );
	}else if(M9>=156){
		MotorSet(9, 1, period * ( 100 - (M9 -156)));
	}
	
	if(M10<=100){
		MotorSet(10, 0, period * M10 );
	}else if(M10>=156){
		MotorSet(10, 1, period * ( 100 - (M10 -156)));
	}
}

void LedControl(volatile uint8_t R, volatile uint8_t G, volatile uint8_t B){//need to test..
	uint16_t period = 1000000 / 20000;
					 // 10000000 / 2000
	
	TIM8->CCR1 = (period * (R));
	TIM8->CCR2 = (period * (G));
	TIM8->CCR3 = (period * (B));
}

void MUXControl(volatile uint8_t M1, volatile uint8_t M2){
	if(M1 == 1){
		GPIO_SetBits(MUX_1_PORT, MUX_1_PIN);
	}else{
		GPIO_ResetBits(MUX_1_PORT, MUX_1_PIN);
	}

	if(M2 == 1){
		GPIO_SetBits(MUX_2_PORT, MUX_2_PIN);
	}else{
		GPIO_ResetBits(MUX_2_PORT, MUX_2_PIN);
	}
}

/****************** Usart & Buffer Functions ******************/
void Usart3Put(uint8_t ch){
	//put char to the buffer
	BufferPut(&U3Tx, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

uint8_t Usart3Get(void){
	uint8_t ch;
	//check if buffer is empty
	while (BufferIsEmpty(U3Rx) ==SUCCESS);
	BufferGet(&U3Rx, &ch);
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
//	return ERROR;//buffer full
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
