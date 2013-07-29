#include "functions.h"
volatile FIFO_TypeDef U2Rx, U2Tx, U6Rx, U6Tx;
volatile uint8_t flag=0;


int main(void){

        SystemInit();
        
		LedInit();
        Usart6Init();
		
	while(1){

		//LF, RF, LB, RB, Nothing , RFV, LBV, RBV, nothing, LFV
		MotorControl(U2Rx.buff[1],U2Rx.buff[2],U2Rx.buff[3],U2Rx.buff[4],0,U2Rx.buff[6],U2Rx.buff[7],U2Rx.buff[8],0,U2Rx.buff[5]);
		ShiftControl(U2Rx.buff[9], U2Rx.buff[10],U2Rx.buff[11], U2Rx.buff[12]);
		MUXControl(U2Rx.buff[13],U2Rx.buff[14]);
	}
}
