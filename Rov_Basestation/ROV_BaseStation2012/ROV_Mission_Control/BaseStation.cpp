//Copyright 2013 Purdue University ROV Team
//Code by Clement Lan, Nick Molo

//Please ask permission (clement.lan@gmail.com, nmolo@purdue.edu)
//before using this code


#include "stdafx.h"
#include "BaseStation.h"


ROV_BaseStation::BaseStation::BaseStation(void) {//needs cleaning
//------------------- Component Initializations -------------------------------
	InitializeComponent();
	vecstatus = false; //needs to be as early as possible to prevent exceptions when reading from serialport
	controller0 = new XBoxController(0);
	controller1 = new XBoxController(1);
	UpdateConnection();
	CheckControllerStatus();
	String ^ out = "";
//-------------------- Array Allocation ---------------------------------------
	inputs = (int *)malloc(sizeof(int)*10);
	thrust_vec = (int *)malloc(sizeof(int)*11);
	polar_vec = (int *)malloc(sizeof(int)*2);
	prev_thrust_vec = (int *)malloc(sizeof(int)*11);
	gyro_data = (short *)malloc(sizeof(short)*4);
	accel_data = (short *)malloc(sizeof(short)*3);
	mag_data = (short *)malloc(sizeof(short)*3);
	pb_data = (int *)malloc(sizeof(int)*3);
	rcvstr = (char *)(malloc(sizeof(char)*9));
	faultdata = (char *)malloc(sizeof(char)*2);
//-------------------- Array Initialization -----------------------------------
	for (int i = 0; i < 11; i++) {
		thrust_vec[i] = 0;
		prev_thrust_vec[i] = 0;
	}
	for (int i = 0; i < 3; i++) {
		pb_data[i] = 0;
		gyro_data[i] = 0;
		accel_data[i] = 0;
		mag_data[i] = 0;
	} gyro_data[3] = 0;

	for(int i = 0; i < 8; i++){
		rcvstr[i] = 0;
	}
	faultdata[0] = 0x00;
	faultdata[1] = 0x00;
//------------------- Integer/Float Initializations ----------------------------
	
	SpeedDivider_H = (float)(HorizLimiter->Value)/10;
	SpeedDivider_V = (float)(VertLimiter->Value)/10;
	StrafeSpeed = (float)(StrafeSpeedCTL->Value)/10;
	PitchRollFactor = (float)(PitchRollBar->Value)/10;
	nLX = 0; nLY = 0;
	nRX = 0; nRY = 0;
	LX = 0; LY = 0;
	nZ = 0;
	depth = 0;
	runtimeus = 0;
	depthoffset = 0;
	fifteentimer = 900000;
	prevrange = 0;
	startCount = 0;
	selectCount = 0;
	bCount = 0;
	yCount = 0;
	timercount = 0;

//----------------------- Booleans Initialization -------------------------------
	cam_toggleF = false;//For the front/ periscope camera
	cam_toggleR = false; //For the rear/ tool camera
	manip_open = false; //True if open false if closed
	//led_on = false; //LED True if on false if Off
	//tape_down = false; //Tape Control
	//patch_open = false;// Patch Rack
	//lift_open = false; //Lift bag control
	estop = false; //ESTOP!!!!
	vecstatus = true; //set true at end of setup so program knows its OK to put values into arrays
	prox = false;
	unitState = false;
	serialstart = false;
	bool missiontime = false;
}

System::Void ROV_BaseStation::BaseStation::CheckControllerStatus() {
	//check controller0 connection status
	if (!controller0->isConnected()) {
		this->C1Status -> Text = gcnew String("Controller Disconnected");
		this->C1Status -> ForeColor = Color::Red;
	}else{
		this->C1Status -> Text = gcnew String("Controller Connected");
		this->C1Status -> ForeColor = Color::Black;
	}
}
System::Void ROV_BaseStation::BaseStation::timer1_Tick(System::Object^  sender, System::EventArgs^  e) { //Needs cleaned

	inputs[0] = (short)(controller0->GetState().Gamepad.sThumbLX*SpeedDivider_H); //left stick x
	inputs[1] = (short)(controller0->GetState().Gamepad.sThumbLY*SpeedDivider_H); //left stick y
	inputs[2] = (short)(controller0->GetState().Gamepad.sThumbRX*SpeedDivider_V*PitchRollFactor); //right stick x
	inputs[3] = (short)(controller0->GetState().Gamepad.sThumbRY*SpeedDivider_V*PitchRollFactor); //right stick y
	inputs[4] = (short)(controller0->GetState().Gamepad.bLeftTrigger*SpeedDivider_V); //left trigger
	inputs[5] = (short)(controller0->GetState().Gamepad.bRightTrigger*SpeedDivider_V); //right trigger

	WORD wbuttons0 = controller0->GetState().Gamepad.wButtons;

	if (wbuttons0 & XINPUT_GAMEPAD_START) { //Toggle E-Stop
		startCount++;
		if(!estop && startCount >= 25){
			estop = true;
			estopDisp -> Text = gcnew String("ESTOP");
			startCount = 0;
		}
		else if(estop && startCount >= 30){
			estop = false;
			estopDisp -> Text = gcnew String("");
			startCount = 0;
		}
	}
	if (wbuttons0 & XINPUT_GAMEPAD_BACK) { //Toggle Speed mode
		selectCount++;
		if(!speed && selectCount >= 25){
			speed = true;
			estopDisp -> Text = gcnew String("LOW SPEED");
			HorizLimiter->Value=8;
			VertLimiter->Value=8;
			StrafeSpeedCTL->Value=8;
			SpeedDivider_H = (float)(HorizLimiter->Value)/10;
			SpeedDivider_V = (float)(VertLimiter->Value)/10;
			StrafeSpeed = (float)(StrafeSpeedCTL->Value)/10;
			selectCount = 0;
		}
		else if(speed && selectCount >= 30){
			speed = false;
			estopDisp -> Text = gcnew String("");
			HorizLimiter->Value=10;
			VertLimiter->Value=10;
			StrafeSpeedCTL->Value=10;
			SpeedDivider_H = (float)(HorizLimiter->Value)/10;
			SpeedDivider_V = (float)(VertLimiter->Value)/10;
			StrafeSpeed = (float)(StrafeSpeedCTL->Value)/10;
			selectCount = 0;
		}
	}



	if (wbuttons0 & XINPUT_GAMEPAD_B) {//Lift bag Toggle
		//bCount++;
		//if(!lift_open && bCount >= 50){
		//	lift_open = true;
		//	LiftStatus -> Text = gcnew String("Locked");
		//	bCount = 0;
		//}
		//else if(lift_open && bCount >= 50){
		//	lift_open = false;
		//	LiftStatus -> Text = gcnew String("Unlocked");
		//	bCount = 0;
		//}
	} 

	if (wbuttons0 & XINPUT_GAMEPAD_X) {//opens manip
		if (manip_open == false) {
			manip_open = true;
			ManipStatus->Text = gcnew String("Open");
		}
	} 

	if (wbuttons0 & XINPUT_GAMEPAD_A) {//closes manip
		if (manip_open == true) {
			manip_open = false;
			ManipStatus->Text = gcnew String("Closed");
		}
	} 

	if ( wbuttons0 & XINPUT_GAMEPAD_RIGHT_THUMB ) {//Camera set 1
		lsCount++;
		if(lsCount >= 25){
			cam_toggleF = !cam_toggleF;
			lsCount = 0;
		}
	} 
	if ( wbuttons0 & XINPUT_GAMEPAD_LEFT_THUMB) {//Camera Set2
		rsCount++;
		if(rsCount >= 25){
			cam_toggleR = !cam_toggleR;
			rsCount = 0;
		}
	} 
	if (wbuttons0 & XINPUT_GAMEPAD_Y) {//LED toggle
		//if(led_on == false && yCount > 25){
		//	led_on = true;
		//
		//	yCount = 0;
		//}
		//else if(led_on == true && yCount > 25){
		//	led_on = false;
		//	
		//	yCount = 0;
		//}
		//yCount++;
	}

	if (wbuttons0 & XINPUT_GAMEPAD_DPAD_DOWN) {//Tape measure deploy
		//if(!tape_down){
		//	tape_down = true;
		//	TapeStatus -> Text = gcnew String("Deployed");
		//}
	} 

	if (wbuttons0 & XINPUT_GAMEPAD_DPAD_UP) {// Tape Measure retract
		//if (tape_down){
		//	tape_down = false;
		//	TapeStatus -> Text = gcnew String("Retracted");
		//}
	} 

	if (wbuttons0 & XINPUT_GAMEPAD_DPAD_LEFT) { //Close patch rack
		//if (patch_open){
		//	patch_open = false;
		//	PatchStatus -> Text = gcnew String("Locked");
		//}
	} 

	if (wbuttons0 & XINPUT_GAMEPAD_DPAD_RIGHT) { //Open Patch Rack
		//if (!patch_open){
		//	patch_open = true;
		//	PatchStatus -> Text = gcnew String("Unlocked");
		//}
	} 

  //check to see if both shoulders are pressed
	if ( wbuttons0 & XINPUT_GAMEPAD_LEFT_SHOULDER && wbuttons0 & XINPUT_GAMEPAD_RIGHT_SHOULDER ) {
		wbuttons0 = wbuttons0&(~XINPUT_GAMEPAD_LEFT_SHOULDER);
		wbuttons0 = wbuttons0&(~XINPUT_GAMEPAD_RIGHT_SHOULDER);
	}

  //check for "deadzone"
	for (int i = 0; i < 4; i++) {if (abs(inputs[i]) < INPUT_DEADZONE) inputs[i] = 0; }
	for (int i = 4; i < 6; i++) {if (abs(inputs[i]) < TRIGGER_THRESH) inputs[i] = 0; }
	for (int i = 7; i < 9; i++) {if (abs(inputs[i]) < TRIGGER_THRESH) inputs[i] = 0; }

//	red = redScroll->Value*(Convert::ToInt32(led_on));
//	green = greenScroll->Value*(Convert::ToInt32(led_on));
//	blue = blueScroll->Value*(Convert::ToInt32(led_on));

//	if(prox == true){
//		this -> proxSensor -> BackColor = Color::Red;
//		controller0->vibrate(60000,60000);
//	}
//	if(prox == false){
//		this -> proxSensor -> BackColor = Color::Green;
	//	controller0->vibrate(0,0);
	//}
	GenerateThrusterVec(inputs);
	AdjustVector();
	UpdateGuiThrust();
	if(serialstart){
		SendThrusterVec();
	}
	else if(!serialstart){
		this -> serialLabel -> Text = "Serial Disconnected";
	}
	out = "";
	
}

System::Void ROV_BaseStation::BaseStation::timer2_Tick(System::Object^  sender, System::EventArgs^  e) {
	timercount++;
	int sec = timercount % 60;
	int min = ((timercount - sec) / 60) % 60;
    this -> TimeMin -> Text = min.ToString();
	this -> TimeSec -> Text = sec.ToString();
}

//actually generates the thruster vectors based on input 
System::Void ROV_BaseStation::BaseStation::GenerateMainThrusterVec(void){
	//generate Polar vector
//	polar_vec[0] = Math.Sqrt(Math.Pow(LX, 2) + Math.Pow(LY, 2));
//	polar_vec[1] = Math.Atan2(LY, LX);


  //calculate normalized values
	nLX = inputs[0]/(float)(32767);
	nLY = inputs[1]/(float)(32767);
	nRX = inputs[2]/(float)(32767);
	nRY = inputs[3]/(float)(32767);
	nZ = ((float)(inputs[5]-inputs[4]))/255;

	//[LF, RF, LB, RB, LFV, RFV, LBV, RBV, Cap, toggles, debug]
	//calculate horizontal thrusters
	//LF = (ForwardBack+LeftRight-LStrafe/2+RStrafe/2)/Nonzeros*255
	thrust_vec[0] = (int)GetThrust(nLX, nLY);
	//RF = (ForwardBack-LeftRight+LeftStrafe/2-RStrafe/2)/Nonzeros*255
	thrust_vec[1] = (int)GetThrust(-nLX, nLY);
	//LB = (ForwardBack+LeftRight+LStrafe/2-RStrafe/2)/Nonzeros*255
	thrust_vec[2] = (int)GetThrust(nLX, nLY);
	//RB = (ForwardBack-LeftRight-LStrafe/2+RStrafe/2)/Nonzeros*255
	thrust_vec[3] = (int)GetThrust(-nLX, nLY);
	//Front thrusters are backwards.
	//calculate vertical thrusters: LFV, RFV, LBV, RBV
	//LFV
	thrust_vec[4] = (int)GetThrustV(nZ, nRX, -nRY);
	//RFV
	thrust_vec[5] = (int)GetThrustV(nZ, -nRX, -nRY);
	//LBV
	thrust_vec[6] = (int)GetThrustV(nZ, nRX, nRY);
	//RBV
	thrust_vec[7] = (int)GetThrustV(nZ, -nRX, nRY);

	for (int i = 0; i < 8; i++) {
		if (thrust_vec[i] > 0) thrust_vec[i] += 40;
		else if (thrust_vec[i] < 0) thrust_vec[i] -= 40;
	}

	WORD wbuttons0 = controller0->GetState().Gamepad.wButtons;
	if ( wbuttons0 & XINPUT_GAMEPAD_RIGHT_SHOULDER ) {
		thrust_vec[0] += (int)(100*StrafeSpeed);
		thrust_vec[1] -= (int)(100*StrafeSpeed);
		thrust_vec[2] -= (int)(100*StrafeSpeed);
		thrust_vec[3] += (int)(100*StrafeSpeed);
		//thrust_vec[3] = -thrust_vec[3];
    strafing = true; //set strafe to true so you know that it needs to be averaged in
	} else if ( wbuttons0 & XINPUT_GAMEPAD_LEFT_SHOULDER ) {
		thrust_vec[0] -= (int)(100*StrafeSpeed);
		thrust_vec[1] += (int)(100*StrafeSpeed);
		thrust_vec[2] += (int)(100*StrafeSpeed);
		thrust_vec[3] -= (int)(100*StrafeSpeed);
		//thrust_vec[3] = -thrust_vec[3];
    strafing = true;
	} else strafing = false;

	if (strafing == true) {
			for (int i = 0; i < 4; i++) {
				if (thrust_vec[i] > 100) thrust_vec[i] = 100;
				else if (thrust_vec[i] < -100) thrust_vec[i] = -100;
			}
	}

	//flip front motors
	thrust_vec[0] = -thrust_vec[0];
	thrust_vec[1] = -thrust_vec[1];
	//for(int i = 0; i<8; i++){
		//if (thrust_vec[i] == 12) thrust_vec[i] =13;
	//}
}

System::Void ROV_BaseStation::BaseStation::UpdateConnection() {
try {
		this->serialPort1->PortName = this->CPortBox->Text;
		this->serialPort1->Open();
	} catch (Exception ^) {}
	if (serialPort1->IsOpen) {
		serialstart = true;
		this->serialLabel->ForeColor = Color::Black;
		this->serialLabel -> Text = gcnew String("");
	}
	if (!serialPort1->IsOpen) {
		serialstart = false;
		this->serialLabel -> Text = gcnew String("Serial Disconnected");
		this->serialLabel->ForeColor = Color::Red;
	}
}

//checks things and makes sure you dont overcurrent the motors, Limits things
System::Void ROV_BaseStation::BaseStation::GenerateThrusterVec(int* inputs) {
	
	GenerateMainThrusterVec();

	int MaximumChangeBar = 10;
	for (int i = 0; i < 9; i++) {
		if (abs(thrust_vec[i]-prev_thrust_vec[i]) > MaximumChangeBar) {
			if (thrust_vec[i] >= prev_thrust_vec[i]) { //i.e. going from 40->70
				thrust_vec[i] = prev_thrust_vec[i]+MaximumChangeBar;
			} else if (thrust_vec[i] < prev_thrust_vec[i]){ //i.e. going from 60->45
				thrust_vec[i] = prev_thrust_vec[i]-MaximumChangeBar;
			}
		}
	}

	if (DisableLF->Checked == true) thrust_vec[0] = 0;
	if (DisableRF->Checked == true) thrust_vec[1] = 0;
	if (DisableLB->Checked == true) thrust_vec[2] = 0;
	if (DisableRB->Checked == true) thrust_vec[3] = 0;
	if (DisableLFV->Checked == true) thrust_vec[4] = 0;
	if (DisableRFV->Checked == true) thrust_vec[5] = 0;
	if (DisableLBV->Checked == true) thrust_vec[6] = 0;
	if (DisableRBV->Checked == true) thrust_vec[7] = 0;

	if (estop) { //go into standby mode (disable pins)
		for (int i = 0; i < 9; i++) { //for each of the 9 motors
			thrust_vec[i] = 0;//set 8th bit on each byte w/ 50% duty cycle
		}
	}

	for (int i = 0; i < 11; i++) { //copy into prev_thrust_vec
		prev_thrust_vec[i] = thrust_vec[i];
	}

}

System::Void ROV_BaseStation::BaseStation::UpdateGuiThrust() {	
	LFThrust->Text = gcnew String(Convert::ToString(thrust_vec[0]));
	RFThrust->Text = gcnew String(Convert::ToString(thrust_vec[1]));
	LBThrust->Text = gcnew String(Convert::ToString(thrust_vec[2]));
	RBThrust->Text = gcnew String(Convert::ToString(thrust_vec[3]));
	LFVThrust->Text = gcnew String(Convert::ToString(thrust_vec[4]));
	RFVThrust->Text = gcnew String(Convert::ToString(thrust_vec[5]));
	LBVThrust->Text = gcnew String(Convert::ToString(thrust_vec[6]));
	RBVThrust->Text = gcnew String(Convert::ToString(thrust_vec[7]));
	this -> recValues -> Text = out;
}


System::Void ROV_BaseStation::BaseStation::AdjustVector() {
	//50% is the center, i.e. 0 thrust
	for (int i = 0; i < 8; i++) {
		if(thrust_vec[i] > 0 && thrust_vec[i] < 40){
			thrust_vec[i] = 40;
		}
		if(thrust_vec[i] < 0 && thrust_vec[i] > -40){
			thrust_vec[i] = -40;
		}
	}
  //thrust_vec[9] = (int)((this->thrust_vec[9])/100*(254));
}


int ROV_BaseStation::BaseStation::GetHorizNonZero(){
	int n = 0;
	for(int i = 0; i < 2; i++)
	{
		if (abs(this->inputs[i]) > 0) n++;
	}

	if (n == 0) return 1; else return n;
}


int ROV_BaseStation::BaseStation::GetVertNonZero(){
	int n = 0;
	for (int i = 2; i < 6; i++) {
		if (abs(this->inputs[i]) > 0) n++;
	}
	if (n == 0) return 1; else return n;
}

System::Void ROV_BaseStation::BaseStation::SendThrusterVec() {
  //copy vector into a char array
	array<unsigned char> ^sendarr = gcnew array<unsigned char>(19);
	sendarr[0] = (char)(12);
	sendarr[18] = (char)(13);
	for(int i = 1; i < 9; i++) {
		sendarr[i] = (char)(this->thrust_vec[i-1]);
		if (i <= 8) sendarr[i]*(Convert::ToInt32(estop));
	}
	sendarr[9] = (char)this->manip_open;
//	sendarr[10] = (char)this->tape_down;
//	sendarr[11] = (char)this ->patch_open;
//	sendarr[12] = (char)this->lift_open;
	sendarr[13] = (char)this->cam_toggleF;
	sendarr[14] = (char)this->cam_toggleR;
	sendarr[15] = (char)this->red;
	sendarr[16] = (char)this->green;
	sendarr[17] = (char)this->blue;
	
	String ^ out;
	for(int i = 0; i < 19; i++){
		out = out+" "+(char)sendarr[i];
	}
	this -> serialLabel ->Text = out;
	this -> SerialDebug -> Text = out;
	this -> SerialLabel2 -> Text = out; 
	if (this->serialPort1->IsOpen) {
		this->serialPort1->Write(sendarr, 0, 19);
	}
}

System::Void ROV_BaseStation::BaseStation::serialPort1_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
  int data;
  if(serialstart){
	  if (this->serialPort1->BytesToRead > 0) {
		data = (char)(this->serialPort1->ReadByte());//read leading byte
		//this->proxSensor->BackColor=Color::Green;
		if (data == (char)(18)) {
		
 			while(this->serialPort1->BytesToRead < 3);
			   //rcvstr[0] = (char)(13);
 			for (int i = 0; i < 2; i++) { 
  					rcvstr[i] = (this->serialPort1->ReadByte()); 
			}
			if(rcvstr[0] == 48){
				prox = false;
			}
			else if(rcvstr[0] == 49){
				prox = true;
			}
			for(int i = 0; i < 2; i++){
				out = out+" "+(char)rcvstr[i];
			}
		} 
	  }
  }
}