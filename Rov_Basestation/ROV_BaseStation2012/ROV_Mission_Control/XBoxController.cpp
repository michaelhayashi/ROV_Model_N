//Copyright 2012 Purdue University ROV Team
//Code by Clement Lan, Tyler Reid

//Please ask permission (clement.lan@gmail.com, reid2@purdue.edu, sbaklor@purdue.edu)
//before using this code


#include "StdAfx.h"
#include "XBoxController.h"


XBoxController::XBoxController(int id)
{
	controllerID = id;
}

XINPUT_STATE XBoxController::GetState()
{
	// Zeroise the state
    ZeroMemory(&controllerState, sizeof(XINPUT_STATE));

    // Get the state
    XInputGetState(controllerID, &controllerState);

    return this->controllerState;
}

bool XBoxController::isConnected()
{
	// Zeroise the state
    ZeroMemory(&controllerState, sizeof(XINPUT_STATE));

    // Get the state
    DWORD Result = XInputGetState(controllerID, &controllerState);

    if(Result == ERROR_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void XBoxController::vibrate(int leftVal, int rightVal)
{
    // Create a Vibraton State
    XINPUT_VIBRATION Vibration;

    // Zeroise the Vibration
    ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));

    // Set the Vibration Values
    Vibration.wLeftMotorSpeed = leftVal;
    Vibration.wRightMotorSpeed = rightVal;

    // Vibrate the controller
    XInputSetState(0, &Vibration);
}