//Copyright 2013 Purdue University ROV Team
//Code by Clement Lan, Nick Molo

//Please ask permission (clement.lan@gmail.com, nmolo@purdue.edu)
//before using this code

#pragma once
#include <Windows.h>
#include <XInput.h>

#pragma comment(lib, "XINPUT9_1_0.LIB")
#define INPUT_DEADZONE  ( 0.24f * FLOAT(0x7FFF) )
#define TRIGGER_THRESH 20

class XBoxController
{
public:
	XBoxController(int id);
	XINPUT_STATE GetState();
	bool isConnected();
	void vibrate(int leftVal, int rightVal);
private:
	int controllerID;
	XINPUT_STATE controllerState;
};

