//Copyright 2012 Purdue University ROV Team
//Code by Clement Lan, Tyler Reid

//Please ask permission (clement.lan@gmail.com, reid2@purdue.edu, sbaklor@purdue.edu)
//before using this code

#include "stdafx.h"
#include "BaseStation.h"



using namespace ROV_BaseStation;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 
	
	// Create the main window and run it
	Application::Run(gcnew BaseStation());
	return 0;
}
