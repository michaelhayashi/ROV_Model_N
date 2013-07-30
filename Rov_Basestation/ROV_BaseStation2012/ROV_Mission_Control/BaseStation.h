//Copyright 2013 Purdue University ROV Team
//Code by Clement Lan, Nick Molo

//Please ask permission (clement.lan@gmail.com, nmolo@purdue.edu)
//before using this code

#pragma once
#include "XBoxController.h" 


#define INPUT_DEADZONE  ( 0.24f * FLOAT(0x7FFF) )
#define TRIGGER_THRESH 20
#define ACK 13

namespace ROV_BaseStation {

	using namespace System;
	//using System::Math;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO::Ports;

	


	/// <summary>
	/// Summary for BaseStation
	/// </summary>
	public ref class BaseStation : public System::Windows::Forms::Form
	{
	public:
		BaseStation(void);

	protected:
		/// <summary>
		/// Clean up any resources being used
		/// </summary>
		~BaseStation()
		{
			if (serialPort1->IsOpen) serialPort1->Close();
			if (components)
			{
				delete components;
			}
		}

	public: System::IO::Ports::SerialPort^  serialPort1;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Label^  estopDisp;
	private: System::Windows::Forms::Label^  recValues;
	private: System::Windows::Forms::ComboBox^  CPortBox;
	private: System::Windows::Forms::Label^  serialLabel;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::GroupBox^  ToolPanel;
	private: System::Windows::Forms::Label^  ManipStatus;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  RFThrust;
	private: System::Windows::Forms::Label^  LFThrust;
	private: System::Windows::Forms::Label^  LBThrust;
	private: System::Windows::Forms::Label^  RBThrust;
	private: System::Windows::Forms::Label^  RBVThrust;
	private: System::Windows::Forms::Label^  LBVThrust;
	private: System::Windows::Forms::Label^  RFVThrust;
	private: System::Windows::Forms::Label^  LFVThrust;
	private: System::Windows::Forms::Panel^  panel1;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::TabPage^  Serial;
	private: System::Windows::Forms::TabPage^  ThrusterToggle;
	private: System::Windows::Forms::CheckBox^  DisableLBV;
	private: System::Windows::Forms::CheckBox^  DisableRBV;
	private: System::Windows::Forms::CheckBox^  DisableRFV;
	private: System::Windows::Forms::CheckBox^  DisableLFV;
	private: System::Windows::Forms::CheckBox^  DisableLB;
	private: System::Windows::Forms::CheckBox^  DisableRB;
	private: System::Windows::Forms::CheckBox^  DisableRF;
	private: System::Windows::Forms::CheckBox^  DisableLF;
	private: System::Windows::Forms::TabPage^  Main;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::TrackBar^  PitchRollBar;
	private: System::Windows::Forms::TrackBar^  StrafeSpeedCTL;
	private: System::Windows::Forms::TrackBar^  VertLimiter;
	private: System::Windows::Forms::TrackBar^  HorizLimiter;
	private: System::Windows::Forms::TabControl^  tabControl1;
	private: Microsoft::VisualBasic::PowerPacks::ShapeContainer^  shapeContainer2;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape1;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape2;
	private: Microsoft::VisualBasic::PowerPacks::Printing::PrintForm^  printForm1;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  Roll;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label9;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  Pitch;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  lineShape6;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  lineShape5;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  lineShape4;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  lineShape3;
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Label^  C2Status;
	private: System::Windows::Forms::Label^  C1Status;
	private: System::Windows::Forms::Button^  ControlCheck;
	private: System::Windows::Forms::CheckedListBox^  checkedListBox1;
	private: System::Windows::Forms::Label^  label18;
	private: System::Windows::Forms::Label^  label17;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label19;
	private: System::Windows::Forms::TabPage^  Debug;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::Label^  label20;
	private: System::Windows::Forms::Button^  TimerStart;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::Label^  TimeSec;
	private: System::Windows::Forms::Timer^  timer2;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::Label^  label24;
	private: System::Windows::Forms::Label^  label23;
	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::Label^  turner1;
	private: Microsoft::VisualBasic::PowerPacks::ShapeContainer^  shapeContainer1;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape6;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape5;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape4;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  ovalShape3;
	private: System::Windows::Forms::Label^  label25;
	private: System::Windows::Forms::TrackBar^  TurnerBar;
	private: System::Windows::Forms::Label^  SerialDebug;
	private: System::Windows::Forms::Button^  SerialForce;
	private: System::Windows::Forms::StatusStrip^  statusStrip1;
	private: System::Windows::Forms::ToolStripStatusLabel^  SerialLabel2;
	private: System::Windows::Forms::ToolStripStatusLabel^  PolarTest;
	private: System::Windows::Forms::TabPage^  Status;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::Label^  label34;
	private: System::Windows::Forms::Label^  label33;
	private: System::Windows::Forms::Label^  label32;
	private: System::Windows::Forms::Label^  label31;
	private: System::Windows::Forms::Label^  label30;
	private: System::Windows::Forms::Label^  label29;
	private: System::Windows::Forms::Label^  label28;
	private: System::Windows::Forms::Label^  label27;
	private: System::Windows::Forms::Label^  label26;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::DataGridView^  dataGridView1;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Motor;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Temperature;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Voltage;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  Current;
	private: System::Windows::Forms::DataGridViewTextBoxColumn^  FaultStatus;
	private: System::Windows::Forms::Button^  TimerReset;
	private: System::Windows::Forms::Label^  label36;
	private: System::Windows::Forms::Label^  TimeMin;
private: System::Windows::Forms::Button^  button5;
private: System::Windows::Forms::Button^  button4;
private: System::Windows::Forms::Button^  button3;
private: System::Windows::Forms::Button^  ManipToggle;



private: //function prototypes
		private: //function prototypes
		System::ComponentModel::IContainer^  components;
		System::Void SendThrusterVec();
		System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e);
		System::Void timer2_Tick(System::Object^  sender, System::EventArgs^  e);
		System::Void GenerateThrusterVec(int* inputs);
		System::Void CheckControllerStatus();
		System::Void UpdateConnection();
		System::Void UpdateGuiThrust();
		System::Void AdjustVector();
		int GetHorizNonZero();
		int GetVertNonZero();
		float GetThrust(float x, float y) { return (x+y)/GetHorizNonZero()*60; }
		float GetThrustV(float x, float y, float z) { return (x+y+z)/GetVertNonZero()*60; }
		System::Void UpdateStatButton_Click(System::Object^  sender, System::EventArgs^  e) {
			CheckControllerStatus();
		}
		System::Void VertLimiter_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
			SpeedDivider_V = (float)(this->VertLimiter->Value)/10;
		}
		System::Void StrafeSpeedCTL_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
			StrafeSpeed = (float)(this->StrafeSpeedCTL->Value)/10;
		}
		System::Void HorizLimiter_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
			SpeedDivider_H = (float)(this->HorizLimiter->Value)/10;
		}
		System::Void ConnectButton_Click(System::Object^  sender, System::EventArgs^  e) {
			 UpdateConnection();
		}
		System::Void serialPort1_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e);
		System::Void GenerateMainThrusterVec(void);
	private: //variables
	//Objects
		XBoxController *controller0;
		XBoxController *controller1;
		String ^thrustbackstr;
		String ^mbstring;
		String ^out;
	//Arrays
		int *thrust_vec;
		int *polar_vec;
		int *prev_thrust_vec;
		int *inputs;
		int *Bbuttonaverage;
		int *Abuttonaverage;
		int *Ybuttonaverage;
		char *rcvstr;
		int *DupAverage;
		int *DdownAverage;
		int* DleftAverage;
		int *Xbuttonaverage;
		int *DrightAverage;
		short *accel_data;
		short *gyro_data;
		short *mag_data;
		int *pb_data;
		char *faultdata;
	//Booleans
		bool cam_toggleF, cam_toggleR, manip_open, estop, speed;
		bool vecstatus;
		bool strafing;
		bool prox;
		bool serialstart;
		bool passcodeOK;
		bool unitState;
		bool missiontime;
	//Floats
		float SpeedDivider_H;
		float SpeedDivider_V;
		float StrafeSpeed;
		float PitchRollFactor;
		float depth, depthoffset;
		float nLX, nLY, nRY, nRX, nZ, LX, LY;
	//Ints
		int red;
		int green;
		int blue;
		int bCount;
		int lsCount;
		int rsCount;
		int startCount;
		int selectCount;
		int Dupcount;
		int Ddowncount;
		int Dleftcount;
		int Drightcount;
		int xcount;
		int yCount;
		int test_counter;
		int test_counter2;
		int runtimeus;
		long fifteentimer;
		int prevrange;
		int timercount;
		/// <summary>
		/// Required designer variable.
		/// </summary>



#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(BaseStation::typeid));
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->estopDisp = (gcnew System::Windows::Forms::Label());
			this->recValues = (gcnew System::Windows::Forms::Label());
			this->CPortBox = (gcnew System::Windows::Forms::ComboBox());
			this->serialLabel = (gcnew System::Windows::Forms::Label());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->ToolPanel = (gcnew System::Windows::Forms::GroupBox());
			this->ManipStatus = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->RFThrust = (gcnew System::Windows::Forms::Label());
			this->LFThrust = (gcnew System::Windows::Forms::Label());
			this->LBThrust = (gcnew System::Windows::Forms::Label());
			this->RBThrust = (gcnew System::Windows::Forms::Label());
			this->RBVThrust = (gcnew System::Windows::Forms::Label());
			this->LBVThrust = (gcnew System::Windows::Forms::Label());
			this->RFVThrust = (gcnew System::Windows::Forms::Label());
			this->LFVThrust = (gcnew System::Windows::Forms::Label());
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->shapeContainer2 = (gcnew Microsoft::VisualBasic::PowerPacks::ShapeContainer());
			this->lineShape6 = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->lineShape5 = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->lineShape4 = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->lineShape3 = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->Pitch = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->Roll = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->ovalShape1 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->ovalShape2 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->Serial = (gcnew System::Windows::Forms::TabPage());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->ControlCheck = (gcnew System::Windows::Forms::Button());
			this->C2Status = (gcnew System::Windows::Forms::Label());
			this->C1Status = (gcnew System::Windows::Forms::Label());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->ThrusterToggle = (gcnew System::Windows::Forms::TabPage());
			this->DisableLBV = (gcnew System::Windows::Forms::CheckBox());
			this->DisableRBV = (gcnew System::Windows::Forms::CheckBox());
			this->DisableRFV = (gcnew System::Windows::Forms::CheckBox());
			this->DisableLFV = (gcnew System::Windows::Forms::CheckBox());
			this->DisableLB = (gcnew System::Windows::Forms::CheckBox());
			this->DisableRB = (gcnew System::Windows::Forms::CheckBox());
			this->DisableRF = (gcnew System::Windows::Forms::CheckBox());
			this->DisableLF = (gcnew System::Windows::Forms::CheckBox());
			this->Main = (gcnew System::Windows::Forms::TabPage());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->TurnerBar = (gcnew System::Windows::Forms::TrackBar());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->PitchRollBar = (gcnew System::Windows::Forms::TrackBar());
			this->StrafeSpeedCTL = (gcnew System::Windows::Forms::TrackBar());
			this->VertLimiter = (gcnew System::Windows::Forms::TrackBar());
			this->HorizLimiter = (gcnew System::Windows::Forms::TrackBar());
			this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
			this->Status = (gcnew System::Windows::Forms::TabPage());
			this->dataGridView1 = (gcnew System::Windows::Forms::DataGridView());
			this->Motor = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->Temperature = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->Voltage = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->Current = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->FaultStatus = (gcnew System::Windows::Forms::DataGridViewTextBoxColumn());
			this->Debug = (gcnew System::Windows::Forms::TabPage());
			this->TimerReset = (gcnew System::Windows::Forms::Button());
			this->SerialForce = (gcnew System::Windows::Forms::Button());
			this->SerialDebug = (gcnew System::Windows::Forms::Label());
			this->printForm1 = (gcnew Microsoft::VisualBasic::PowerPacks::Printing::PrintForm(this->components));
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->label34 = (gcnew System::Windows::Forms::Label());
			this->label33 = (gcnew System::Windows::Forms::Label());
			this->label32 = (gcnew System::Windows::Forms::Label());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->checkedListBox1 = (gcnew System::Windows::Forms::CheckedListBox());
			this->TimerStart = (gcnew System::Windows::Forms::Button());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->label36 = (gcnew System::Windows::Forms::Label());
			this->TimeMin = (gcnew System::Windows::Forms::Label());
			this->TimeSec = (gcnew System::Windows::Forms::Label());
			this->timer2 = (gcnew System::Windows::Forms::Timer(this->components));
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->turner1 = (gcnew System::Windows::Forms::Label());
			this->shapeContainer1 = (gcnew Microsoft::VisualBasic::PowerPacks::ShapeContainer());
			this->ovalShape6 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->ovalShape5 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->ovalShape4 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->ovalShape3 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->statusStrip1 = (gcnew System::Windows::Forms::StatusStrip());
			this->PolarTest = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->SerialLabel2 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->ManipToggle = (gcnew System::Windows::Forms::Button());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->ToolPanel->SuspendLayout();
			this->panel1->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->Serial->SuspendLayout();
			this->groupBox4->SuspendLayout();
			this->groupBox3->SuspendLayout();
			this->ThrusterToggle->SuspendLayout();
			this->Main->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->TurnerBar))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->PitchRollBar))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->StrafeSpeedCTL))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->VertLimiter))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->HorizLimiter))->BeginInit();
			this->tabControl1->SuspendLayout();
			this->Status->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->dataGridView1))->BeginInit();
			this->Debug->SuspendLayout();
			this->groupBox5->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->groupBox6->SuspendLayout();
			this->statusStrip1->SuspendLayout();
			this->SuspendLayout();
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &BaseStation::timer1_Tick);
			// 
			// serialPort1
			// 
			this->serialPort1->BaudRate = 115200;
			this->serialPort1->PortName = L"COM5";
			this->serialPort1->ReadBufferSize = 256;
			this->serialPort1->WriteBufferSize = 256;
			this->serialPort1->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &BaseStation::serialPort1_DataReceived);
			// 
			// estopDisp
			// 
			this->estopDisp->AutoSize = true;
			this->estopDisp->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 25, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->estopDisp->ForeColor = System::Drawing::Color::Red;
			this->estopDisp->Location = System::Drawing::Point(12, 292);
			this->estopDisp->Name = L"estopDisp";
			this->estopDisp->Size = System::Drawing::Size(0, 39);
			this->estopDisp->TabIndex = 5;
			// 
			// recValues
			// 
			this->recValues->AutoSize = true;
			this->recValues->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->recValues->Location = System::Drawing::Point(86, 24);
			this->recValues->Name = L"recValues";
			this->recValues->Size = System::Drawing::Size(56, 16);
			this->recValues->TabIndex = 6;
			this->recValues->Text = L"Serial In";
			// 
			// CPortBox
			// 
			this->CPortBox->FormattingEnabled = true;
			this->CPortBox->Items->AddRange(gcnew cli::array< System::Object^  >(11) {L"COM0", L"COM1", L"COM2", L"COM3", L"COM4", L"COM5", 
				L"COM6", L"COM7", L"COM8", L"COM9", L"COM10"});
			this->CPortBox->Location = System::Drawing::Point(356, 12);
			this->CPortBox->Name = L"CPortBox";
			this->CPortBox->Size = System::Drawing::Size(134, 21);
			this->CPortBox->TabIndex = 0;
			this->CPortBox->Text = L"COM5";
			// 
			// serialLabel
			// 
			this->serialLabel->AutoSize = true;
			this->serialLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->serialLabel->Location = System::Drawing::Point(86, 52);
			this->serialLabel->Name = L"serialLabel";
			this->serialLabel->Size = System::Drawing::Size(66, 16);
			this->serialLabel->TabIndex = 8;
			this->serialLabel->Text = L"Serial Out";
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(356, 82);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(134, 35);
			this->button2->TabIndex = 9;
			this->button2->Text = L"Toggle Serial";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &BaseStation::button2_Click);
			// 
			// ToolPanel
			// 
			this->ToolPanel->BackColor = System::Drawing::Color::Transparent;
			this->ToolPanel->Controls->Add(this->ManipStatus);
			this->ToolPanel->Controls->Add(this->label1);
			this->ToolPanel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->ToolPanel->Location = System::Drawing::Point(534, 334);
			this->ToolPanel->Name = L"ToolPanel";
			this->ToolPanel->Size = System::Drawing::Size(143, 43);
			this->ToolPanel->TabIndex = 0;
			this->ToolPanel->TabStop = false;
			this->ToolPanel->Text = L"Tool Status";
			// 
			// ManipStatus
			// 
			this->ManipStatus->AutoSize = true;
			this->ManipStatus->Location = System::Drawing::Point(77, 20);
			this->ManipStatus->Name = L"ManipStatus";
			this->ManipStatus->Size = System::Drawing::Size(51, 16);
			this->ManipStatus->TabIndex = 3;
			this->ManipStatus->Text = L"Closed";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->BackColor = System::Drawing::Color::Transparent;
			this->label1->Location = System::Drawing::Point(2, 19);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(81, 16);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Manipulator:";
			// 
			// RFThrust
			// 
			this->RFThrust->AutoSize = true;
			this->RFThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->RFThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->RFThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->RFThrust->Location = System::Drawing::Point(317, 25);
			this->RFThrust->Name = L"RFThrust";
			this->RFThrust->Size = System::Drawing::Size(18, 20);
			this->RFThrust->TabIndex = 17;
			this->RFThrust->Text = L"0";
			// 
			// LFThrust
			// 
			this->LFThrust->AutoSize = true;
			this->LFThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->LFThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->LFThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->LFThrust->Location = System::Drawing::Point(40, 25);
			this->LFThrust->Name = L"LFThrust";
			this->LFThrust->Size = System::Drawing::Size(18, 20);
			this->LFThrust->TabIndex = 16;
			this->LFThrust->Text = L"0";
			// 
			// LBThrust
			// 
			this->LBThrust->AutoSize = true;
			this->LBThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->LBThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->LBThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->LBThrust->Location = System::Drawing::Point(40, 261);
			this->LBThrust->Name = L"LBThrust";
			this->LBThrust->Size = System::Drawing::Size(18, 20);
			this->LBThrust->TabIndex = 15;
			this->LBThrust->Text = L"0";
			// 
			// RBThrust
			// 
			this->RBThrust->AutoSize = true;
			this->RBThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->RBThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->RBThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->RBThrust->Location = System::Drawing::Point(317, 261);
			this->RBThrust->Name = L"RBThrust";
			this->RBThrust->Size = System::Drawing::Size(18, 20);
			this->RBThrust->TabIndex = 14;
			this->RBThrust->Text = L"0";
			// 
			// RBVThrust
			// 
			this->RBVThrust->AutoSize = true;
			this->RBVThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->RBVThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->RBVThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->RBVThrust->Location = System::Drawing::Point(273, 199);
			this->RBVThrust->Name = L"RBVThrust";
			this->RBVThrust->Size = System::Drawing::Size(18, 20);
			this->RBVThrust->TabIndex = 13;
			this->RBVThrust->Text = L"0";
			// 
			// LBVThrust
			// 
			this->LBVThrust->AutoSize = true;
			this->LBVThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->LBVThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->LBVThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->LBVThrust->Location = System::Drawing::Point(82, 199);
			this->LBVThrust->Name = L"LBVThrust";
			this->LBVThrust->Size = System::Drawing::Size(18, 20);
			this->LBVThrust->TabIndex = 12;
			this->LBVThrust->Text = L"0";
			// 
			// RFVThrust
			// 
			this->RFVThrust->AutoSize = true;
			this->RFVThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->RFVThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->RFVThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->RFVThrust->Location = System::Drawing::Point(273, 86);
			this->RFVThrust->Name = L"RFVThrust";
			this->RFVThrust->Size = System::Drawing::Size(18, 20);
			this->RFVThrust->TabIndex = 11;
			this->RFVThrust->Text = L"0";
			// 
			// LFVThrust
			// 
			this->LFVThrust->AutoSize = true;
			this->LFVThrust->BackColor = System::Drawing::SystemColors::ControlDark;
			this->LFVThrust->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->LFVThrust->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->LFVThrust->Location = System::Drawing::Point(82, 86);
			this->LFVThrust->Name = L"LFVThrust";
			this->LFVThrust->Size = System::Drawing::Size(18, 20);
			this->LFVThrust->TabIndex = 10;
			this->LFVThrust->Text = L"0";
			// 
			// panel1
			// 
			this->panel1->BackgroundImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"panel1.BackgroundImage")));
			this->panel1->BackgroundImageLayout = System::Windows::Forms::ImageLayout::Center;
			this->panel1->Controls->Add(this->LFThrust);
			this->panel1->Controls->Add(this->RBThrust);
			this->panel1->Controls->Add(this->LBThrust);
			this->panel1->Controls->Add(this->RBVThrust);
			this->panel1->Controls->Add(this->RFThrust);
			this->panel1->Controls->Add(this->LBVThrust);
			this->panel1->Controls->Add(this->LFVThrust);
			this->panel1->Controls->Add(this->RFVThrust);
			this->panel1->Location = System::Drawing::Point(683, 12);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(374, 307);
			this->panel1->TabIndex = 1;
			// 
			// groupBox1
			// 
			this->groupBox1->BackColor = System::Drawing::Color::Transparent;
			this->groupBox1->Controls->Add(this->label21);
			this->groupBox1->Controls->Add(this->label20);
			this->groupBox1->Controls->Add(this->label19);
			this->groupBox1->Controls->Add(this->label10);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->shapeContainer2);
			this->groupBox1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->groupBox1->Location = System::Drawing::Point(683, 334);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(373, 269);
			this->groupBox1->TabIndex = 10;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Roll and Pitch";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label21->Location = System::Drawing::Point(349, 140);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(16, 17);
			this->label21->TabIndex = 3;
			this->label21->Text = L"F";
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label20->Location = System::Drawing::Point(179, 140);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(18, 17);
			this->label20->TabIndex = 3;
			this->label20->Text = L"R";
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label19->Location = System::Drawing::Point(8, 141);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(16, 17);
			this->label19->TabIndex = 3;
			this->label19->Text = L"L";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label10->Location = System::Drawing::Point(244, 40);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(60, 25);
			this->label10->TabIndex = 2;
			this->label10->Text = L"Pitch";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label9->Location = System::Drawing::Point(63, 40);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(49, 25);
			this->label9->TabIndex = 1;
			this->label9->Text = L"Roll";
			// 
			// shapeContainer2
			// 
			this->shapeContainer2->Location = System::Drawing::Point(3, 18);
			this->shapeContainer2->Margin = System::Windows::Forms::Padding(0);
			this->shapeContainer2->Name = L"shapeContainer2";
			this->shapeContainer2->Shapes->AddRange(gcnew cli::array< Microsoft::VisualBasic::PowerPacks::Shape^  >(8) {this->lineShape6, 
				this->lineShape5, this->lineShape4, this->lineShape3, this->Pitch, this->Roll, this->ovalShape1, this->ovalShape2});
			this->shapeContainer2->Size = System::Drawing::Size(367, 248);
			this->shapeContainer2->TabIndex = 0;
			this->shapeContainer2->TabStop = false;
			// 
			// lineShape6
			// 
			this->lineShape6->BorderColor = System::Drawing::Color::White;
			this->lineShape6->Name = L"lineShape6";
			this->lineShape6->X1 = 273;
			this->lineShape6->X2 = 273;
			this->lineShape6->Y1 = 65;
			this->lineShape6->Y2 = 195;
			// 
			// lineShape5
			// 
			this->lineShape5->BorderColor = System::Drawing::Color::White;
			this->lineShape5->Name = L"lineShape5";
			this->lineShape5->X1 = 95;
			this->lineShape5->X2 = 95;
			this->lineShape5->Y1 = 65;
			this->lineShape5->Y2 = 195;
			// 
			// lineShape4
			// 
			this->lineShape4->BorderColor = System::Drawing::Color::White;
			this->lineShape4->Name = L"lineShape4";
			this->lineShape4->X1 = 209;
			this->lineShape4->X2 = 339;
			this->lineShape4->Y1 = 130;
			this->lineShape4->Y2 = 130;
			// 
			// lineShape3
			// 
			this->lineShape3->BorderColor = System::Drawing::Color::White;
			this->lineShape3->Name = L"lineShape3";
			this->lineShape3->X1 = 30;
			this->lineShape3->X2 = 160;
			this->lineShape3->Y1 = 130;
			this->lineShape3->Y2 = 130;
			// 
			// Pitch
			// 
			this->Pitch->BorderColor = System::Drawing::Color::DarkOrange;
			this->Pitch->BorderWidth = 4;
			this->Pitch->Name = L"Pitch";
			this->Pitch->X1 = 214;
			this->Pitch->X2 = 334;
			this->Pitch->Y1 = 130;
			this->Pitch->Y2 = 130;
			// 
			// Roll
			// 
			this->Roll->BorderColor = System::Drawing::Color::DarkOrange;
			this->Roll->BorderWidth = 4;
			this->Roll->Name = L"Roll";
			this->Roll->X1 = 35;
			this->Roll->X2 = 155;
			this->Roll->Y1 = 130;
			this->Roll->Y2 = 130;
			// 
			// ovalShape1
			// 
			this->ovalShape1->BackColor = System::Drawing::Color::DimGray;
			this->ovalShape1->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape1->BorderColor = System::Drawing::Color::White;
			this->ovalShape1->BorderWidth = 5;
			this->ovalShape1->Location = System::Drawing::Point(30, 65);
			this->ovalShape1->Name = L"ovalShape1";
			this->ovalShape1->Size = System::Drawing::Size(130, 130);
			// 
			// ovalShape2
			// 
			this->ovalShape2->BackColor = System::Drawing::Color::DimGray;
			this->ovalShape2->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape2->BorderColor = System::Drawing::Color::White;
			this->ovalShape2->BorderWidth = 5;
			this->ovalShape2->Location = System::Drawing::Point(209, 65);
			this->ovalShape2->Name = L"ovalShape2";
			this->ovalShape2->Size = System::Drawing::Size(130, 130);
			// 
			// Serial
			// 
			this->Serial->Controls->Add(this->groupBox4);
			this->Serial->Controls->Add(this->groupBox3);
			this->Serial->Location = System::Drawing::Point(4, 22);
			this->Serial->Name = L"Serial";
			this->Serial->Padding = System::Windows::Forms::Padding(3);
			this->Serial->Size = System::Drawing::Size(508, 243);
			this->Serial->TabIndex = 3;
			this->Serial->Text = L"Serial and Controller";
			this->Serial->UseVisualStyleBackColor = true;
			// 
			// groupBox4
			// 
			this->groupBox4->Controls->Add(this->ControlCheck);
			this->groupBox4->Controls->Add(this->C2Status);
			this->groupBox4->Controls->Add(this->C1Status);
			this->groupBox4->Controls->Add(this->label14);
			this->groupBox4->Controls->Add(this->label15);
			this->groupBox4->Location = System::Drawing::Point(7, 140);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(495, 94);
			this->groupBox4->TabIndex = 13;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"Controller";
			// 
			// ControlCheck
			// 
			this->ControlCheck->Location = System::Drawing::Point(355, 26);
			this->ControlCheck->Name = L"ControlCheck";
			this->ControlCheck->Size = System::Drawing::Size(134, 45);
			this->ControlCheck->TabIndex = 12;
			this->ControlCheck->Text = L"Check for Controllers";
			this->ControlCheck->UseVisualStyleBackColor = true;
			this->ControlCheck->Click += gcnew System::EventHandler(this, &BaseStation::ControlCheck_Click);
			// 
			// C2Status
			// 
			this->C2Status->AutoSize = true;
			this->C2Status->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->C2Status->Location = System::Drawing::Point(137, 55);
			this->C2Status->Name = L"C2Status";
			this->C2Status->Size = System::Drawing::Size(75, 16);
			this->C2Status->TabIndex = 3;
			this->C2Status->Text = L"Controller 2";
			// 
			// C1Status
			// 
			this->C1Status->AutoSize = true;
			this->C1Status->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->C1Status->Location = System::Drawing::Point(137, 26);
			this->C1Status->Name = L"C1Status";
			this->C1Status->Size = System::Drawing::Size(75, 16);
			this->C1Status->TabIndex = 2;
			this->C1Status->Text = L"Controller 1";
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label14->Location = System::Drawing::Point(10, 55);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(121, 16);
			this->label14->TabIndex = 1;
			this->label14->Text = L"Controller 2 Status :";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label15->Location = System::Drawing::Point(10, 26);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(121, 16);
			this->label15->TabIndex = 0;
			this->label15->Text = L"Controller 1 Status :";
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->label13);
			this->groupBox3->Controls->Add(this->label12);
			this->groupBox3->Controls->Add(this->button2);
			this->groupBox3->Controls->Add(this->serialLabel);
			this->groupBox3->Controls->Add(this->recValues);
			this->groupBox3->Controls->Add(this->CPortBox);
			this->groupBox3->Location = System::Drawing::Point(6, 6);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(496, 128);
			this->groupBox3->TabIndex = 12;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Serial";
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label13->Location = System::Drawing::Point(11, 52);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(69, 16);
			this->label13->TabIndex = 11;
			this->label13->Text = L"Serial Out:";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label12->Location = System::Drawing::Point(19, 24);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(59, 16);
			this->label12->TabIndex = 10;
			this->label12->Text = L"Serial in:";
			// 
			// ThrusterToggle
			// 
			this->ThrusterToggle->AutoScroll = true;
			this->ThrusterToggle->Controls->Add(this->DisableLBV);
			this->ThrusterToggle->Controls->Add(this->DisableRBV);
			this->ThrusterToggle->Controls->Add(this->DisableRFV);
			this->ThrusterToggle->Controls->Add(this->DisableLFV);
			this->ThrusterToggle->Controls->Add(this->DisableLB);
			this->ThrusterToggle->Controls->Add(this->DisableRB);
			this->ThrusterToggle->Controls->Add(this->DisableRF);
			this->ThrusterToggle->Controls->Add(this->DisableLF);
			this->ThrusterToggle->Location = System::Drawing::Point(4, 22);
			this->ThrusterToggle->Name = L"ThrusterToggle";
			this->ThrusterToggle->Size = System::Drawing::Size(508, 243);
			this->ThrusterToggle->TabIndex = 2;
			this->ThrusterToggle->Text = L"Toggle Thrusters";
			this->ThrusterToggle->UseVisualStyleBackColor = true;
			// 
			// DisableLBV
			// 
			this->DisableLBV->AutoSize = true;
			this->DisableLBV->Location = System::Drawing::Point(113, 153);
			this->DisableLBV->Name = L"DisableLBV";
			this->DisableLBV->Size = System::Drawing::Size(132, 17);
			this->DisableLBV->TabIndex = 7;
			this->DisableLBV->Text = L"Disable Left Back Vert";
			this->DisableLBV->UseVisualStyleBackColor = true;
			// 
			// DisableRBV
			// 
			this->DisableRBV->AutoSize = true;
			this->DisableRBV->Location = System::Drawing::Point(254, 153);
			this->DisableRBV->Name = L"DisableRBV";
			this->DisableRBV->Size = System::Drawing::Size(139, 17);
			this->DisableRBV->TabIndex = 6;
			this->DisableRBV->Text = L"Disable Right Back Vert";
			this->DisableRBV->UseVisualStyleBackColor = true;
			// 
			// DisableRFV
			// 
			this->DisableRFV->AutoSize = true;
			this->DisableRFV->Location = System::Drawing::Point(254, 83);
			this->DisableRFV->Name = L"DisableRFV";
			this->DisableRFV->Size = System::Drawing::Size(138, 17);
			this->DisableRFV->TabIndex = 5;
			this->DisableRFV->Text = L"Disable Right Front Vert";
			this->DisableRFV->UseVisualStyleBackColor = true;
			// 
			// DisableLFV
			// 
			this->DisableLFV->AutoSize = true;
			this->DisableLFV->Location = System::Drawing::Point(113, 83);
			this->DisableLFV->Name = L"DisableLFV";
			this->DisableLFV->Size = System::Drawing::Size(131, 17);
			this->DisableLFV->TabIndex = 4;
			this->DisableLFV->Text = L"Disable Left Front Vert";
			this->DisableLFV->UseVisualStyleBackColor = true;
			// 
			// DisableLB
			// 
			this->DisableLB->AutoSize = true;
			this->DisableLB->Location = System::Drawing::Point(33, 192);
			this->DisableLB->Name = L"DisableLB";
			this->DisableLB->Size = System::Drawing::Size(110, 17);
			this->DisableLB->TabIndex = 3;
			this->DisableLB->Text = L"Disable Left Back";
			this->DisableLB->UseVisualStyleBackColor = true;
			// 
			// DisableRB
			// 
			this->DisableRB->AutoSize = true;
			this->DisableRB->Location = System::Drawing::Point(353, 192);
			this->DisableRB->Name = L"DisableRB";
			this->DisableRB->Size = System::Drawing::Size(117, 17);
			this->DisableRB->TabIndex = 2;
			this->DisableRB->Text = L"Disable Right Back";
			this->DisableRB->UseVisualStyleBackColor = true;
			// 
			// DisableRF
			// 
			this->DisableRF->AutoSize = true;
			this->DisableRF->Location = System::Drawing::Point(353, 45);
			this->DisableRF->Name = L"DisableRF";
			this->DisableRF->Size = System::Drawing::Size(116, 17);
			this->DisableRF->TabIndex = 1;
			this->DisableRF->Text = L"Disable Right Front";
			this->DisableRF->UseVisualStyleBackColor = true;
			// 
			// DisableLF
			// 
			this->DisableLF->AutoSize = true;
			this->DisableLF->Location = System::Drawing::Point(39, 45);
			this->DisableLF->Name = L"DisableLF";
			this->DisableLF->Size = System::Drawing::Size(109, 17);
			this->DisableLF->TabIndex = 0;
			this->DisableLF->Text = L"Disable Left Front";
			this->DisableLF->UseVisualStyleBackColor = true;
			// 
			// Main
			// 
			this->Main->Controls->Add(this->label25);
			this->Main->Controls->Add(this->TurnerBar);
			this->Main->Controls->Add(this->label8);
			this->Main->Controls->Add(this->label7);
			this->Main->Controls->Add(this->label6);
			this->Main->Controls->Add(this->label5);
			this->Main->Controls->Add(this->PitchRollBar);
			this->Main->Controls->Add(this->StrafeSpeedCTL);
			this->Main->Controls->Add(this->VertLimiter);
			this->Main->Controls->Add(this->HorizLimiter);
			this->Main->Location = System::Drawing::Point(4, 22);
			this->Main->Name = L"Main";
			this->Main->Padding = System::Windows::Forms::Padding(3);
			this->Main->Size = System::Drawing::Size(508, 243);
			this->Main->TabIndex = 0;
			this->Main->Text = L"Main";
			this->Main->UseVisualStyleBackColor = true;
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label25->Location = System::Drawing::Point(9, 201);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(96, 17);
			this->label25->TabIndex = 9;
			this->label25->Text = L"Turner Speed";
			// 
			// TurnerBar
			// 
			this->TurnerBar->BackColor = System::Drawing::SystemColors::Window;
			this->TurnerBar->LargeChange = 1;
			this->TurnerBar->Location = System::Drawing::Point(164, 189);
			this->TurnerBar->Minimum = 1;
			this->TurnerBar->Name = L"TurnerBar";
			this->TurnerBar->Size = System::Drawing::Size(338, 45);
			this->TurnerBar->TabIndex = 8;
			this->TurnerBar->TickStyle = System::Windows::Forms::TickStyle::Both;
			this->TurnerBar->Value = 8;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label8->Location = System::Drawing::Point(9, 155);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(107, 17);
			this->label8->TabIndex = 7;
			this->label8->Text = L"Pitch/Roll Effect";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label7->Location = System::Drawing::Point(9, 111);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(140, 17);
			this->label7->TabIndex = 6;
			this->label7->Text = L"Strafe Speed Control";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label6->Location = System::Drawing::Point(9, 66);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(149, 17);
			this->label6->TabIndex = 5;
			this->label6->Text = L"Vertical Speed Control";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label5->Location = System::Drawing::Point(9, 20);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(166, 17);
			this->label5->TabIndex = 4;
			this->label5->Text = L"Horizontal Speed Control";
			// 
			// PitchRollBar
			// 
			this->PitchRollBar->BackColor = System::Drawing::SystemColors::Window;
			this->PitchRollBar->LargeChange = 1;
			this->PitchRollBar->Location = System::Drawing::Point(164, 144);
			this->PitchRollBar->Minimum = 1;
			this->PitchRollBar->Name = L"PitchRollBar";
			this->PitchRollBar->Size = System::Drawing::Size(338, 45);
			this->PitchRollBar->TabIndex = 3;
			this->PitchRollBar->TickStyle = System::Windows::Forms::TickStyle::Both;
			this->PitchRollBar->Value = 8;
			this->PitchRollBar->Scroll += gcnew System::EventHandler(this, &BaseStation::PitchRollBar_Scroll);
			// 
			// StrafeSpeedCTL
			// 
			this->StrafeSpeedCTL->BackColor = System::Drawing::SystemColors::Window;
			this->StrafeSpeedCTL->LargeChange = 1;
			this->StrafeSpeedCTL->Location = System::Drawing::Point(164, 99);
			this->StrafeSpeedCTL->Minimum = 1;
			this->StrafeSpeedCTL->Name = L"StrafeSpeedCTL";
			this->StrafeSpeedCTL->Size = System::Drawing::Size(338, 45);
			this->StrafeSpeedCTL->TabIndex = 2;
			this->StrafeSpeedCTL->TickStyle = System::Windows::Forms::TickStyle::Both;
			this->StrafeSpeedCTL->Value = 8;
			this->StrafeSpeedCTL->Scroll += gcnew System::EventHandler(this, &BaseStation::StrafeSpeedCTL_Scroll);
			// 
			// VertLimiter
			// 
			this->VertLimiter->BackColor = System::Drawing::SystemColors::Window;
			this->VertLimiter->LargeChange = 1;
			this->VertLimiter->Location = System::Drawing::Point(164, 54);
			this->VertLimiter->Minimum = 1;
			this->VertLimiter->Name = L"VertLimiter";
			this->VertLimiter->Size = System::Drawing::Size(338, 45);
			this->VertLimiter->TabIndex = 1;
			this->VertLimiter->TickStyle = System::Windows::Forms::TickStyle::Both;
			this->VertLimiter->Value = 10;
			this->VertLimiter->Scroll += gcnew System::EventHandler(this, &BaseStation::VertLimiter_Scroll);
			// 
			// HorizLimiter
			// 
			this->HorizLimiter->BackColor = System::Drawing::SystemColors::Window;
			this->HorizLimiter->LargeChange = 1;
			this->HorizLimiter->Location = System::Drawing::Point(164, 9);
			this->HorizLimiter->Minimum = 1;
			this->HorizLimiter->Name = L"HorizLimiter";
			this->HorizLimiter->Size = System::Drawing::Size(338, 45);
			this->HorizLimiter->TabIndex = 0;
			this->HorizLimiter->TickStyle = System::Windows::Forms::TickStyle::Both;
			this->HorizLimiter->Value = 10;
			this->HorizLimiter->Scroll += gcnew System::EventHandler(this, &BaseStation::HorizLimiter_Scroll);
			// 
			// tabControl1
			// 
			this->tabControl1->Controls->Add(this->Main);
			this->tabControl1->Controls->Add(this->ThrusterToggle);
			this->tabControl1->Controls->Add(this->Serial);
			this->tabControl1->Controls->Add(this->Status);
			this->tabControl1->Controls->Add(this->Debug);
			this->tabControl1->Location = System::Drawing::Point(12, 334);
			this->tabControl1->Name = L"tabControl1";
			this->tabControl1->SelectedIndex = 0;
			this->tabControl1->Size = System::Drawing::Size(516, 269);
			this->tabControl1->TabIndex = 0;
			// 
			// Status
			// 
			this->Status->Controls->Add(this->dataGridView1);
			this->Status->Location = System::Drawing::Point(4, 22);
			this->Status->Name = L"Status";
			this->Status->Padding = System::Windows::Forms::Padding(3);
			this->Status->Size = System::Drawing::Size(508, 243);
			this->Status->TabIndex = 5;
			this->Status->Text = L"Status";
			this->Status->UseVisualStyleBackColor = true;
			// 
			// dataGridView1
			// 
			this->dataGridView1->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
			this->dataGridView1->Columns->AddRange(gcnew cli::array< System::Windows::Forms::DataGridViewColumn^  >(5) {this->Motor, 
				this->Temperature, this->Voltage, this->Current, this->FaultStatus});
			this->dataGridView1->Location = System::Drawing::Point(3, 3);
			this->dataGridView1->Name = L"dataGridView1";
			this->dataGridView1->Size = System::Drawing::Size(499, 234);
			this->dataGridView1->TabIndex = 0;
			// 
			// Motor
			// 
			this->Motor->HeaderText = L"Motor";
			this->Motor->Name = L"Motor";
			this->Motor->ReadOnly = true;
			this->Motor->Width = 50;
			// 
			// Temperature
			// 
			this->Temperature->HeaderText = L"Temperature";
			this->Temperature->Name = L"Temperature";
			// 
			// Voltage
			// 
			this->Voltage->HeaderText = L"Voltage";
			this->Voltage->Name = L"Voltage";
			// 
			// Current
			// 
			this->Current->HeaderText = L"Current";
			this->Current->Name = L"Current";
			// 
			// FaultStatus
			// 
			this->FaultStatus->HeaderText = L"Fault Status";
			this->FaultStatus->Name = L"FaultStatus";
			// 
			// Debug
			// 
			this->Debug->Controls->Add(this->button5);
			this->Debug->Controls->Add(this->button4);
			this->Debug->Controls->Add(this->button3);
			this->Debug->Controls->Add(this->ManipToggle);
			this->Debug->Controls->Add(this->TimerReset);
			this->Debug->Controls->Add(this->SerialForce);
			this->Debug->Controls->Add(this->SerialDebug);
			this->Debug->Location = System::Drawing::Point(4, 22);
			this->Debug->Name = L"Debug";
			this->Debug->Padding = System::Windows::Forms::Padding(3);
			this->Debug->Size = System::Drawing::Size(508, 243);
			this->Debug->TabIndex = 4;
			this->Debug->Text = L"Debug";
			this->Debug->UseVisualStyleBackColor = true;
			// 
			// TimerReset
			// 
			this->TimerReset->Location = System::Drawing::Point(396, 71);
			this->TimerReset->Name = L"TimerReset";
			this->TimerReset->Size = System::Drawing::Size(106, 66);
			this->TimerReset->TabIndex = 2;
			this->TimerReset->Text = L"Reset Timer";
			this->TimerReset->UseVisualStyleBackColor = true;
			this->TimerReset->Click += gcnew System::EventHandler(this, &BaseStation::TimerReset_Click);
			// 
			// SerialForce
			// 
			this->SerialForce->Location = System::Drawing::Point(396, 6);
			this->SerialForce->Name = L"SerialForce";
			this->SerialForce->Size = System::Drawing::Size(106, 37);
			this->SerialForce->TabIndex = 1;
			this->SerialForce->Text = L"I Dont need to stinkin serial!";
			this->SerialForce->UseVisualStyleBackColor = true;
			this->SerialForce->Click += gcnew System::EventHandler(this, &BaseStation::SerialForce_Click);
			// 
			// SerialDebug
			// 
			this->SerialDebug->AutoSize = true;
			this->SerialDebug->Location = System::Drawing::Point(6, 18);
			this->SerialDebug->Name = L"SerialDebug";
			this->SerialDebug->Size = System::Drawing::Size(41, 13);
			this->SerialDebug->TabIndex = 0;
			this->SerialDebug->Text = L"label26";
			// 
			// printForm1
			// 
			this->printForm1->DocumentName = L"document";
			this->printForm1->Form = this;
			this->printForm1->PrintAction = System::Drawing::Printing::PrintAction::PrintToPrinter;
			this->printForm1->PrinterSettings = (cli::safe_cast<System::Drawing::Printing::PrinterSettings^  >(resources->GetObject(L"printForm1.PrinterSettings")));
			this->printForm1->PrintFileName = nullptr;
			// 
			// groupBox5
			// 
			this->groupBox5->Controls->Add(this->label34);
			this->groupBox5->Controls->Add(this->label33);
			this->groupBox5->Controls->Add(this->label32);
			this->groupBox5->Controls->Add(this->label31);
			this->groupBox5->Controls->Add(this->label30);
			this->groupBox5->Controls->Add(this->label29);
			this->groupBox5->Controls->Add(this->label28);
			this->groupBox5->Controls->Add(this->label27);
			this->groupBox5->Controls->Add(this->label26);
			this->groupBox5->Controls->Add(this->label18);
			this->groupBox5->Controls->Add(this->label17);
			this->groupBox5->Controls->Add(this->label16);
			this->groupBox5->Controls->Add(this->label11);
			this->groupBox5->Controls->Add(this->label4);
			this->groupBox5->Controls->Add(this->label3);
			this->groupBox5->Controls->Add(this->label2);
			this->groupBox5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->groupBox5->Location = System::Drawing::Point(534, 381);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(143, 222);
			this->groupBox5->TabIndex = 11;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"Motor Status";
			// 
			// label34
			// 
			this->label34->AutoSize = true;
			this->label34->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label34->Location = System::Drawing::Point(101, 200);
			this->label34->Name = L"label34";
			this->label34->Size = System::Drawing::Size(26, 16);
			this->label34->TabIndex = 9;
			this->label34->Text = L"OK";
			// 
			// label33
			// 
			this->label33->AutoSize = true;
			this->label33->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label33->Location = System::Drawing::Point(101, 174);
			this->label33->Name = L"label33";
			this->label33->Size = System::Drawing::Size(26, 16);
			this->label33->TabIndex = 8;
			this->label33->Text = L"OK";
			// 
			// label32
			// 
			this->label32->AutoSize = true;
			this->label32->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label32->Location = System::Drawing::Point(101, 148);
			this->label32->Name = L"label32";
			this->label32->Size = System::Drawing::Size(26, 16);
			this->label32->TabIndex = 7;
			this->label32->Text = L"OK";
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label31->Location = System::Drawing::Point(101, 122);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(26, 16);
			this->label31->TabIndex = 6;
			this->label31->Text = L"OK";
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label30->Location = System::Drawing::Point(101, 96);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(26, 16);
			this->label30->TabIndex = 5;
			this->label30->Text = L"OK";
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label29->Location = System::Drawing::Point(101, 70);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(26, 16);
			this->label29->TabIndex = 4;
			this->label29->Text = L"OK";
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label28->Location = System::Drawing::Point(101, 44);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(26, 16);
			this->label28->TabIndex = 3;
			this->label28->Text = L"OK";
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label27->Location = System::Drawing::Point(101, 18);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(26, 16);
			this->label27->TabIndex = 2;
			this->label27->Text = L"OK";
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->Location = System::Drawing::Point(0, 200);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(96, 16);
			this->label26->TabIndex = 1;
			this->label26->Text = L"Left Back Vert :";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(0, 174);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(106, 16);
			this->label18->TabIndex = 0;
			this->label18->Text = L"Right Back Vert :";
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(0, 148);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(105, 16);
			this->label17->TabIndex = 0;
			this->label17->Text = L"Right Front Vert :";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(0, 122);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(95, 16);
			this->label16->TabIndex = 0;
			this->label16->Text = L"Left Front Vert :";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(0, 96);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(69, 16);
			this->label11->TabIndex = 0;
			this->label11->Text = L"Left Back :";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(0, 70);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(79, 16);
			this->label4->TabIndex = 0;
			this->label4->Text = L"Right Back :";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(0, 44);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(78, 16);
			this->label3->TabIndex = 0;
			this->label3->Text = L"Right Front :";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(0, 18);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(68, 16);
			this->label2->TabIndex = 0;
			this->label2->Text = L"Left Front :";
			// 
			// checkedListBox1
			// 
			this->checkedListBox1->FormattingEnabled = true;
			this->checkedListBox1->Items->AddRange(gcnew cli::array< System::Object^  >(29) {L"1. Transferring the SIA to the seafloor – 5 points", 
				L"2. Installing the SIA so that it rests completely within the BIA – 15 points", L"3. Removing the CTA from the seafloor – 5 points", 
				L"4. Inserting the CTA into the bulkhead connector on the BIA – 10 points", L"5. Pulling the pin to release the secondary node from the elevator – 10 points", 
				L"6. Removing the secondary node from the elevator – 5 points", L"7. Measuring distance to find the designated location – 15 points", 
				L"8. Installing the secondary node in the designated location on the seafloor – 10 " 
				L"points", L"9. Adjusting the legs to level the secondary node – 25 points", 
				L"10. Opening the door of the BIA – 5 points", L"11. Removing the secondary node cable connector from the elevator – 5 points", 
				L"12. Inserting the secondary node cable connector into the bulkhead connector on t" 
				L"he SIA – 10 points", L"", L"1. Designing and constructing an optical beam transmissometer prior to the compet" 
				L"ition – 15 points", 
				L"2. Installing the transmissometer in the vent field to monitor the opacity throug" 
				L"h the medium – 10 points", L"3. Detecting the relative changes in opacity – 10 points", 
				L"4. Detecting the relative changes in opacity over five minutes – 20 points", L"5. Graphing the relative optical transmission (aka opacity) versus time on a vide" 
				L"o display – 20 points", 
				L"", L"1. Disconnecting power to the platform – 10 points", L"2. Turning the handle to unlock the hatch – 10 points", L"3. Opening the hatch – 10 points", 
				L"4. Removing the ADCP from the mooring platform – 10 point", L"5. Installing the new ADCP into the mooring platform – 10 points", 
				L"6. Closing the hatch – 10 points", L"7. Turning the handle to lock the hatch – 10 points", L"8. Reconnecting power to the platform – 10 points", 
				L"", L"1. Locate five areas of biofouling and removing all biofouling organisms – 5 poin" 
				L"ts each"});
			this->checkedListBox1->Location = System::Drawing::Point(12, 12);
			this->checkedListBox1->Name = L"checkedListBox1";
			this->checkedListBox1->Size = System::Drawing::Size(516, 274);
			this->checkedListBox1->TabIndex = 12;
			// 
			// TimerStart
			// 
			this->TimerStart->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->TimerStart->Location = System::Drawing::Point(534, 11);
			this->TimerStart->Name = L"TimerStart";
			this->TimerStart->Size = System::Drawing::Size(143, 39);
			this->TimerStart->TabIndex = 13;
			this->TimerStart->Text = L"Start Timer";
			this->TimerStart->UseVisualStyleBackColor = true;
			this->TimerStart->Click += gcnew System::EventHandler(this, &BaseStation::TimerStart_Click);
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->label36);
			this->groupBox2->Controls->Add(this->TimeMin);
			this->groupBox2->Controls->Add(this->TimeSec);
			this->groupBox2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->groupBox2->Location = System::Drawing::Point(534, 57);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(143, 50);
			this->groupBox2->TabIndex = 14;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Current Run Time";
			// 
			// label36
			// 
			this->label36->AutoSize = true;
			this->label36->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 16, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label36->Location = System::Drawing::Point(65, 19);
			this->label36->Name = L"label36";
			this->label36->Size = System::Drawing::Size(18, 26);
			this->label36->TabIndex = 2;
			this->label36->Text = L":";
			// 
			// TimeMin
			// 
			this->TimeMin->AutoSize = true;
			this->TimeMin->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 16, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->TimeMin->Location = System::Drawing::Point(35, 21);
			this->TimeMin->Name = L"TimeMin";
			this->TimeMin->Size = System::Drawing::Size(36, 26);
			this->TimeMin->TabIndex = 1;
			this->TimeMin->Text = L"00";
			// 
			// TimeSec
			// 
			this->TimeSec->AutoSize = true;
			this->TimeSec->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 16, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->TimeSec->Location = System::Drawing::Point(77, 21);
			this->TimeSec->Name = L"TimeSec";
			this->TimeSec->Size = System::Drawing::Size(36, 26);
			this->TimeSec->TabIndex = 0;
			this->TimeSec->Text = L"00";
			// 
			// timer2
			// 
			this->timer2->Interval = 1000;
			this->timer2->Tick += gcnew System::EventHandler(this, &BaseStation::timer2_Tick);
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->label24);
			this->groupBox6->Controls->Add(this->label23);
			this->groupBox6->Controls->Add(this->label22);
			this->groupBox6->Controls->Add(this->turner1);
			this->groupBox6->Controls->Add(this->shapeContainer1);
			this->groupBox6->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->groupBox6->Location = System::Drawing::Point(534, 113);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(143, 163);
			this->groupBox6->TabIndex = 15;
			this->groupBox6->TabStop = false;
			this->groupBox6->Text = L"Turner Motors";
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->BackColor = System::Drawing::Color::Silver;
			this->label24->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label24->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->label24->Location = System::Drawing::Point(102, 115);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(18, 20);
			this->label24->TabIndex = 19;
			this->label24->Text = L"0";
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->BackColor = System::Drawing::Color::Silver;
			this->label23->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label23->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->label23->Location = System::Drawing::Point(22, 115);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(18, 20);
			this->label23->TabIndex = 18;
			this->label23->Text = L"0";
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->BackColor = System::Drawing::Color::Silver;
			this->label22->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label22->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->label22->Location = System::Drawing::Point(102, 40);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(18, 20);
			this->label22->TabIndex = 19;
			this->label22->Text = L"0";
			// 
			// turner1
			// 
			this->turner1->AutoSize = true;
			this->turner1->BackColor = System::Drawing::Color::Silver;
			this->turner1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->turner1->ForeColor = System::Drawing::SystemColors::ActiveCaptionText;
			this->turner1->Location = System::Drawing::Point(22, 40);
			this->turner1->Name = L"turner1";
			this->turner1->Size = System::Drawing::Size(18, 20);
			this->turner1->TabIndex = 18;
			this->turner1->Text = L"0";
			// 
			// shapeContainer1
			// 
			this->shapeContainer1->Location = System::Drawing::Point(3, 19);
			this->shapeContainer1->Margin = System::Windows::Forms::Padding(0);
			this->shapeContainer1->Name = L"shapeContainer1";
			this->shapeContainer1->Shapes->AddRange(gcnew cli::array< Microsoft::VisualBasic::PowerPacks::Shape^  >(4) {this->ovalShape6, 
				this->ovalShape5, this->ovalShape4, this->ovalShape3});
			this->shapeContainer1->Size = System::Drawing::Size(137, 141);
			this->shapeContainer1->TabIndex = 0;
			this->shapeContainer1->TabStop = false;
			// 
			// ovalShape6
			// 
			this->ovalShape6->BackColor = System::Drawing::Color::Silver;
			this->ovalShape6->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape6->BorderColor = System::Drawing::Color::White;
			this->ovalShape6->BorderWidth = 4;
			this->ovalShape6->Location = System::Drawing::Point(83, 80);
			this->ovalShape6->Name = L"ovalShape6";
			this->ovalShape6->Size = System::Drawing::Size(50, 50);
			// 
			// ovalShape5
			// 
			this->ovalShape5->BackColor = System::Drawing::Color::Silver;
			this->ovalShape5->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape5->BorderColor = System::Drawing::Color::White;
			this->ovalShape5->BorderWidth = 4;
			this->ovalShape5->Location = System::Drawing::Point(2, 80);
			this->ovalShape5->Name = L"ovalShape5";
			this->ovalShape5->Size = System::Drawing::Size(50, 50);
			// 
			// ovalShape4
			// 
			this->ovalShape4->BackColor = System::Drawing::Color::Silver;
			this->ovalShape4->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape4->BorderColor = System::Drawing::Color::White;
			this->ovalShape4->BorderWidth = 4;
			this->ovalShape4->Location = System::Drawing::Point(83, 6);
			this->ovalShape4->Name = L"ovalShape4";
			this->ovalShape4->Size = System::Drawing::Size(50, 50);
			// 
			// ovalShape3
			// 
			this->ovalShape3->BackColor = System::Drawing::Color::Silver;
			this->ovalShape3->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->ovalShape3->BorderColor = System::Drawing::Color::White;
			this->ovalShape3->BorderWidth = 4;
			this->ovalShape3->Location = System::Drawing::Point(2, 6);
			this->ovalShape3->Name = L"ovalShape3";
			this->ovalShape3->Size = System::Drawing::Size(50, 50);
			// 
			// statusStrip1
			// 
			this->statusStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {this->PolarTest, this->SerialLabel2});
			this->statusStrip1->Location = System::Drawing::Point(0, 609);
			this->statusStrip1->Name = L"statusStrip1";
			this->statusStrip1->Size = System::Drawing::Size(1067, 22);
			this->statusStrip1->TabIndex = 16;
			this->statusStrip1->Text = L"statusStrip1";
			// 
			// PolarTest
			// 
			this->PolarTest->Name = L"PolarTest";
			this->PolarTest->Size = System::Drawing::Size(10, 17);
			this->PolarTest->Text = L".";
			// 
			// SerialLabel2
			// 
			this->SerialLabel2->Name = L"SerialLabel2";
			this->SerialLabel2->Size = System::Drawing::Size(10, 17);
			this->SerialLabel2->Text = L".";
			// 
			// ManipToggle
			// 
			this->ManipToggle->Location = System::Drawing::Point(9, 71);
			this->ManipToggle->Name = L"ManipToggle";
			this->ManipToggle->Size = System::Drawing::Size(123, 23);
			this->ManipToggle->TabIndex = 3;
			this->ManipToggle->Text = L"Toggle Manipulator";
			this->ManipToggle->UseVisualStyleBackColor = true;
			this->ManipToggle->Click += gcnew System::EventHandler(this, &BaseStation::ManipToggle_Click);
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(9, 101);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(123, 23);
			this->button3->TabIndex = 4;
			this->button3->Text = L"button3";
			this->button3->UseVisualStyleBackColor = true;
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(9, 131);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(123, 23);
			this->button4->TabIndex = 5;
			this->button4->Text = L"button4";
			this->button4->UseVisualStyleBackColor = true;
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(9, 161);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(123, 23);
			this->button5->TabIndex = 6;
			this->button5->Text = L"button5";
			this->button5->UseVisualStyleBackColor = true;
			// 
			// BaseStation
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1067, 631);
			this->Controls->Add(this->statusStrip1);
			this->Controls->Add(this->groupBox6);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->TimerStart);
			this->Controls->Add(this->checkedListBox1);
			this->Controls->Add(this->groupBox5);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->estopDisp);
			this->Controls->Add(this->ToolPanel);
			this->Controls->Add(this->panel1);
			this->Controls->Add(this->tabControl1);
			this->Name = L"BaseStation";
			this->RightToLeftLayout = true;
			this->Text = L"BaseStation";
			this->Load += gcnew System::EventHandler(this, &BaseStation::BaseStation_Load);
			this->ToolPanel->ResumeLayout(false);
			this->ToolPanel->PerformLayout();
			this->panel1->ResumeLayout(false);
			this->panel1->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->Serial->ResumeLayout(false);
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			this->ThrusterToggle->ResumeLayout(false);
			this->ThrusterToggle->PerformLayout();
			this->Main->ResumeLayout(false);
			this->Main->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->TurnerBar))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->PitchRollBar))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->StrafeSpeedCTL))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->VertLimiter))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->HorizLimiter))->EndInit();
			this->tabControl1->ResumeLayout(false);
			this->Status->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->dataGridView1))->EndInit();
			this->Debug->ResumeLayout(false);
			this->Debug->PerformLayout();
			this->groupBox5->ResumeLayout(false);
			this->groupBox5->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->groupBox6->ResumeLayout(false);
			this->groupBox6->PerformLayout();
			this->statusStrip1->ResumeLayout(false);
			this->statusStrip1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void BaseStation_Load(System::Object^  sender, System::EventArgs^  e) {
			}
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
				 UpdateConnection();
			 }
	private: System::Void HorizLimiter_Scroll(System::Object^  sender, System::EventArgs^  e) {
				 SpeedDivider_H = (float)(HorizLimiter->Value)/10;
			 }
	private: System::Void StrafeSpeedCTL_Scroll(System::Object^  sender, System::EventArgs^  e) {
				 StrafeSpeed = (float)(StrafeSpeedCTL->Value)/10;
			 }
	private: System::Void VertLimiter_Scroll(System::Object^  sender, System::EventArgs^  e) {
				 SpeedDivider_V = (float)(VertLimiter->Value)/10;
			 }
	private: System::Void PitchRollBar_Scroll(System::Object^  sender, System::EventArgs^  e) {
				 PitchRollFactor = (float)(PitchRollBar->Value)/10;
			 }
	private: System::Void ControlCheck_Click(System::Object^  sender, System::EventArgs^  e) {
				 CheckControllerStatus();
			 }
private: System::Void SerialForce_Click(System::Object^  sender, System::EventArgs^  e) {
			if(!serialstart){
				 serialstart = true;
			}else{
				serialstart = false;
			}
		 }

private: System::Void TimerStart_Click(System::Object^  sender, System::EventArgs^  e) {
			if(!missiontime){
				missiontime = true;
				timer2->Enabled = true;
				TimerStart -> Text = "Pause Timer";
			}else{
				missiontime = false;
				timer2->Enabled = false;
				TimerStart -> Text = "Start Timer";
			}
		 }
private: System::Void TimerReset_Click(System::Object^  sender, System::EventArgs^  e) {
			 missiontime = false;
			 timer2->Enabled = false;
			 TimerStart -> Text = "Start Timer";
			 timercount = 0;
			 TimeMin -> Text = "0";
			 TimeSec -> Text = "0";
		 }
private: System::Void ManipToggle_Click(System::Object^  sender, System::EventArgs^  e) {
			if (manip_open == true) {
				manip_open = false;
				ManipStatus->Text = gcnew String("Closed");
			}else{
				manip_open = true;
				ManipStatus->Text = gcnew String("Open");
			}
		}
};
}

