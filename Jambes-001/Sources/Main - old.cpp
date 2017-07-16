//////////////////////////////////////////////////////////////////////////////////
//Using a servo motor samples.
//////////////////////////////////////////////////////////////////////////////////
#include <iostream>
using namespace std;


#include "ModaCPP.h"
using namespace ModaCPP;

//#include <Eigen/Core>
#include <Eigen>
using namespace Eigen;



///////////////////////////////////////////////////////////////////////////////////
void DisplayServo(ModaCPP::DeviceServoMotor *pServo)
///////////////////////////////////////////////////////////////////////////////////
	{
	_cprintf("\tmax torque: %f / %f\r\n"		, pServo->GetEffectiveMaxTorque()			,pServo->GetMaxTorque());
	_cprintf("\tmax acceleration: %f / %f\r\n"	, pServo->GetEffectiveMaxAccelerationDEG()	, pServo->GetMaxAccelerationDEG());
	_cprintf("\tmax velocity: %f / %f\r\n"		, pServo->GetEffectiveMaxVelocityDPS()		, pServo->GetMaxVelocityDPS());
	}

///////////////////////////////////////////////////////////////////////////////////
int main(int argc, CHAR* argv[])
///////////////////////////////////////////////////////////////////////////////////
{

//Test matrice
  Matrix2d a;
  a << 10, 12,
       3, 4;
  MatrixXd b(2,2);
  b << 2, 3,
       1, 4;
  std::cout << "a + b =\n" << a + b << std::endl;
  std::cout << "a - b =\n" << a - b << std::endl;
  std::cout << "Doing a += b;" << std::endl;
  a += b;
  std::cout << "Now a =\n" << a << std::endl;

  std::cout << "XXXXXXXXXXXXXXXXXX" << a << std::endl;



//Fin test matrice

//process the command line
CommandLine::ProcessCommandLine(argc,argv);
//Connect to MODA server
Connection *pConnection=new ModaCPP::Connection(true);
if(pConnection->Connect( CommandLine::GetArgumentValue("/modaserver","127.0.0.1"),CommandLine::GetArgumentValueINT("/modaport",0),false))
	{
	_cprintf("Connection ok\r\n");
	
	//Find the robot
	ModaCPP::RobotPHX *robot=pConnection->QueryRobotPHX("/");
	if(robot)
		{

		_cprintf("robot found in this world\r\n");
		DeviceServoMotor *pServoCheville_G=robot->QueryDeviceServoMotor("phx0/HingeCheville_G/a1/ServoCheville_G");
		DeviceServoMotor *pServoTibia_G=robot->QueryDeviceServoMotor("phx0/HingeTibia_G/a1/ServoTibia_G");
		DeviceServoMotor *pServoGenou_G=robot->QueryDeviceServoMotor("phx0/HingeGenou_G/a1/ServoGenou_G");
		DeviceServoMotor *pServoJambe_G=robot->QueryDeviceServoMotor("phx0/HingeJambe_G/a1/ServoJambe_G");
		DeviceServoMotor *pServoPivot_G=robot->QueryDeviceServoMotor("phx0/HingeJambe_G/a2/ServoPivot_G");

		if(pServoJambe_G)
		{
			_cprintf("ServoJambe found ...\r\n");
			pConnection->Sleep(2000);
		}
				else
			{
			_cprintf("ServoCheville_G not found ...\r\n");
				pConnection->Sleep(2000);
			}

		if(pServoPivot_G)
		{
			_cprintf("ServoPivot_G found ...\r\n");
			pConnection->Sleep(2000);
		}
				else
			{
			_cprintf("ServoTibia_G not found ...\r\n");
				pConnection->Sleep(2000);
			}

		if(pServoCheville_G && pServoTibia_G && pServoGenou_G && pServoJambe_G && pServoPivot_G)
			{
			//Save configuration values
			float maxvel=pServoCheville_G->GetMaxVelocityDPS();
			float maxtorque=pServoTibia_G->GetMaxTorque();
					

			//Basic servo checking
			_cprintf("Servo1 configuration:\r\n");
			DisplayServo(pServoCheville_G);
			_cprintf("Servo2 configuration:\r\n");
			DisplayServo(pServoTibia_G);
			
			///////////////////////////////////////////////////////////////////////
			//Position regulation, up down at max speed
			/*
			_cprintf("Up/Down servo1 at max speed (%f °/s)\r\n",maxvel);
			pServoCheville_G->GoPositionDeg(-89);
			pServoCheville_G->WaitPositionDegComplete(-89,0.2f,5000);
			pConnection->Sleep(500);
			pServoCheville_G->GoPositionDeg(89);
			pServoCheville_G->WaitPositionDegComplete(89,0.2f,5000);
			pConnection->Sleep(500);
			
			_cprintf("Up/Down servo2 at max speed (%f °/s)\r\n",maxvel);
			pServoTibia_G->GoPositionDeg(-89);
			pServoTibia_G->WaitPositionDegComplete(-89,0.2f,5000);
			pConnection->Sleep(500);
			pServoTibia_G->GoPositionDeg(89);
			pServoTibia_G->WaitPositionDegComplete(89,0.2f,5000);
			pConnection->Sleep(500);
			*/
			
			
			_cprintf("Testing servo1 + servo2\r\n");
			float deg=0;
			while(!_kbhit())
				{
				_cprintf("NO MOVE ...\r\n");
				pConnection->Sleep(2000);

				pServoCheville_G->GoPositionDeg(-20);
				_cprintf("ServoCheville_G = 90° ...\r\n");
				pConnection->Sleep(2000);
				
				pServoTibia_G->GoPositionDeg(30);
				_cprintf("ServoTibia_G = 30° ...\r\n");
				pConnection->Sleep(2000);

				pServoGenou_G->GoPositionDeg(-30);
				_cprintf("ServoCheville_G = 90° ...\r\n");
				pConnection->Sleep(2000);

				pServoPivot_G->GoPositionDeg(90);
				_cprintf("ServoCheville_G = 90° ...\r\n");
				pConnection->Sleep(2000);



				pServoCheville_G->GoPositionDeg(0);
				pServoTibia_G->GoPositionDeg(0);
				pServoGenou_G->GoPositionDeg(0);
				pServoPivot_G->GoPositionDeg(0);
				_cprintf("ALL Servo = 0° ...\r\n");
				pConnection->Sleep(2000);


				/*
				pServoTibia_G->GoPositionDeg(90);
				pConnection->Sleep(2000);

				pServoCheville_G->GoPositionDeg(90);
				pServoTibia_G->GoPositionDeg(-90);
				pConnection->Sleep(2000);

				pServoCheville_G->GoPositionDeg(0);
				pServoTibia_G->GoPositionDeg(0);
				pConnection->Sleep(2000);
				*/
				
				//deg+=90;
				//pServo2->GoPositionDeg(deg);
				//pConnection->Sleep(2000);
				}
			
			//delete devices
			delete pServoCheville_G;
			delete pServoTibia_G;
			}
		else
			{
			_cprintf("servo(s) not found ...\r\n");
				pConnection->Sleep(2000);
			}
		delete robot;
		}
	else
		{
		_cprintf("robot not found in this world\r\n");
		}
	}
else
	{
	_cprintf("Unable to connect to moda server : be sure Exec is running and MODA TCP/UDP ports are open\r\n");
	}

	pConnection->Disconnect();
	delete pConnection;

return 0;
}


