//////////////////////////////////////////////////////////////////////////////////
//Using a servo motor samples.
//////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//LES LIBRAIRIES
#include <iostream>
using namespace std;
#include "ModaCPP.h"
using namespace ModaCPP;
//#include <Eigen/Core>

#include <Eigen>
using namespace Eigen;


#include <math.h>

#include "matrices.h"



///////////////////////////////////////////////////////////////////////////////////
//Déclaration des valeurs PI et PI/2
# define PI (3.14159265358979323846)
# define PI2 (1.57079632679489661923)

///////////////////////////////////////////////////////////////////////////////////
//Déclaration des Variables
bool JambeGauche, JambeDroite;//Permet de tester les servomoteurs de la jambe gauche et de la jambe droite
int iteration =0;//Permet de déterminer l'itération dans une boucle

double Angle; //Permet de transmettre un angle à une articulation

///////////////////////////////////////////////////////////////////////////////////
//Déclaration du tableau contenant les paramètres de DH, 23 repères
double DH[20][4] = {35.1,0,PI2,-PI2,
	72.35,0,-PI2,0,
	0,0,PI2,PI2,
	99.95,0,0,-PI2,
	92,0,0,0,
	-36.35,0,PI2,PI,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	-35.1,0,PI2,-PI2,
	-72.35,0,-PI2,0,
	0,0,PI2,PI2,
	99.95,0,0,-PI2,
	92,0,0,0,
	-36.35,0,PI2,PI,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0};
//Paramètres DH de la jambe droite
double DH_01[4] = {35.1,0,PI2,-PI2};
double DH_12[4] = {72.35,0,-PI2,0};
double DH_23[4] = {0,0,PI2,PI2};
double DH_34[4] = {99.95,0,0,-PI2};
double DH_45[4] = {92,0,0,0};
double DH_56[4] = {-36.35,0,PI2,PI};
double DH_67[4] = {0,0,0,0};
double DH_68[4] = {0,0,0,0};
double DH_69[4] = {0,0,0,0};
double DH_610[4] = {0,0,0,0};
//Paramètres DH de la jambe gauche
double DH_011[4] = {-35.1,0,PI2,-PI2};
double DH_1211[4] = {-72.35,0,-PI2,0};
double DH_1213[4] = {0,0,PI2,PI2};
double DH_1314[4] = {99.95,0,0,-PI2};
double DH_1415[4] = {92,0,0,0};
double DH_1516[4] = {-36.35,0,PI2,PI};
double DH_1617[4] = {0,0,0,0};
double DH_1618[4] = {0,0,0,0};
double DH_1619[4] = {0,0,0,0};
double DH_1620[4] = {0,0,0,0};

///////////////////////////////////////////////////////////////////////////////////
//Déclaration du tableau angle et coordonnées de chaque articulation [angle,X,Y,Z]
double AngleCoord[20][4];


///////////////////////////////////////////////////////////////////////////////////
//déclaration des variables matrices en extern
//Une MATRICE de TRANSLATION 
extern Matrix4d Mt;
//Une MATRICES RESULTAT 
extern Matrix4d Mres;
//Deux MATRICES de PASSAGE
extern Matrix4d Mp_A;
extern Matrix4d Mp_B;
//Deux MATRICES de PASSAGE INVERSE
extern Matrix4d Mpi_A;
extern Matrix4d Mpi_B;
//Une Matrice de rotation
extern Matrix4d Mrot;




///////////////////////////////////////////////////////////////////////////////////
//fonction calculs matricielles

void CalculMatricielle (int a, int b, double Angle)
{
	if (b<a)//Si b<A, nous sommes dans une matrice inverse __ exemple :: CalculMatricielle (16,11);
	std::cout << "Si b<A, nous sommes dans une matrice inverse" << std::endl;
	{
		int f=0;//Variable utiliser pour connaitre l'itération en cours

		for (f = a-1; f >= b; --f)//ATTENTION AVEC :: a-1, car le tableau commence à 0, donc 16 devient 15
		{
				if (iteration == 0)
				{
					
				IniMatriceMrot(Angle);//Ici nous passons l'angle de rotation à la matrice de rotation, c'est l'angle de la maléolle 

							std::cout << "Iteration 0 avec f =  " << f << std::endl;
							std::cout << "Valeur de la Matrice rotation :: " << std::endl;
							std::cout << Mrot << std::endl;
							std::cout << "*********************" << std::endl;

				IniMatriceMpi_A(DH[f][0], DH[f][1], DH[f][2], DH[f][3],0); //Passage de r,d,alpha,teta et angle de rotation de l'articulation
		
							std::cout << "Valeur de la Matrice Mpi_A  :: " << std::endl;
							std::cout << Mpi_A << std::endl;
							std::cout << "*********************" << std::endl;
			
				Mres = Mrot * Mpi_A;

							std::cout << "Valeur de la Matrice resultat  :: " << std::endl;
							std::cout << Mres << std::endl;
							std::cout << "*********************" << std::endl;

				//Récupération de l'angle de l'articulation de la maléolle, ici ce n'est pas f mais a, les coordonnées sont pour f ::
				AngleCoord[a][0]= Angle;//Angle
				//AngleCoord[f][0];//Angle, pas de changement
				AngleCoord[f][1]= Mres(0,3);//X
				AngleCoord[f][2]= Mres(1,3);//Y
				AngleCoord[f][3]= Mres(2,3);//Z


				}
				else if (iteration > 0)
				{
					std::cout << "Iteration > 0 avec f = " << f << std::endl;

					///////////////////////////////
					//IH f = 10 alors passer -angle
					///////////////////////////////

					IniMatriceMpi_A(DH[f][0], DH[f][1], DH[f][2], DH[f][3],AngleCoord[f][0]); //Passage de r,d,alpha,teta et angle de rotation de l'articulation
		
							std::cout << "Valeur de la Matrice Mpi_A  :: " << std::endl;
							std::cout << Mpi_A << std::endl;
							std::cout << "*********************" << std::endl;
					

							Mres = Mres * Mpi_A;

							std::cout << "Valeur de la Matrice resultat  :: " << std::endl;
							std::cout << Mres << std::endl;
							std::cout << "*********************" << std::endl;
				//Récupération de l'angle de l'articulation N° f et de ses coordonnées ::
				//AngleCoord[f][0];//Angle, pas de changement
				AngleCoord[f][1]= Mres(0,3);//X
				AngleCoord[f][2]= Mres(1,3);//Y
				AngleCoord[f][3]= Mres(2,3);//Z
				}
		std::cout << "BOUCLE :: " << std::endl;
		//Incrémentation de la variable itération 
		++iteration;
		}
			std::cout << "OOOOOOOOOOOOOOOOOOOOOO" << std::endl;
			std::cout << "a = " << a << std::endl;
			std::cout << "b = " << b << std::endl;
			std::cout << "f = " << f << std::endl;
			std::cout << "Iteration = " << iteration << std::endl;
			std::cout << "OOOOOOOOOOOOOOOOOOOOOO" << std::endl;
	}
}//FIN :: void CalculMatricielle (int a, int b)


///////////////////////////////////////////////////////////////////////////////////
//////////////////////PARTIE PRINCIPAL ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
int main(int argc, CHAR* argv[])
{


iteration =0;
Angle = 0; 
CalculMatricielle (16,10,Angle);//calcul des coordonnées de 16/15 jusqu'à 16/10, coordonnées de 15, 14, ....10 = 0, prendre en inverse la ligne inférieure => a-1




///////////////////////////////////////////////////////////////////////////////////
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
		_cprintf("robot found in this world\r\n");//Le robot à été trouvé

		//configuration des variables servomoteurs Jambes Gauche
		DeviceServoMotor *pServoMaleolle_G=robot->QueryDeviceServoMotor("phx0/HingeMaleolle_G/a1/ServoMaleolle_G");
		DeviceServoMotor *pServoCheville_G=robot->QueryDeviceServoMotor("phx0/HingeCheville_G/a1/ServoCheville_G");
		DeviceServoMotor *pServoGenou_G=robot->QueryDeviceServoMotor("phx0/HingeGenou_G/a1/ServoGenou_G");
		DeviceServoMotor *pServoJambe_G=robot->QueryDeviceServoMotor("phx0/HingeJambe_G/a1/ServoJambe_G");
		DeviceServoMotor *pServoPivot_G=robot->QueryDeviceServoMotor("phx0/HingeJambe_G/a2/ServoPivot_G");
		DeviceServoMotor *pServoHanche_G=robot->QueryDeviceServoMotor("phx0/HingeHanche_G/a1/ServoHanche_G");
		//configuration des variables servomoteurs Jambes Droite
		DeviceServoMotor *pServoMaleolle_D=robot->QueryDeviceServoMotor("phx0/HingeMaleolle_D/a1/ServoMaleolle_D");
		DeviceServoMotor *pServoCheville_D=robot->QueryDeviceServoMotor("phx0/HingeCheville_D/a1/ServoCheville_D");
		DeviceServoMotor *pServoGenou_D=robot->QueryDeviceServoMotor("phx0/HingeGenou_D/a1/ServoGenou_D");
		DeviceServoMotor *pServoJambe_D=robot->QueryDeviceServoMotor("phx0/HingeJambe_D/a1/ServoJambe_D");
		DeviceServoMotor *pServoPivot_D=robot->QueryDeviceServoMotor("phx0/HingeJambe_D/a2/ServoPivot_D");
		DeviceServoMotor *pServoHanche_D=robot->QueryDeviceServoMotor("phx0/HingeHanche_D/a1/ServoHanche_D");

		//Test des servomoteurs jambe gauche
		if(pServoMaleolle_G && pServoCheville_G && pServoGenou_G && pServoJambe_G && pServoPivot_G && pServoHanche_G)//si tous les servo trouvés
			{
				JambeGauche = true;
			}
		//test des servomoteurs jambe droite
		if(pServoMaleolle_D && pServoCheville_D && pServoGenou_D && pServoJambe_D && pServoPivot_D && pServoHanche_D)//si tous les servo trouvés
			{
				JambeDroite = true;
			}

		//Permet de tester si les servomoteurs sont trouvés...POUR TEST
		if(pServoJambe_G)
		{
			_cprintf("ServoJambe found ...\r\n");
			pConnection->Sleep(2000);
		}
				else
			{
			_cprintf("ServoJambe_G not found ...\r\n");
				pConnection->Sleep(2000);
			}

		if(pServoPivot_G)
		{
			_cprintf("ServoPivot_G found ...\r\n");
			pConnection->Sleep(2000);
		}
				else
			{
			_cprintf("ServoPivot_G not found ...\r\n");
				pConnection->Sleep(2000);
			}
		/////////////////////////////////////////////////////////////////////////////////////////////////


		if(JambeGauche && JambeDroite)//si tous les servo sont trouvés
			{		
			_cprintf("Testing servo1 + servo2\r\n");
			//float deg=0;


			while(!_kbhit())
				{
				//_cprintf("INITIALISATION ALL = 0°...\r\n");
				//pServoMaleolle_G->GoPositionDeg(0);
				//pServoTibia_G->GoPositionDeg(0);
				//pServoGenou_G->GoPositionDeg(0);
				//pServoJambe_G->GoPositionDeg(0);
				//pServoPivot_G->GoPositionDeg(0);


				pConnection->Sleep(2000);

				_cprintf(" ALL = 90°...\r\n");
				pServoMaleolle_G->GoPositionDeg(20);
				//pServoCheville_G->GoPositionDeg(90);
				//pServoGenou_G->GoPositionDeg(90);
				//pServoJambe_G->GoPositionDeg(0);
				//pServoPivot_G->GoPositionDeg(90);
				pServoHanche_G->GoPositionDeg(10);

				pServoMaleolle_D->GoPositionDeg(20);
				//pServoCheville_D->GoPositionDeg(90);
				//pServoGenou_D->GoPositionDeg(90);
				//pServoJambe_D->GoPositionDeg(0);
				//pServoPivot_D->GoPositionDeg(90);
				pServoHanche_D->GoPositionDeg(20);
				pConnection->Sleep(2000);
				}//fin de while


			delete pServoMaleolle_G;
			delete pServoCheville_G;
			delete pServoGenou_G;
			delete pServoJambe_G;
			delete pServoPivot_G;
			delete pServoHanche_G;
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


