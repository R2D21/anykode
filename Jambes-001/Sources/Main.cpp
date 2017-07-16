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
//Déclaration du tableau contenant les paramètres de DH-Direct, 24 repères MAIS 25 lignes ET ligne 0 non exploitée
//P0/1 = Mt_0 * Mp_O/1
//Utilsation de l'indice de droite = ligne ds tableau DHD
double DHD[25][4] = {
	0,0,0,0,
	35.1,0,0,-PI2,
	-72.35,0,PI2,PI,
	0,0,PI2,PI2,
	-99.95,0,0,0,
	-92,0,0,0,
	0,0,-PI2,0,
	-20,0,PI2,0,
	0,0,PI2,PI2,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	-35.1,0,0,-PI2,
	72.35,0,PI2,PI,
	0,0,PI2,PI2,
	-99.95,0,0,0,
	-92,0,0,0,
	0,0,-PI2,0,
	-20,0,PI2,0,
	0,0,PI2,PI2,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0};

///////////////////////////////////////////////////////////////////////////////////
//Déclaration du tableau contenant les paramètres de DH-Direct, 24 repères ET 24 lignes CAR ligne 0 exploitée = Mpi 1/0
//Pos°1/0 <-- Mpi_1/0
//Pos°0/8 = Pos°8/1 * Mpi_1/0:: utilisation de l'indice 1 du tableau
//Utilsation de l'indice de droite = ligne ds tableau DHI
double DHI[25][4] = {
	35.1,0,0,-PI2,
	-72.35,0,PI2,PI,
	0,0,PI2,PI2,
	-99.95,0,0,0,
	-92,0,0,0,
	0,0,-PI2,0,
	-20,0,PI2,0,
	0,0,PI2,PI2,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	-35.1,0,0,-PI2,
	72.35,0,PI2,PI,
	0,0,PI2,PI2,
	-99.95,0,0,0,
	-92,0,0,0,
	0,0,-PI2,0,
	-20,0,PI2,0,
	0,0,PI2,PI2,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0};
///////////////////////////////////////////////////////////////////////////////////
//Déclaration du tableau contenant les coordonnées de chaque articulation (12), de chaque origine pied (4), chaque extrémités pieds (8) et du CMG (1) : [X,Y,Z] 
double Coord[25][3];//0 = CMG, de 1 à 6 articulations pied droit, de 7 à 8, origines pieds droit, de 9 à 12 extremités pied droit,  
//de 13 à 18 articulations pied gauche, 19 à 20 origine pied gauche, de 21 à 24 extrémitès pied gauche
//Déclaration du tableau contenant les angles avant mouvement de chaque articulation [angle]
double Angle_OLD[25];
//Déclaration du tableau contenant les angles pour effectuer le mouvement suivant, de chaque articulation [angle]
double Angle_NEW[25];


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

void CalculMatricielle (int a, int b, double Angle) //CalculMatricielle (8,0,Angle);
{
	if (b<a)//Si b<A, nous sommes dans une matrice inverse __ exemple :: CalculMatricielle (8,0) = Pos_8/0 :: Mt_8 * Mpi_7 * Mpi_6*....Mpi_0
	std::cout << "Si b<A, nous sommes dans une matrice inverse" << std::endl;
	{
		int iteration=0;//Variable utilisé pour connaitre l'itération en cours

		//Il faut dabord initialiser la matrice de translation du point8 avec les coordonnées du point 8
		//Puis multiplier Mresul = Mt8 * Mpi-7
		//Puis Mresul = Mresul * Mpi_6*.....*Mpi_0

		//DHI[25][4]

		for (int f = a; f >= b; --f)//ATTENTION 
		{
			//Coord[25][3], récupdes coord de 8 pour constistuer Mt8

				if (iteration == 0)
				{
					IniMatriceMt(Coord[f+1][0], Coord[f+1][1], Coord[f+1][2]); //IniMatriceMt(double x, double y, double z)//Matrice de translation avec les coordonnées du point f+1 = origine du pied
					
					IniMatriceMpi_A(DHI[f][0], DHI[f][1], DHI[f][2], DHI[f][3],Angle); //Passage de r,d,alpha,teta et angle = -Angle de rotation de l'articulation : pied
					Mres = Mt * Mpi_A;


					/*std::cout << "a = " << a << std::endl;
					std::cout << "b = " << b << std::endl;
					*/
					std::cout << "f = " << f << std::endl;
					/*
					std::cout << "Iteration = 0 = " << iteration<< std::endl;
					std::cout << "-----------------------" << std::endl;
					std::cout << "Valeur de la Matrice translation  :: " << std::endl;
					std::cout << Mt << std::endl;
					std::cout << "-----------------------" << std::endl;
					std::cout << "r = " << DHI[f][0] << std::endl;
					std::cout << "d = " << DHI[f][1] << std::endl;
					std::cout << "alpfa  = " << DHI[f][3] << std::endl;
					std::cout << "teta = " << DHI[f][3] << std::endl;
					std::cout << "Valeur de la Matrice Mpi_A  :: " << std::endl;
					std::cout << Mpi_A << std::endl;
					std::cout << "-----------------------" << std::endl;
					*/
					std::cout << "Valeur de la Matrice resultat  :: " << std::endl;
					std::cout << Mres << std::endl;
					std::cout << "*********************" << std::endl;

					Coord[f][0] = Mres(0,3);
					Coord[f][1] = Mres(1,3);
					Coord[f][2] = Mres(2,3);
				}
				else
				{
					///////////////////////////////
					//SI f = 10 alors passer -angle
					///////////////////////////////
					IniMatriceMpi_A(DHI[f][0], DHI[f][1], DHI[f][2], DHI[f][3],Angle); //Passage de r,d,alpha,teta et angle = -Angle de rotation de l'articulation a
					Mres = Mres * Mpi_A;

					/*std::cout << "a = " << a << std::endl;
					std::cout << "b = " << b << std::endl;
					*/
					std::cout << "f = " << f << std::endl;
					/*
					std::cout << "Iteration = 1 = " << iteration<< std::endl;
					std::cout << "-----------------------" << std::endl;
					std::cout << "r = " << DHI[f][0] << std::endl;
					std::cout << "d = " << DHI[f][1] << std::endl;
					std::cout << "alpfa  = " << DHI[f][3] << std::endl;
					std::cout << "teta = " << DHI[f][3] << std::endl;
					std::cout << "Valeur de la Matrice Mpi_A  :: " << std::endl;
					std::cout << Mpi_A << std::endl;
					std::cout << "-----------------------" << std::endl;
					*/
					std::cout << "Valeur de la Matrice resultat  :: " << std::endl;
					std::cout << Mres << std::endl;
					std::cout << "*********************" << std::endl;

					Coord[f][0] = Mres(0,3);
					Coord[f][1] = Mres(1,3);
					Coord[f][2] = Mres(2,3);
				}
				/*else if (iteration > 1)
				{
					
					std::cout << " f = " << f << std::endl;

					///////////////////////////////
					//SI f = 10 alors passer -angle
					///////////////////////////////
					IniMatriceMpi_A(DHI[f][0], DHI[f][1], DHI[f][2], DHI[f][3],Angle); //Passage de r,d,alpha,teta et angle = 0 de rotation de l'articulation
					
					Mres = Mres * Mpi_A;

					std::cout << "a = " << a << std::endl;
					std::cout << "b = " << b << std::endl;
					std::cout << "f = " << f << std::endl;
					std::cout << "Iteration > 1 = " <<iteration<< std::endl;
					std::cout << "-----------------------" << std::endl;
					std::cout << "Valeur de la Matrice Mpi_A  :: " << std::endl;
					std::cout << Mpi_A << std::endl;
					std::cout << "-----------------------" << std::endl;

					std::cout << "Valeur de la Matrice resultat  :: " << std::endl;
					std::cout << Mres << std::endl;
					std::cout << "*********************" << std::endl;
				}*/

					/*if (f==(a-1))
					{

						IniMatriceMpi_A(DH[f][0], DH[f][1], DH[f][2], DH[f][3],-Angle); //Passage de r,d,alpha,teta et angle = -Angle de rotation de l'articulation a
						Mres = Mt * Mpi_A;


					}
					else
					{
						IniMatriceMpi_A(DH[f][0], DH[f][1], DH[f][2], DH[f][3],0); //Passage de r,d,alpha,teta et angle = 0 de rotation de l'articulation
					}*/
		

				//Récupération de l'angle de l'articulation N° f et de ses coordonnées ::
				//AngleCoord[f][0];//Angle, pas de changement
				Coord[f][0]= Mres(0,3);//X
				Coord[f][1]= Mres(1,3);//Y
				Coord[f][2]= Mres(2,3);//Z
		
		std::cout << "OOOOOOOOOOOOOOOOOOOOOO" << std::endl;
		std::cout << "BOUCLE :: " << std::endl;
		std::cout << "OOOOOOOOOOOOOOOOOOOOOO" << std::endl;


		//Incrémentation de la variable itération 
		++iteration;
		}//FIN :: for (int f = a; f >= b; --f)//ATTENTION 

	}
}//FIN :: void CalculMatricielle (int a, int b)


///////////////////////////////////////////////////////////////////////////////////
//////////////////////PARTIE PRINCIPAL ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
int main(int argc, CHAR* argv[])
{


iteration =0;
Angle = 0; 
Coord[8][0] = 0;
Coord[8][1] = 0;
Coord[8][2] = 0;
//CalculMatricielle (8,0,Angle) --> calcul des coordonnées de 8/0, Pos_8/7 * Pos_7/6*...Pos_1/0
CalculMatricielle(7,0,Angle);

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
		DeviceServoMotor *pServoMaleolle_G=robot->QueryDeviceServoMotor("phx0/Hingecheville_G/maleolle_G/ServoMaleolle_G");
		DeviceServoMotor *pServoCheville_G=robot->QueryDeviceServoMotor("phx0/HingeCheville_G/cheville_G/ServoCheville_G");
		DeviceServoMotor *pServoGenou_G=robot->QueryDeviceServoMotor("phx0/HingeGenou_G/a1/ServoGenou_G");
		DeviceServoMotor *pServoJambe_G=robot->QueryDeviceServoMotor("phx0/HingeHanche_G/jambe_G/ServoJambe_G");
		DeviceServoMotor *pServoHanche_G=robot->QueryDeviceServoMotor("phx0/HingeHanche_G/hanche_G/ServoHanche_G");
		DeviceServoMotor *pServoPivot_G=robot->QueryDeviceServoMotor("phx0/HingePivot_G/a1/ServoPivot_G");
		//configuration des variables servomoteurs Jambes Droite
		DeviceServoMotor *pServoMaleolle_D=robot->QueryDeviceServoMotor("phx0/HingeCheville_D/maleolle_D/ServoMaleolle_D");
		DeviceServoMotor *pServoCheville_D=robot->QueryDeviceServoMotor("phx0/HingeCheville_D/cheville_D/ServoCheville_D");
		DeviceServoMotor *pServoGenou_D=robot->QueryDeviceServoMotor("phx0/HingeGenou_D/a1/ServoGenou_D");
		DeviceServoMotor *pServoJambe_D=robot->QueryDeviceServoMotor("phx0/HingeHanche_D/jambe_D/ServoJambe_D");
		DeviceServoMotor *pServoHanche_D=robot->QueryDeviceServoMotor("phx0/HingeHanche_D/Hanche_D/ServoHanche_D");
		DeviceServoMotor *pServoPivot_D=robot->QueryDeviceServoMotor("phx0/HingePivot_D/a1/ServoPivot_D");

		//Test des servomoteurs jambe gauche
		if(pServoMaleolle_G && pServoCheville_G && pServoGenou_G && pServoJambe_G && pServoPivot_G && pServoHanche_G)//si tous les servo trouvés
			{
				JambeGauche = true;
				_cprintf("JambeGauche found ...\r\n");
			}
		//test des servomoteurs jambe droite
		if(pServoMaleolle_D && pServoCheville_D && pServoGenou_D && pServoJambe_D && pServoPivot_D && pServoHanche_D)//si tous les servo trouvés
			{
				JambeDroite = true;
			    _cprintf("jambeDroite found ...\r\n");
			}
		/////////////////////////////////////////////////////////////////////////////////////////////////


		if(JambeGauche && JambeDroite)//si tous les servo sont trouvés
			{		
			_cprintf("Jambe Gauche et Droite found\r\n");
			//float deg=0;


			while(!_kbhit())
				{

				///////////////////////////////////////////////////////////////////////////
				pConnection->Sleep(2000);
				_cprintf("Initialisation...\r\n");

				pServoMaleolle_G->GoPositionDeg(0);
				pServoCheville_G->GoPositionDeg(0);
				pServoJambe_G->GoPositionDeg(0);
				pServoHanche_G->GoPositionDeg(0);
				pServoGenou_G->GoPositionDeg(0);
				pServoPivot_G->GoPositionDeg(0);
				
				///////////////////////////////////
				pServoMaleolle_D->GoPositionDeg(0);
				pServoCheville_D->GoPositionDeg(0);
				pServoJambe_D->GoPositionDeg(0);
				pServoHanche_D->GoPositionDeg(0);
				pServoGenou_D->GoPositionDeg(0);
				pServoPivot_D->GoPositionDeg(0);
				///////Récupération des angles 


				///////////////////////////////////////////////////////////////////////////
				
			//now, at low speed
			//pServo1->SetEffectiveMaxVelocityDPS(90);
						
				pConnection->Sleep(2000);
				_cprintf("Inclinaison gauche°...\r\n");

				pServoHanche_G->GoPositionDeg(-20);
				//Angle_NEW[a]=0;
				pServoMaleolle_G->GoPositionDeg(20);
				//Angle_NEW[a]=0;
				
				
				/////////////////////////////////////
				pServoMaleolle_D->GoPositionDeg(20);
				//Angle_NEW[a]=0;
				pServoHanche_D->GoPositionDeg(-20);

				///////////////////////////////////////////////////////////////////////////
				pConnection->Sleep(2000);
				_cprintf("Léve pied gauche°...\r\n");

				//Save configuration values
			   float maxvel=pServoJambe_G->GetMaxVelocityDPS();
			   _cprintf("Torque regulation: ",maxvel);
			   _cprintf("Torque regulation: max speed %f °/s\r\n",maxvel);

				pServoGenou_G->SetEffectiveMaxVelocityDPS(60);
				pServoJambe_G->SetEffectiveMaxVelocityDPS(29);
				pServoCheville_G->SetEffectiveMaxVelocityDPS(31);


				pServoGenou_G->GoPositionDeg(74);
				pServoJambe_G->GoPositionDeg(-35);
				pServoCheville_G->GoPositionDeg(-38);
				//pServoPivot_G->GoPositionDeg(0);

				pConnection->Sleep(2000);
				pServoGenou_G->GoPositionDeg(0);
				pServoJambe_G->GoPositionDeg(0);
				pServoCheville_G->GoPositionDeg(0);

				pServoGenou_G->SetEffectiveMaxVelocityDPS(60);
				pServoJambe_G->SetEffectiveMaxVelocityDPS(60);
				pServoCheville_G->SetEffectiveMaxVelocityDPS(60);


				/////////////////////////////////////

				
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


}  //FIN MAIN

