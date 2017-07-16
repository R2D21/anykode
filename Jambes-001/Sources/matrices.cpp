
#include "matrices.h"

#include <Eigen>
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////////
//Déclaration des valeurs PI et PI/2
# define PI (3.14159265358979323846) ///A mettre en extern
# define PI2 (1.57079632679489661923) /// A mettre en extern

///////////////////////////////////////////////////////////////////////////////////
//déclaration des variables matrices en extern
//Une MATRICE de TRANSLATION 
Matrix4d Mt(4,4);
//Une MATRICES RESULTAT 
Matrix4d Mres(4,4);
//Deux MATRICES de PASSAGE
Matrix4d Mp_A(4,4);
Matrix4d Mp_B(4,4);
//Deux MATRICES de PASSAGE INVERSE
Matrix4d Mpi_A(4,4);
Matrix4d Mpi_B(4,4);
//Une Matrice de rotation
Matrix4d Mrot(4,4);


///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de translation
void IniMatriceMt(double x, double y, double z)
	{	
		Mt(0,0) = 1;Mt(0,1) = 0;Mt(0,2) = 0;Mt(0,3) = x;
		Mt(1,0) = 0;Mt(1,1) = 1;Mt(1,2) = 0;Mt(1,3) = y;
		Mt(2,0) = 0;Mt(2,1) = 0;Mt(2,2) = 1;Mt(2,3) = z;
		Mt(3,0) = 0;Mt(3,1) = 0;Mt(3,2) = 0;Mt(3,3) = 1;
	}

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de passage A
void IniMatriceMp_A(double r, double d, double alpha, double teta, double angle)
	{
		angle = angle*PI/180;//Conversion de l'angle en radi
		teta = teta + angle; //Permet d'intégrer l'angle de la rotation
		Mp_A(0,0) = cos(teta);Mp_A(0,1) = -cos(alpha)*sin(teta);Mp_A(0,2) = sin(alpha)*sin(teta);Mp_A(0,3) = r*cos(teta);
		Mp_A(1,0) = sin(teta);Mp_A(1,1) = cos(alpha)*cos(teta);Mp_A(1,2) = -sin(alpha)*cos(teta);Mp_A(1,3) = r*sin(teta);
		Mp_A(2,0) = 0;Mp_A(2,1) = sin(alpha);Mp_A(2,2) = cos(alpha);Mp_A(2,3) = d;
		Mp_A(3,0) = 0;Mp_A(3,1) = 0;Mp_A(3,2) = 0;Mp_A(3,3) = 1;
	}

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de passage B
void IniMatriceMp_B(double r, double d, double alpha, double teta, double angle)
	{
		angle = angle*PI/180;//Conversion de l'angle en radi
		teta = teta + angle; //Permet d'intégrer l'angle de la rotation
		Mp_B(0,0) = cos(teta);Mp_B(0,1) = -cos(alpha)*sin(teta);Mp_B(0,2) = sin(alpha)*sin(teta);Mp_B(0,3) = r*cos(teta);
		Mp_B(1,0) = sin(teta);Mp_B(1,1) = cos(alpha)*cos(teta);Mp_B(1,2) = -sin(alpha)*cos(teta);Mp_B(1,3) = r*sin(teta);
		Mp_B(2,0) = 0;Mp_B(2,1) = sin(alpha);Mp_B(2,2) = cos(alpha);Mp_B(2,3) = d;
		Mp_B(3,0) = 0;Mp_B(3,1) = 0;Mp_B(3,2) = 0;Mp_B(3,3) = 1;
	}

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation d'une matrice de passage Inverse A
void IniMatriceMpi_A(double r, double d, double alpha, double teta, double angle)
	{		
		angle = angle*PI/180;//Conversion de l'angle en radi
		teta = teta + angle; //Permet d'intégrer l'angle de la rotation
		Mpi_A(0,0) = cos(teta);Mpi_A(0,1) = sin(teta);Mpi_A(0,2) = 0;Mpi_A(0,3) = -r;
		Mpi_A(1,0) = -cos(alpha)*sin(teta);Mpi_A(1,1) = cos(alpha)*cos(teta);Mpi_A(1,2) = sin(alpha);Mpi_A(1,3) = -d*sin(alpha);
		Mpi_A(2,0) = sin(alpha)*sin(teta);Mpi_A(2,1) = -sin(alpha)*cos(teta);Mpi_A(2,2) = cos(alpha);Mpi_A(2,3) = -d*cos(alpha);
		Mpi_A(3,0) = 0;Mpi_A(3,1) = 0;Mpi_A(3,2) = 0;Mpi_A(3,3) = 1;
	}

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation d'une matrice de passage Inverse B
void IniMatriceMpi_B(double r, double d, double alpha, double teta, double angle)
	{
		angle = angle*PI/180;//Conversion de l'angle en radi
		teta = teta + angle; //Permet d'intégrer l'angle de la rotation
		Mpi_B(0,0) = cos(teta);Mpi_B(0,1) = sin(teta);Mpi_B(0,2) = 0;Mpi_B(0,3) = -r;
		Mpi_B(1,0) = -cos(alpha)*sin(teta);Mpi_B(1,1) = cos(alpha)*cos(teta);Mpi_B(1,2) = sin(alpha);Mpi_B(1,3) = -d*sin(alpha);
		Mpi_B(2,0) = sin(alpha)*sin(teta);Mpi_B(2,1) = -sin(alpha)*cos(teta);Mpi_B(2,2) = cos(alpha);Mpi_B(2,3) = -d*cos(alpha);
		Mpi_B(3,0) = 0;Mpi_B(3,1) = 0;Mpi_B(3,2) = 0;Mpi_B(3,3) = 1;
	}
///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de rotation

void IniMatriceMrot(double angle)
	{
		angle = angle*PI/180;//Conversion de l'angle en radian

		Mrot(0,0) = cos(angle);Mrot(0,1) = -sin(angle);
		Mrot(1,0) = sin(angle);Mrot(1,1) = cos(angle);
		Mrot(2,2) = 1;
		Mrot(3,3) = 1;
	}




