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
int main(int argc, CHAR* argv[])
///////////////////////////////////////////////////////////////////////////////////
{

//Test matrice
  Matrix2d a;
  a << 10, 12,
       3, 4;
  Matrix2d b(2,2);
  b << 2, 3,
       1, 4;
  std::cout << "a + b =\n" << a + b << std::endl;
  std::cout << "a - b =\n" << a - b << std::endl;
  std::cout << "Doing a += b;" << std::endl;
  a += b;
  std::cout << "Now a =\n" << a << std::endl;

  std::cout << "XXXXXXXXXXXXXXXXXX" << a(0,0) << std::endl;

   std::cout << "XXXXXXXXXXXXXXXXXX" << a(0,1) << std::endl;

   a *= b;

std::cout << "Now a =\n" << a << std::endl;


//Fin test matrice


}