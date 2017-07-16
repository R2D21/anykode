
// fichier monModule.h
#ifndef _MATRICES_H
#define _MATRICES_H



///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de translation
void IniMatriceMt(double x, double y, double z);

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de passage A
void IniMatriceMp_A(double r, double d, double alpha, double teta, double angle);

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de passage B
void IniMatriceMp_B(double r, double d, double alpha, double teta, double angle);

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation d'une matrice de passage Inverse A
void IniMatriceMpi_A(double r, double d, double alpha, double teta, double angle);

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation d'une matrice de passage Inverse B
void IniMatriceMpi_B(double r, double d, double alpha, double teta, double angle);

///////////////////////////////////////////////////////////////////////////////////
//fonction initialisation de la matrice de rotation
void IniMatriceMrot(double angle);


#endif 