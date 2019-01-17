#ifndef ODO_H
#define ODO_H
// ---------------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ---------------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: odo.h	
// Auteurs              : Nicolas Dos Santos, José María Martin
// Description			: Fichier d'entête
// Revisions			: 
//			o 20061208 Creation du fichier
//						
// ---------------------------------------------------------------------

void ODOInitialize(void);		//initialise les périphériques
void ODOCalculerDeltaX();
void ODORaz(void);		//Pour remettre à zéro les odometres
void ODOStart(void);	//Pour demarer les odometres
void ODOStop(void);	//Pour arreter sans remettre à zéro les odometres
void ODOSelect(char S);//Selection une paire d'odometres
float ODODistRoue0_N(void);//distance [m] parcourue par la roue 0
float ODODistRoue1_N(void);//distance [m] parcourue par la roue 1
void ODOAjouterAngle(float angle);
void ODOSetDistanceRoue0_N(float val);
void ODOSetDistanceRoue1_N(float val);

#endif
