#ifndef FCT_ASSERV_H
#define FCT_ASSERV_H

#include "variables.h"

#define AsservPosLong	1
#define AsservVitLong	2
#define AsservPosAng	4
#define AsservVitAng	8

#define AsservPosD		16
#define AsservVitD		32
#define AsservPosG		64
#define AsservVitG		128

float Seuiller(float Val, float Abs);
float Mod2Pi(float Angle);
float fabs(float val);
int EstDansInterval(float Val, float Abs);

void CalculerEtat(ETAT *Etat, MECANIQUE *Mecanique, CARTE *Carte);

void CalculCorrecteur(float Mesure, float Consigne, CORRECTEUR *Pid, CARTE *Carte);

void InitierOuActualiserConsigneMouvement(MOUVEMENT *Mouvement, ETAT *Etat, ASSERV *Longitudinal, ASSERV *Angulaire, ASSERV *RoueD, ASSERV *RoueG, MECANIQUE *Mecanique);

void SeuillageVitesse(ASSERV *Param, CARTE *Carte);

void AsservPositionLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidPositionLongitudinale, CARTE *Carte);
void AsservPositionAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidPositionAngulaire, CARTE *Carte);
void AsservVitesseLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidVitesseLongitudinale, CARTE *Carte, MECANIQUE *Mecanique);
void AsservVitesseAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidVitesseAngulaire, CARTE *Carte, MECANIQUE *Mecanique);

void ApplicationPuissanceMoteurPolaire(CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, MECANIQUE *Mecanique);

void AsservPositionD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidPositionD, CARTE *Carte);
void AsservPositionG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidPositionG, CARTE *Carte);
void AsservVitesseD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidVitesseD, CARTE *Carte, MECANIQUE *Mecanique);
void AsservVitesseG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidVitesseG, CARTE *Carte, MECANIQUE *Mecanique);

void ApplicationPuissanceMoteurDG(CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MECANIQUE *Mecanique);

#endif
