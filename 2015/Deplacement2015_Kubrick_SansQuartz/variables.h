#ifndef VARIABLES_H
#define VARIABLES_H

#define PI 3.141592654

typedef struct _ETAT
{
	float DistanceTotale;
	float DistanceRoueD;
	float DistanceRoueG;

	float AngleBrut;
	float AngleReel;

	float VitesseLongitudinale;
	float VitesseAngulaire;
	float VitesseRoueD;
	float VitesseRoueG;

	float X;
	float Y;

	int EnvoiAuto;
} ETAT;

typedef struct _MECANIQUE
{
	float EntreAxe;

	float DiametreRoueD;
	float DiametreRoueG;

	float PuissanceMoteurD;
	float PuissanceMoteurG;

	float PuissanceMaxD;
	float PuissanceMaxG;
	float PuissanceMaxLongitudinal;
	float PuissanceMaxAngulaire;

	float VitesseLongitudinaleMax;
	float VitesseAngulaireMax;

	float VitesseRoueDMax;
	float VitesseRoueGMax;

	int NombrePasD;
	int NombrePasG;
} MECANIQUE;

typedef struct _CARTE
{
	float Fcy;
	float Tcy;

	float Fe;
	float TeOdometrie;
	float TeAsserv;

	float Fpwm;
	float Tpwm;
} CARTE;

typedef struct _DEBUG
{
	float Record1[100];
	float Record2[100];
	int i;
} DEBUG;

typedef struct _ASSERV
{
	float Desire;
	float Consigne;

	float VitesseMax;
	float VitesseDemande;
	float VitesseConsigne;
	float VitesseConsignePrecedente;

	float Acceleration;
	float Decceleration;

	int Est_Active;
} ASSERV;

typedef struct _CORRECTEUR
{
	float Erreur;
	float ErreurPrecedente;
	float IntegraleErreur;
	float DeriveErreur;

	float CommandeProportionelle;
	float CommandeIntegrale;
	float CommandeDerive;

	float Kp;
	float Ki;
	float Kd;

	float KpArret;
	float KiArret;
	float KdArret;

	float KpRoule;
	float KiRoule;
	float KdRoule;

	float Commande;
} CORRECTEUR;

typedef struct _MOUVEMENT
{
	float DistanceDemande;
	float AngleDemande;

	float DistanceDroiteDemande;
	float DistanceGaucheDemande;

	float Xc;
	float Yc;

	float PrecisionLongitudinale;
	float PrecisionAngulaire;

	/* Types de mouvements :
		0: //Stop
		1: //Translation de DistanceDemande
		2: //Rotation relative de AngleDemande
		3: //Rotation absolue jusqu'� AngleDemande (Avant / Arri�re)
		4: //Pointe vers Xc,Yc (Avant / Arri�re)
		5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
		6: //VaZy Xc, Yc (Decompose, Avant/Arriere)
		7: //VaZy Xc, Yc (Avant/Arriere)
		8: //Translation � vitesse constante avec angle fixe
		9: //Rotation � vitesse constante sur place
		10: //Passer par le point Xc, Yc � vitesse constante
		11: //Pivot D
		12: //Pivot G
		13: //Position D&G
		14: //VitesseD&G
		15: //Arret urgence
		17: //VitesseL&A
	*/
	char Type;
	char EnAvant; //1 <=> Avant, 0 <=> Arri�re
	char Etape;
	char Termine;
	char TermineEnvoye;
	char TempsBlocage;

	char ConfigAsserv;

} MOUVEMENT;

void VARInitialize(CARTE *Carte, MECANIQUE *Mecanique, ASSERV *Longitudinal, ASSERV *Angulaire, ETAT *Etat, CORRECTEUR *PidPositionLongitudinale, CORRECTEUR *PidPositionAngulaire, CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, ASSERV *RoueD, ASSERV *RoueG, CORRECTEUR *PidPositionD, CORRECTEUR *PidPositionG, CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MOUVEMENT *MouvementActuel, int DemarageCarte);

#endif
