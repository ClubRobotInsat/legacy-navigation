#ifndef DONNEES_H
#define DONNEES_H

#define PI 3.141592654

typedef struct _MESSAGE
{
	unsigned int SID;
	unsigned long EID;
	unsigned char Est_Extended;
	unsigned char Nombre_Data;
	unsigned char Data[8];
} MESSAGE;

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
	
	int envoi_auto;
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
} MECANIQUE;

typedef struct _CARTE
{
	float Fcy;
	float Tcy;
	
	float Fe;
	float Te;
	
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
	float VitesseConsigne;
	float VitesseConsignePrecedente;
	
	float Acceleration;
	
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
		3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
		4: //Pointe vers Xc,Yc (Avant / Arrière)
		5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
		6: //VaZy Xc, Yc (Decompose, Avant/Arriere)
		7: //VaZy Xc, Yc (Avant/Arriere)
		8: //Translation à vitesse constante avec angle fixe
		9: //Rotation à vitesse constante sur place
		10: //Passer par le point Xc, Yc à vitesse constante
		11: //Pivot D
		12: //Pivot G
		13: //Position D&G
		14: //VitesseD&G
	*/
	char Type;
	char EnAvant; //1 <=> Avant, 0 <=> Arrière
	char Etape;
	char Termine;
	char TermineEnvoye;
	char TempsBlocage;
	
	char ConfigAsserv;

} MOUVEMENT;

#endif
