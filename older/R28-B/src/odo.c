// ---------------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ---------------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: odo.c	
// Auteurs              : Nicolas Dos Santos, Jos� Mar�a Martin
// Revisions			: 
//			o 20061115 Creation du fichier
//			o 20061128 cr�ation de l'odom�trie mat�rielle
//						-> test globaux � faire!! => c'est bon
//			o 20060102 -> remplacement de l'odo materielle par une odo
//						  logicielle
//						-> rajout et renommage de fonctions
//						-> rajout d'un diviseur de nombres de traits par tour
//						-> constantes g�n�rales plac�es dans reglages.h
//						-> modification des entr�es de l'odometrie
//						-> tests exaustifs en simulation
// ---------------------------------------------------------------------
//----------------------------------------------------------------------
//FONCTIONNEMENT DE L'ODOMETRE
//
//*** branchement: ***
//roue 0  = INT0 = RE8  = VoieA = input (avec interruption)
//roue 1  = INT1 = RD0  = VoieA = input (avec interruption)

//*** remarque : ***
//Le comptage se fait sur front montant (et descendant) de la voie A de 
//chaque roue. Le comptage est sign� : les variables vont de -32768 �
// +32767. S'il y a d�passement, les resultats serint eronn�s.
//FIN FONCTIONNEMENT DE L'ODOMETRE
//----------------------------------------------------------------------

#include "p30fxxxx.h"
#include "odo.h"

//----------------------------------------------------------------------
//DECLARATIONS GLOBALES
//roue 0
	float DistanceRoue0_N = 0; // distance relative de l'odo branch� sur INT0 plage : [-32768,+32767]
	
//roue 1
	float DistanceRoue1_N = 0; // distance relative de l'odo branch� sur INT1 plage : [-32768,+32767]
	
//resolution des odometres
	static float DeltaX0 = 0 ; //resolution sur la mesure de distance sur la roue 0
	static float DeltaX1 = 0 ; //resolution sur la mesure de distance sur la roue 1
	
    short old_POSCNT = 0;
//FIN DECLARATIONS GLOBALES

#include "donnees.h"
extern ASSERV Longitudinal;
extern ASSERV Angulaire;
extern ASSERV RoueD;
extern ASSERV RoueG;
extern MECANIQUE Mecanique;
extern CARTE Carte;

#define N 200
#define Coef_ODO 2 // 1 <=> 1x, 2 <=> 2x, 4 <=> 4x
#define PI 3.141592654

//roue 0
//la voie A est fig�e car une interruption y est associ�e (INT0)
#define VOIE_0B PORTBbits.RB1

//----------------------------------------------------------------------

void ODOInitialize(void)
{
// ----------------------------------------------------------------
// Fonction : 		InitODO
// But 	    : 		Initialisation des registres des odometres
// ---------------------------------------------------------------- 
//INPUT   : rien 
//OUTPUT  : rien
//COMMENT : initialise la direction des ports et le niveau ainsi que la
//			polarit� des interruptions associ�es.
//			Calcule une fois pour toutes (gain de temps) le quantum (Delta_x) 
//			pour passer du nombre d'impulsions � une distance en mm

    //Entr�es en mode Num�rique
	ADPCFGbits.PCFG1 = 1;

	//Roue 0	
	TRISEbits.TRISE8  = 1	;//INT0 = RE8 = VoieA_D = input
	TRISBbits.TRISB1 = 1	;//		  RB1= VoieB_D = input
	
    //init interruptions
	//Priorit� de 7
	IPC0bits.INT0IP = 7;
	IPC4bits.INT1IP = 7;
	
	INTCON2bits.INT0EP = 0	;//bit 0//Polarity interrup :0 = positive edge
	INTCON2bits.INT1EP = 1	;//bit 0//Polarity interrup :1 = negative edge
	
	QEIInitialize();
		
	//resolution des odometres
	ODOCalculerDeltaX();
}//

//resolution des odometres
void ODOCalculerDeltaX()
{
	DeltaX0 = Mecanique.DiametreRoueD * PI / (Coef_ODO * N);
	DeltaX1 = Mecanique.DiametreRoueG * PI / (Coef_ODO * N);
}	

void ODOStart(void){
// ----------------------------------------------------------------
// Fonction : 		Start_ODO
// But 	    : 		Autorise les interruptions associ�es aux odometres
// ---------------------------------------------------------------- 
	IEC0bits.INT0IE = 1;//interruptions autoris�e
	IEC1bits.INT1IE = 1;//interruptions autoris�e	
}

void ODOStop(void){
// ----------------------------------------------------------------
// Fonction : 		ODOStop
// But 	    : 		Interdit les interruptions associ�es aux odometres
// ---------------------------------------------------------------- 
	IEC0bits.INT0IE = 0;//interruptions autoris�e
	IEC1bits.INT1IE = 0;//interruptions autoris�e
}


void ODORaz(void){
// ----------------------------------------------------------------
// Fonction : 		ODORaz
// But 	    : 		Remet a zero la mesure odometrique
// ---------------------------------------------------------------- 
	DistanceRoue0_N = 0;
	DistanceRoue1_N = 0;
}//

void ODOAjouterAngle(float angle)
{
	DistanceRoue0_N -= Mecanique.EntreAxe * angle / 2.0;
	DistanceRoue1_N -= Mecanique.EntreAxe * angle / 2.0;
}	

void ODOSetDistanceRoue0_N(float val)
{
	DistanceRoue0_N = val;
}

void ODOSetDistanceRoue1_N(float val)
{
	DistanceRoue1_N = val;
}

float ODODistRoue0_N(void){
// ----------------------------------------------------------------
// Fonction : 		ODODistRoue0
// But 	    : 		Renvoie la distance parcourue par la roue n�0
//					en m
// ---------------------------------------------------------------- 
//OUTPUT  : distance lin�aire droite [m]
	/*Longitudinal.Desire += (Longitudinal.VitesseConsignePrecedente + Longitudinal.VitesseConsigne) / 2 * Carte.Te;
	Angulaire.Desire += (Angulaire.VitesseConsignePrecedente + Angulaire.VitesseConsigne) / 2 * Carte.Te;
	return Longitudinal.Desire + Angulaire.Desire * Mecanique.EntreAxe / 2.0;//Pour le debug*/
	
	//RoueD.Desire += (RoueD.VitesseConsignePrecedente + RoueD.VitesseConsigne) / 2.0 * Carte.Te;
	//return RoueD.Desire; 
	return -DistanceRoue0_N; //Pour de vrai
}

float ODODistRoue1_N(void){
// ----------------------------------------------------------------
// Fonction : 		ODODistRoue1
// But 	    : 		Renvoie la distance parcourue par la roue n�1
//					en m
// ---------------------------------------------------------------- 
//OUTPUT  : distance lin�aire droite [m]
	//return Longitudinal.Desire - Angulaire.Desire * Mecanique.EntreAxe / 2.0;//Pour le debug
	//RoueG.Desire += (RoueG.VitesseConsignePrecedente + RoueG.VitesseConsigne) / 2.0 * Carte.Te;
	//return RoueG.Desire; 
	return DistanceRoue1_N;	//Pour de vrai
}//

void _ISRFAST _INT0Interrupt(void)
{
// ----------------------------------------------------------------
// Fonction : 		pas de fonction
// But 	    : 		traitement de l'interruption sur INT0
// ---------------------------------------------------------------- 
	LATEbits.LATE4 ^= 1;	
	if (INTCON2bits.INT0EP == 0)
	{//On est juste apr�s un front montant sur la voie A
		if (VOIE_0B == 1)
		{//on recule
			DistanceRoue0_N -= DeltaX0;//decr�mentation
		}
		else
		{//La voie B vaut 0 => on avance
			DistanceRoue0_N += DeltaX0;	
		}
	}
	else
	{//On est juste apr�s un front descendant
		if (VOIE_0B == 1)
		{//on avance
			DistanceRoue0_N += DeltaX0;
		}
		else
		{//La voie B vaut 0 => on recule
			DistanceRoue0_N -= DeltaX0;		
		}
	}
	IFS0bits.INT0IF = 0;//reset flag
}

void _ISRFAST _INT1Interrupt(void)
{
// ----------------------------------------------------------------
// Fonction : 		pas de fonction
// But 	    : 		traitement de l'interruption sur INT1
// ---------------------------------------------------------------- 
	LATEbits.LATE4 ^= 1;
	if (INTCON2bits.INT1EP == 0)
	{//On est juste apr�s un front montant sur la voie A
		if (VOIE_0B == 1)
		{//on recule 
			DistanceRoue0_N -= DeltaX0;//decr�mentation
		}
		else
		{//La voie B vaut 0 => on avance
			DistanceRoue0_N += DeltaX0;	
		}
	}
	else
	{//On est juste apr�s un front descendant
		if (VOIE_0B == 1)
		{//on avance
			DistanceRoue0_N += DeltaX0;
		}
		else
		{//La voie B vaut 0 => on recule
			DistanceRoue0_N -= DeltaX0;		
		}
	}
	IFS1bits.INT1IF = 0;//reset flag
}

void QEIInitialize(void)
{	
    //Entr�es en mode Num�rique
	ADPCFGbits.PCFG4 = 1;
	ADPCFGbits.PCFG5 = 1;
    
	//Roue 1
	TRISBbits.TRISB4 = 1	;//   QEA => Input
	TRISBbits.TRISB5 = 1	;//   QEB => Input
    
    // Configuration du module
    //QEICONbits.UDSRC = 1; // Le sens de rotation est d�termin� � partie du canal B et non pas � partir de l'entr�e UPDN.
    QEICON = 1;
    QEICONbits.CNTERR = 0; // Pas d'erreur
    QEICONbits.QEISIDL = 0; // Le QEI ne se met pas en veille avec le �C
    QEICONbits.QEIM = 5; // QEI Mode : Activ�, 2x, pas d'index, reset si POSCNT = MAXCNT
    QEICONbits.SWPAB = 0; // Canal A et B non invers�s (pour s'adapter au cablage)
    QEICONbits.PCDOUT = 0; // Pas de sortie direction sur les I/O (patte UPDN non pr�sente sur le dsPIC30F4012 en plus)
    QEICONbits.POSRES = 0; // Pas de reset si signal index
    DFLTCONbits.QEOUT = 0; // Filtres passe bas num�riques sur les entr�es : d�sactiv�s
    
    // Initialisation du compteur
    POSCNT = 0;
    MAXCNT = 0xFFFF;
    
    old_POSCNT = POSCNT;
}//

float QEIDistanceRoue1(void)
{
    short tmp = POSCNT;
	short Vitesse = tmp - old_POSCNT;
	old_POSCNT = tmp;
	
	LATEbits.LATE5 = tmp;
	
	DistanceRoue1_N += (float)(Vitesse) * DeltaX1;
	
	return DistanceRoue1_N;
}
