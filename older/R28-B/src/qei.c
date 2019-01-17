#include "p30fxxxx.h"
#include "qei.h"

//----------------------------------------------------------------------
//DECLARATIONS GLOBALES
//roue 1
	float DistanceRoue1 = 0; // distance relative de l'odo branch� sur QEI1

//resolution des odometres
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
#define PI 3.141592654

//----------------------------------------------------------------------

void QEIInitialize(void)
{	
    //Entr�es en mode Num�rique
	ADPCFGbits.PCFG4 = 1;
	ADPCFGbits.PCFG5 = 1;
    
	//Roue 1
	TRISBbits.TRISB4 = 1	;//   QEA => Input
	TRISBbits.TRISB5 = 1	;//   QEB => Input
    
    // Configuration du module
    QEICONbits.CNTERR = 0; // Pas d'erreur
    QEICONbits.QEISIDL = 0; // Le QEI ne se met pas en veille avec le �C
    QEICONbits.QEIM = 5; // QEI Mode : Activ�, 2x, pas d'index, reset si POSCNT = MAXCNT
    QEICONbits.SWPAB = 0; // Canal A et B non invers�s (pour s'adapter au cablage)
    QEICONbits.PCDOUT = 0; // Pas de sortie direction sur les I/O (patte UPDN non pr�sente sur le dsPIC30F4012 en plus)
    QEICONbits.POSRES = 0; // Pas de reset si signal index
    QEICONbits.UDSRC = 1; // Le sens de rotation est d�termin� � partie du canal B et non pas � partir de l'entr�e UPDN.
    
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
	
	DistanceRoue1 += (float)(Vitesse) * DeltaX1;
	
	return DistanceRoue1;
}
