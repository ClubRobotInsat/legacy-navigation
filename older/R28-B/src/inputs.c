#include "inputs.h"
#include "donnees.h"

#define N 200.0
#define PI 3.141592654

int lastPORTB = 0;

extern MECANIQUE Mecanique;

float DistanceRoue0 = 0.0;
float DistanceRoue1 = 0.0;	

float DeltaX0;
float DeltaX1;

void ODO0moins(void)
{
	DistanceRoue0 -= DeltaX0;
	LATEbits.LATE4 ^= 1;
}

void ODO0plus(void)
{
	DistanceRoue0 += DeltaX0;
	LATEbits.LATE4 ^= 1;
}

void ODO1moins(void)
{
	DistanceRoue1 -= DeltaX1;
	LATEbits.LATE5 ^= 1;
}

void ODO1plus(void)
{
	DistanceRoue1 += DeltaX1;
	LATEbits.LATE5 ^= 1;
}
	
//Pour le debug :
extern ASSERV Longitudinal;
extern ASSERV Angulaire;
extern CARTE Carte;
	
float ODODistRoue0(void)
{
	//Pour simuler les odom�tres
	//DistanceRoue0 += Longitudinal.VitesseConsigne * Carte.Te + Angulaire.VitesseConsigne * Carte.Te;
	return DistanceRoue0;
}
	
float ODODistRoue1(void)
{
	//Pour simuler les odom�tres
	//DistanceRoue1 += Longitudinal.VitesseConsigne * Carte.Te - Angulaire.VitesseConsigne * Carte.Te;
	return -DistanceRoue1;
}
	
void _ISRFAST _CNInterrupt(void) //Une patte sur le PORTB ou PORTC a �t� modifi�e
{
	int actPORTB = PORTB; //R�cup�ration de l'�tat actuel du PORTB
	
	int Diff = abs(actPORTB - lastPORTB); //Vu que une seule patte a chang� d'�tat, Diff est un multiple de deux, reste � savoir lequel pour d�terminer la patte ayant chang�
	
	if(Diff & RB0) //Si c'est RB0 qui a chang�
	{			
		if(actPORTB & RB0) //Si RB0 est � 1
		{
			//L'entr�e est pass�e � 1
			if(actPORTB & RB1) //Si RB1 est � 1
			{
				//La voie B est en avance sur la voie A => Sens positif
				ODO0plus();
			}
			else
			{
				//La voie A est en avance sur la voie B => Sens negatif
				ODO0moins();
			}
		}
		else
		{
			/*
			//L'entr�e est pass�e � 0
			if(actPORTB & RB1) //Si RB1 est � 1
			{
				//La voie A est en avance sur la voie B => Sens negatif
				ODO0moins();
			}
			else
			{
				//La voie B est en avance sur la voie A => Sens positif
				ODO0plus();
			}
			*/
		}
	}
	
	if(Diff & RB2) //Si c'est RB2 qui a chang�
	{			
		if(actPORTB & RB2) //Si RB2 est � 1
		{
			//L'entr�e est pass�e � 1
			if(actPORTB & RB3) //Si RB3 est � 1
			{
				//La voie B est en avance sur la voie A => Sens positif
				ODO1plus();
			}
			else
			{
				//La voie A est en avance sur la voie B => Sens negatif
				ODO1moins();
			}
		}
		else
		{
			/*
			//L'entr�e est pass�e � 0
			if(actPORTB & RB3) //Si RB3 est � 1
			{
				//La voie A est en avance sur la voie B => Sens negatif
				ODO1moins();
			}
			else
			{
				//La voie B est en avance sur la voie A => Sens positif
				ODO1plus();
			}
			*/
		}
	}
				
	lastPORTB = actPORTB;
	
	IFS0bits.CNIF = 0;
}

int INPUTInitialize(void)
{
	/*
		Les pattes 12, 11, 2, 3, 4, 5, 6, 7 peuvent �tre utilis�es pour g�rer des entr�es de type interrupteur.
		Les pattes 2 � 7 sont sur le PORTB, les pattes 12 et 11 sont sur le PORTC.
		L'ordre donn� correspond � leur num�ro de CN, de CN0 � CN7. (12 <=> CN0, 1<=> CN2, 7 <=> CN7)
		
		CNEN1 : Sur chaque patte il faut d�finir si elle est utilis�e ou non comme une entr�e type CN.
		CNUP1 : Sur chaque patte utilis�e comme CN, on peut activer ou non une r�sistance de pull-up au 5V.
		
		Interruptions : IFS0bits.CNIF, IEC0bits.CNIE, 	IPC3bits.CNIP (flag, enable, priority)
		
		Mat�riel : On dispose d'interrupteurs classiques, ainsi que d'une molette (encodeur m�canique, 2 phases en quadrature).
		On va utiliser 4 entr�es :
		-> MOLA, MOLB (les deux canaux en quadrature de la molette)
		-> SELECT, CANCEL
		
		L'utilisateur doit d�finir sur quelles pattes sont branch�es ces canaux.
		
		Il faut d�finir ces pattes comme des entr�es (TRISB) num�riques (ADPCFG).
		
		NB : Pour la molette, on ne mettra qu'une des deux voies en CN, 
		l'autre �tant uniquement lue lors de l'interruption pour connaitre le sens.
	*/
	
	/* Branchements choisis :
		-> ODOA sur RB0
		-> ODOB sur RB1
	*/
	
	DeltaX0 = Mecanique.DiametreRoueD * PI / N;
	DeltaX1 = Mecanique.DiametreRoueG * PI / N;
	
	//Definition des pattes en tant qu'entr�es
	TRISBbits.TRISB0 = 1; //MOLA0
	TRISBbits.TRISB1 = 1; //MOLB0
	TRISBbits.TRISB2 = 1; //MOLA1
	TRISBbits.TRISB3 = 1; //MOLB1
	
	//Configuration des pattes utilis�es comme des entr�es num�riques (et non analogiques)
	ADPCFGbits.PCFG0 = 1; //MOLA0
	ADPCFGbits.PCFG1 = 1; //MOLB0
	ADPCFGbits.PCFG2 = 1; //MOLA1
	ADPCFGbits.PCFG3 = 1; //MOLB1
		
	//Activation des interruptions
	CNEN1bits.CN2IE = 1; //MOLA0
	CNEN1bits.CN3IE = 0; //MOLB0
	CNEN1bits.CN4IE = 1; //MOLA1
	CNEN1bits.CN5IE = 0; //MOLB1
	
	//Configuration des r�sistances de pull-up
	CNPU1bits.CN2PUE = 0; //MOLA0
	CNPU1bits.CN3PUE = 0; //MOLB0
	CNPU1bits.CN4PUE = 0; //MOLA1
	CNPU1bits.CN5PUE = 0; //MOLB1
	
	//Configuration des interruptions
	IFS0bits.CNIF = 0; //On rabaisse le flag de l'interruption
	IPC3bits.CNIP = 7; //Priorit� de l'interruption : 7/7
	IEC0bits.CNIE = 1; //On autorise l'interruption	
	
	lastPORTB = PORTB;
}
