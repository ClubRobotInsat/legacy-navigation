// ---------------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ---------------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: pwm.c	
// Auteurs              : Nicolas Dos Santos (nicolas.dossantos.1@gmail.com)
// Description			: 
// Revisions			: 
//			o 20070102 correction de la formule de calcul de la freq de la PWM
//			o 20061204 Probleme de rapport / 2 corrigé mais on n'a pas  trouvé une explication
//			o 20061125 Configuation de registres
//			o 20061115 Creation du fichier
//						
// ---------------------------------------------------------------------

#include "p33fxxxx.h"
#include "pwm.h"
#include <math.h>

#define  	MY_PTPER 		1473				// Calcul:  PTPER = (Fcy* Prescaler / Freq PWM) - 1 = 1473

// ----------------------------------------------------------------
// Fonction : 		InitPWM
// But 	    : 		Initialisation de registres pour le PWM
// ---------------------------------------------------------------- 

void InitPWM(void){

	// Registre	        :	PTCON : PWM time base control register
	// Contenu		:	
	//		PTEN	= 0		:	Time base is ON
	//		PTSIDL	= 0		:	Time base runs in CPU idle mode
	//		PTOPS	= 0000	:	1:1 postscale
	//		PTCKS	= 00	:	1:1 prescale
	//		PTMOD	= 00	:	Free running mode
	//
	// Contenu final du registre :
	//		PCON = 1x0x xxxx 0000 0000 = 0x8000; 
	P1TCON = 0x0000;


	// Registre		:	PTPER : Timer base period register

	P1TPER = MY_PTPER;
	P1FLTACON = 0;


	// Registre	        :	PWMCON1 : PWM control register 1
	// Contenu		:	x=1,2,3
	//		PMODx	= 0		:	Complementary output mode
	//		PENxH	= 1		:	ON pour PWM
	//		PENxL	= 1		:	ON pour PWM
	//
	// Contenu final du registre :
	//		PCON = xxxx x000 x011 x011 = 0x0033; 
	//PWMCON1 = 0x0033;
	PWM1CON1bits.PMOD1 = 0;
	PWM1CON1bits.PMOD2 = 0;
	PWM1CON1bits.PEN1H = 1;
	PWM1CON1bits.PEN1L = 0;
	PWM1CON1bits.PEN2H = 1;
	PWM1CON1bits.PEN2L = 0;
 
	// Registre	    :	PWMCON2 : PWM control register 2
	// Contenu		:	
	//		IUE	= 1			:	Imediate Update 
	//
	// Contenu final du registre :
	//		PWMCON2 = xxxx xxxx xxxx x1xx = 0x0004; 
	PWM1CON2 = 0x0004;

	// Registre	        :	DTCON1 : Dead time control 1
	// Contenu		:	 NON UTILISE!
	//		REGx	= 1		:	
	//
	// Contenu final du registre :
	//		DTCON1 = 1x0x xxxx 0000 0000 = ; 

	// Registre	        :	DTCON2 : Dead time control 2
	// Contenu		:	 NON UTILISE!
	//		REGx	= 1		:	
	//
	// Contenu final du registre :
	//		DTCON1 = 1x0x xxxx 0000 0000 = 0x8000;

	// Registre	        :	OVDCON : Override Control register
	// Contenu		:	 x=1,2,3
	//		POVDx	= 1		:	IO pin controlled by PWM
	//		POUTx	= 0		: 	?????
	//
	// Contenu final du registre :
	//		OVDCON = 1111 1111 xxxx xxxx = 0xFF00;
	P1OVDCON = 0xFF00;
	
	//On veut DeadTime = ns avec Tcy = 34ns donc DTAPS = 0 et DTA = 12.
	P1DTCON1bits.DTA = 12; //Dead Time Value : DTA = DeadTime / (Prescaler * Tcy)
	P1DTCON1bits.DTAPS = 0; //Clock input selection for Dead Time Generation : 0 <=> Tcy, 1 <=> 2Tcy, 2<=> 4Tcy, 3<=>8Tcy
		       
	// Initialisation du timer
	P1TCONbits.PTEN = 1;

	// Initialisation du PWM à 50 %
	ChangerPWM(500.0,1);
	ChangerPWM(500.0,2);
	
	TRISBbits.TRISB15 = 0;
	TRISBbits.TRISB13 = 0;
}//

// ----------------------------------------------------------------
// Fonction : 		ChangerPWM(rapport, pwm)
// But 	    : 		Change le rapport cyclique par 1000 du PWM
// ---------------------------------------------------------------- 

void ChangerPWM (float rapport, char pwm)
{
	float tampon;

	if(rapport > 975)
		rapport = 975;
		
	if(rapport < 25)
		rapport = 25;
	
		tampon = (2.0 * 1473.0) * fabs(rapport - 500.0)/500.0;
		
		switch (pwm) 
		{
			case 1: 
				P1DC1 = tampon;
				if(rapport > 500.0)
				{
					LATBbits.LATB15 = 1;
				}
				else
				{
					LATBbits.LATB15 = 0;
				}	
				break;
				
			case 2: 
				P1DC2 = tampon;
				if(rapport > 500.0)
				{
					LATBbits.LATB13 = 1;
				}	
				else
				{
					LATBbits.LATB13 = 0;
				}	
				break;
				
			default: 
				break;
		}
}//

