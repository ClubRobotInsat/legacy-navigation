/**********************************************************************
*
* FileName:        main.c
*
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Lukas				Eté 2007   En cours de développement
**********************************************************************/

#include "p33fxxxx.h"
#include "math.h"

#include "..\..\..\..\Librairies Périphériques\TIMER dsPIC33F\Librairie\timer.h"
#include "..\..\..\..\Librairies Périphériques\ECAN dsPIC33F\Librairie\ecan.h"
#include "..\..\..\..\Librairies Périphériques\QEI dsPIC33F\Librairie\qei.h"

#include "communication.h"
#include "variables.h"
#include "pwm.h"
#include "fct_asserv.h"

// Internal FRC Oscillator
_FOSCSEL(FNOSC_FRCPLL);			// FRC Oscillator with PLL
_FOSC(OSCIOFNC_ON);				// OSCIOFNC_ON          OSC2 Pin function:Digital I/O
_FWDT(FWDTEN_OFF & WINDIS_OFF);	// Watchdog Timer Enabled/disabled by user software

#define TEMPS_BLOCAGE 15 //15 <=> 300ms

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Normes : Toutes les mesures sont données en sec, m, m/s, m/s², rad, rad/s, rad/s², Hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//Structures contenant toutes les variables, classées par utilité
	DEBUG Debug;

	CARTE Carte;

	MECANIQUE Mecanique;

	ETAT Etat;

	ASSERV Longitudinal;
	ASSERV Angulaire;
	ASSERV RoueD;
	ASSERV RoueG;

	CORRECTEUR PidPositionLongitudinale;
	CORRECTEUR PidPositionAngulaire;
	CORRECTEUR PidVitesseLongitudinale;
	CORRECTEUR PidVitesseAngulaire;

	CORRECTEUR PidPositionD;
	CORRECTEUR PidPositionG;
	CORRECTEUR PidVitesseD;
	CORRECTEUR PidVitesseG;

	MOUVEMENT MouvementActuel;

//Fin de la déclaration des structures

// Variables globales pour l'envoi auto
	int compteur = 0;

//Fonction s'executant tout les Te, à la base de l'asservissement
void T1PeriodMatch()
{
	//Acquisition des mesures
	CalculerEtat(&Etat, &Mecanique, &Carte);

	//Recuperation des consignes en fonction du mouvement actuel
	switch(MouvementActuel.Type)
	{
		case 0: //Stop
			if ((EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
					&& (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire)))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 1: //Translation de DistanceDemande
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 2: //Rotation relative de AngleDemande
		case 3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
		case 4: //Pointe vers Xc,Yc (Avant / Arrière)
			if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
			//Etape 0 :
				//Rotation
			//Etape 1:
				//Translation
			switch(MouvementActuel.Etape)
			{
				case 0: //Rotation
					if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire)
							&& Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
					{
						MouvementActuel.Etape++;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					break;
				case 1: //Translation
					if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
					{
						if(MouvementActuel.TermineEnvoye == 0)
							SendAtteint();
						MouvementActuel.TermineEnvoye = 1;
						if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
						{
							MouvementActuel.Termine = 1;
							if(MouvementActuel.TermineEnvoye == 1)
								SendTermine();
							MouvementActuel.TermineEnvoye = 2;
						}
					}
					break;
				default :
					break;
			}
			break;

		case 6: //VaZy Xc, Yc (Decompose, Avant/Arriere)
			//Etape 0 :
				//Rotation
			//Etape 1:
				//Translation avec correction permanente de l'angle
			switch(MouvementActuel.Etape)
			{
				case 0: //Rotation
					if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire)
							&& Etat.VitesseAngulaire == 0.0)
					{
						MouvementActuel.Etape++;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					break;
				case 1: //Translation

					if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
					{
						if(MouvementActuel.TermineEnvoye == 0)
							SendAtteint();
						MouvementActuel.TermineEnvoye = 1;
						if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
						{
							MouvementActuel.Termine = 1;
							if(MouvementActuel.TermineEnvoye == 1)
								SendTermine();
							MouvementActuel.TermineEnvoye = 2;
			             }
					}
					else
					{
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
					}
					break;
				default :
					break;
			}
			break;

		case 7: //VaZy Xc, Yc (Avant/Arriere)
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
	             }
			}
			else
			{
				InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
			}
			break;

		case 8: //Translation à vitesse constante avec angle fixe
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;
			break;

		case 9: //Rotation à vitesse constante sur place
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;
			break;

		case 10: //Passer par le point Xc, Yc à vitesse constante
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale) && MouvementActuel.Termine == 0)
			{
				MouvementActuel.Termine = 1;
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;

				//On active une translation sur 2 fois la distance de freinage histoire de laisser le temps au PC d'envoyer le point suivant, sinon le robot s'arrete
				MouvementActuel.Type = 1;

				if (MouvementActuel.EnAvant)
					MouvementActuel.DistanceDemande = (Longitudinal.VitesseMax * Longitudinal.VitesseMax / (2.0 * Longitudinal.Acceleration));
				else
					MouvementActuel.DistanceDemande = -(Longitudinal.VitesseMax * Longitudinal.VitesseMax / (2.0 * Longitudinal.Acceleration));

				InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			}
			else if(MouvementActuel.Termine == 0)
			{
				InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
				SeuillageVitesse(&Longitudinal, &Carte);
				//Longitudinal.VitesseConsigne = Longitudinal.VitesseConsigne * (1.0 - absf(Mod2Pi(Etat.AngleBrut - Angulaire.Consigne)) / PI) ;
				Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;
			}
			break;

		case 11: //Pivot D
			if (EstDansInterval(RoueG.Consigne - Etat.DistanceRoueG, MouvementActuel.PrecisionLongitudinale))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseRoueD == 0.0 && Etat.VitesseRoueG == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 12: //Pivot G
			if (EstDansInterval(RoueD.Consigne - Etat.DistanceRoueD, MouvementActuel.PrecisionLongitudinale))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseRoueD == 0.0 && Etat.VitesseRoueG == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 13: //Position D&G
			if ((EstDansInterval(RoueD.Consigne - Etat.DistanceRoueD, MouvementActuel.PrecisionLongitudinale))
					&& (EstDansInterval(RoueG.Consigne - Etat.DistanceRoueG, MouvementActuel.PrecisionLongitudinale)))
			{
				if(MouvementActuel.TermineEnvoye == 0)
					SendAtteint();
				MouvementActuel.TermineEnvoye = 1;
				if(Etat.VitesseRoueD == 0.0 && Etat.VitesseRoueG == 0.0)
				{
					MouvementActuel.Termine = 1;
					if(MouvementActuel.TermineEnvoye == 1)
						SendTermine();
					MouvementActuel.TermineEnvoye = 2;
				}
			}
			break;

		case 14: //VitesseD&G
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);

			SeuillageVitesse(&RoueD, &Carte);
			RoueD.VitesseConsignePrecedente = RoueD.VitesseConsigne;

			SeuillageVitesse(&RoueG, &Carte);
			RoueG.VitesseConsignePrecedente = RoueG.VitesseConsigne;
			break;

		case 15:
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);

			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;

			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;

			if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
			{
				Longitudinal.Consigne = Etat.DistanceTotale;
				Angulaire.Consigne = Etat.AngleBrut;
			}
			break;

		case 17: //Commande simultané des vitesses longitudinales et angulaires (pour le suivi webcam)
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);

			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;

			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;
			break;

		default : //Arret d'urgence
			MouvementActuel.Type = 0;
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			break;
	}
	
	/*--------------------------------------------------------------------------------------*/
										/* POLAIRE */
	//Calcul correcteur de position longitudinale
	if (MouvementActuel.ConfigAsserv & AsservPosLong)
		AsservPositionLongitudinale(&Etat, &Longitudinal, &PidPositionLongitudinale, &Carte);

	//Calcul correcteur de position angulaire
	if (MouvementActuel.ConfigAsserv & AsservPosAng)
		AsservPositionAngulaire(&Etat, &Angulaire, &PidPositionAngulaire, &Carte);

	if(MouvementActuel.Termine) //Les réglages PID varient selon si le robot est à l'arret ou pas
	{
		PidVitesseLongitudinale.Kp = PidVitesseLongitudinale.KpArret;
		PidVitesseAngulaire.Kp = PidVitesseAngulaire.KpArret;
	}
	else
	{
		PidVitesseLongitudinale.Kp = PidVitesseLongitudinale.KpRoule;
		PidVitesseAngulaire.Kp = PidVitesseAngulaire.KpRoule;
	}

	//Calcul correcteur de vitesse longitudinale
	if (MouvementActuel.ConfigAsserv & AsservVitLong)
		AsservVitesseLongitudinale(&Etat, &Longitudinal, &PidVitesseLongitudinale, &Carte, &Mecanique);

	//Calcul correcteur de vitesse angulaire
	if (MouvementActuel.ConfigAsserv & AsservVitAng)
		AsservVitesseAngulaire(&Etat, &Angulaire, &PidVitesseAngulaire, &Carte, &Mecanique);

	if(!Longitudinal.Est_Active)
		PidVitesseLongitudinale.Commande = 0;

	if(!Angulaire.Est_Active)
		PidVitesseAngulaire.Commande = 0;

	//Application commande puissance moteur polaire
	if ((MouvementActuel.ConfigAsserv & AsservVitLong) && (MouvementActuel.ConfigAsserv & AsservVitAng))
		ApplicationPuissanceMoteurPolaire(&PidVitesseLongitudinale, &PidVitesseAngulaire, &Mecanique);

	/*--------------------------------------------------------------------------------------*/
										/* D & G */
	//Calcul correcteur de position D
	if (MouvementActuel.ConfigAsserv & AsservPosD)
		AsservPositionD(&Etat, &RoueD, &PidPositionD, &Carte);

	//Calcul correcteur de position G
	if (MouvementActuel.ConfigAsserv & AsservPosG)
		AsservPositionG(&Etat, &RoueG, &PidPositionG, &Carte);

	if(MouvementActuel.Termine) //Les réglages PID varient selon si le robot est à l'arret ou pas
	{
		PidVitesseD.Kp = PidVitesseD.KpArret;
		PidVitesseG.Kp = PidVitesseG.KpArret;
	}
	else
	{
		PidVitesseD.Kp = PidVitesseD.KpRoule;
		PidVitesseG.Kp = PidVitesseG.KpRoule;
	}

	//Calcul correcteur de vitesse D
	if (MouvementActuel.ConfigAsserv & AsservVitD)
		AsservVitesseD(&Etat, &RoueD, &PidVitesseD, &Carte, &Mecanique);

	//Calcul correcteur de vitesse G
	if (MouvementActuel.ConfigAsserv & AsservVitG)
		AsservVitesseG(&Etat, &RoueG, &PidVitesseG, &Carte, &Mecanique);

	if(!RoueD.Est_Active)
		PidVitesseD.Commande = 0;

	if(!RoueG.Est_Active)
		PidVitesseG.Commande = 0;

	//Application commande puissance moteur D&G
	if ((MouvementActuel.ConfigAsserv & AsservVitD) && (MouvementActuel.ConfigAsserv & AsservVitG))
		ApplicationPuissanceMoteurDG(&PidVitesseD, &PidVitesseG, &Mecanique);

	//PWM Sortie 1 => Roue Droite
	//PWM Sortie 2 => Roue Gauche
	ChangerPWM(500 - Mecanique.PuissanceMoteurD, 1);
	ChangerPWM(500 + Mecanique.PuissanceMoteurG, 2);

    if((!MouvementActuel.Termine) && (fabs(Longitudinal.VitesseConsigne) > MouvementActuel.PrecisionLongitudinale) && (fabs(Etat.VitesseLongitudinale) == 0.0))
        MouvementActuel.TempsBlocage++;
    else
        MouvementActuel.TempsBlocage = 0;

    if(MouvementActuel.TempsBlocage > TEMPS_BLOCAGE)
    {
        SendBloque();
        MouvementActuel.TempsBlocage = 0;
    }

	// Envoi automatique de la postion toutes les 100ms
	if(Etat.envoi_auto == 1)
	{
		compteur++;
		if(compteur == 4)
			{
				SendPosition();
				compteur=0;
			}
	}
}//

//Propramme principal (initialisation)
int main (void)
{
	// Configure Oscillator to operate the device at 40Mhz ie Fosc @ 80MHz
	// Fosc= Fin*M/(N1*N2)
	// Finter1 = Fcristal / PLLPRE = 7.37 MHz / 2 = 3,685 Mhz (ok entre 0.8 et 8Mhz) 
	// Finter2 = Finter1 * PLLFBD =  3,685 MHz * 43 = 158,455 Mhz (ok entre 100 et 200Mhz) 
	// Fosc = Finter2 / PLLPOST =  158.455 /  2 = 79.2275 Mhz
	PLLFBD=30;			// M=43 <=> PLLFBD = 41 <=> 40MHz | M=32 <=> PLLFBD = 30 <=> 30MHz
	CLKDIVbits.PLLPOST=0;		// N1=2
	CLKDIVbits.PLLPRE=0;		// N2=2
	OSCTUN=0;					// Tune FRC oscillator, if FRC is used
    // Wait for PLL to lock
	while(OSCCONbits.LOCK!=1);  

	// On met les pins des LEDs en sortie
	TRISBbits.TRISB5 = 0;
	TRISBbits.TRISB6 = 0;

	InitPWM();	
	VARInitialize(&Carte, &Mecanique, &Longitudinal, &Angulaire, &Etat, &PidPositionLongitudinale, &PidPositionAngulaire, &PidVitesseLongitudinale, &PidVitesseAngulaire, &RoueD, &RoueG, &PidPositionD, &PidPositionG, &PidVitesseD, &PidVitesseG, &MouvementActuel); //Initialisation de toutes les variables du programme d'asservissement

	T1Initialize(Carte.Te, Carte.Fcy); //Periode du timer : Te
	QEI1Initialize(Mecanique.Nombre_Pas_D, Mecanique.DiametreRoueD);
	QEI2Initialize(Mecanique.Nombre_Pas_G, Mecanique.DiametreRoueG);
	ECANInitialize(ID_CARTE, 30);

	MouvementActuel.DistanceDemande = 0.0;
	MouvementActuel.AngleDemande = 0.0;
	MouvementActuel.Type = 0;
	MouvementActuel.Termine = 0;
	InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
	
	T1SetStatus(1); // Active la boucle d'asservissement
	
	int debug = 0;
	
	while(1)
	{
		/*
		if(debug)
		{
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			debug = 0;
		}
		*/
	}
}//

void ECANPinMapping()
{
	RPINR26bits.C1RXR = 11; //C1RX -> PIN22 -> RP11
	RPOR5bits.RP10R = 16; //C1TX -> PIN21 -> RP10
}

void QEI1PinMapping(void)
{
	// On passe les pattes en mode Numérique (et pas Analogique)
	AD1PCFGLbits.PCFG4 = 1;
	AD1PCFGLbits.PCFG5 = 1;
	
	// On définit le mapping
	RPINR14bits.QEA1R = 2;
	RPINR14bits.QEB1R = 3;
	RPINR15bits.INDX1R = 4;
}

void QEI2PinMapping(void)
{		
	// On définit le mapping
	RPINR16bits.QEA2R = 9;
	RPINR16bits.QEB2R = 8;
	RPINR17bits.INDX2R = 7;
}

void T2PeriodMatch()
{
	
}

void T3PeriodMatch()
{
	
}

void T4PeriodMatch()
{
	
}

void T5PeriodMatch()
{
	
}		
