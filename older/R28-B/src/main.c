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

#include "p30fxxxx.h"
#include <math.h>

#include "fct_asserv.h"
#include "timer.h"
#include "donnees.h"
#include "pwm.h"
#include "can.h"
#include "inputs.h"
#include "odo.h"
#include "communication.h"

// XT Oscillator
	_FOSC(FRC_PLL16)
	_FWDT(WDT_OFF & WDTPSA_512)
	_FBORPOR(PBOR_ON & BORV_27)
	_FGS(CODE_PROT_OFF)
	
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

//Fonctions utilisables pour du debug
void EnregistrementDebug(DEBUG *Debug, ASSERV *Longitudinal, ASSERV *Angulaire, CORRECTEUR *PidPositionLongitudinale, CORRECTEUR *PidPositionAngulaire, CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, ETAT *Etat, MECANIQUE *Mecanique)
{
	//Enregistrement des mesures
	Debug->Record1[Debug->i] = Etat->DistanceTotale;
	Debug->Record2[Debug->i] = Longitudinal->VitesseConsigne;
	//Temps d'execution du mouvement : Tmvt = VitesseMax / Acceleration + Desire / VitesseMax;
	
	if (Debug->i < 99)
	{
		Debug->i++;
	}
}//

//Fonction s'executant tout les Te, à la base de l'asservissement
void ExecutionToutLesTe(void)
{	
	//Acquisition des mesures
	CalculerEtat(&Etat, &Mecanique, &Carte);	
	
	//Recuperation des consignes en fonction du mouvement actuel
	switch(MouvementActuel.Type)
	{
		case 0: //Stop
			if ((EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale)) && (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire)) && Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
			{
				MouvementActuel.Termine = 1;
                if(!MouvementActuel.TermineEnvoye)
                    SendTermine();
                MouvementActuel.TermineEnvoye = 1;
			}
			break;
			
		case 1: //Translation de DistanceDemande
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale) && Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
			{
				MouvementActuel.Termine = 1;
                if(!MouvementActuel.TermineEnvoye)
                    SendTermine();
                MouvementActuel.TermineEnvoye = 1;
			}
			break;
			
		case 2: //Rotation relative de AngleDemande
		case 3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
		case 4: //Pointe vers Xc,Yc (Avant / Arrière)
			if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire) && Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
			{
				MouvementActuel.Termine = 1;
                if(!MouvementActuel.TermineEnvoye)
                    SendTermine();
                MouvementActuel.TermineEnvoye = 1;
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
					if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire) && Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
					{
						MouvementActuel.Etape++;
						ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
						Debug.i = 0;
					}
					break;
				case 1: //Translation
					if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale) && Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
					{
						MouvementActuel.Termine = 1;
		      			if(!MouvementActuel.TermineEnvoye)
		              		SendTermine();
		             	MouvementActuel.TermineEnvoye = 1;
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
					if (EstDansInterval(Angulaire.Consigne - Etat.AngleBrut, MouvementActuel.PrecisionAngulaire) && Etat.VitesseAngulaire == 0.0)
					{
						MouvementActuel.Etape++;
						ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
						Debug.i = 0;
					}
					break;
				case 1: //Translation
					
					if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
					{
						if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
						{
							MouvementActuel.Termine = 1;
			      			if(!MouvementActuel.TermineEnvoye)
			              		SendTermine();
			             	MouvementActuel.TermineEnvoye = 1;
			             }
					}
					else
					{
						ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
					}
					break;
				default :
					break;
			}	
			break;
			
		case 7: //VaZy Xc, Yc (Avant/Arriere)	
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale))
			{
				if(Etat.VitesseLongitudinale == 0.0 && Etat.VitesseAngulaire == 0.0)
				{
					MouvementActuel.Termine = 1;
	      			if(!MouvementActuel.TermineEnvoye)
	              		SendTermine();
	             	MouvementActuel.TermineEnvoye = 1;
	             }
			}
			else
			{
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
			}
			break;
			
		case 8: //Translation à vitesse constante avec angle fixe
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne; 	
			break;
			
		case 9: //Rotation à vitesse constante sur place
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;
			break;
			
		case 10: //Passer par le point Xc, Yc à vitesse constante
			if (EstDansInterval(Longitudinal.Consigne - Etat.DistanceTotale, MouvementActuel.PrecisionLongitudinale) && MouvementActuel.Termine == 0)
			{
				MouvementActuel.Termine = 1;
      			if(!MouvementActuel.TermineEnvoye)
              		SendTermine();
             	MouvementActuel.TermineEnvoye = 1;
				
				//On active une translation sur 2 fois la distance de freinage histoire de laisser le temps au PC d'envoyer le point suivant, sinon le robot s'arrete
				MouvementActuel.Type = 1;
		
				if (MouvementActuel.EnAvant)
					MouvementActuel.DistanceDemande = (Longitudinal.VitesseMax * Longitudinal.VitesseMax / (2.0 * Longitudinal.Acceleration));
				else
					MouvementActuel.DistanceDemande = -(Longitudinal.VitesseMax * Longitudinal.VitesseMax / (2.0 * Longitudinal.Acceleration));
					
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			}
			else if(MouvementActuel.Termine == 0)
			{
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique); //On n'actualise la distance et l'angle que si on n'a pas atteint la précision
				SeuillageVitesse(&Longitudinal, &Carte);
				//Longitudinal.VitesseConsigne = Longitudinal.VitesseConsigne * (1.0 - absf(Mod2Pi(Etat.AngleBrut - Angulaire.Consigne)) / PI) ;
				Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;
			}
			break;
			
		case 11: //Pivot D
			if (EstDansInterval(RoueG.Consigne - Etat.DistanceRoueG, MouvementActuel.PrecisionLongitudinale))
			{
				MouvementActuel.Termine = 1;
      			if(!MouvementActuel.TermineEnvoye)
              		SendTermine();
             	MouvementActuel.TermineEnvoye = 1;
			}
			break;
			
		case 12: //Pivot G
			if (EstDansInterval(RoueD.Consigne - Etat.DistanceRoueD, MouvementActuel.PrecisionLongitudinale))
			{
				MouvementActuel.Termine = 1;
      			if(!MouvementActuel.TermineEnvoye)
              		SendTermine();
             	MouvementActuel.TermineEnvoye = 1;
			}
			break;
			
		case 13: //Position D&G
			if ((EstDansInterval(RoueD.Consigne - Etat.DistanceRoueD, MouvementActuel.PrecisionLongitudinale)) && (EstDansInterval(RoueG.Consigne - Etat.DistanceRoueG, MouvementActuel.PrecisionLongitudinale)))
			{
				MouvementActuel.Termine = 1;
      			if(!MouvementActuel.TermineEnvoye)
              		SendTermine();
             	MouvementActuel.TermineEnvoye = 1;
			}
			break;
			
		case 14: //VitesseD&G
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&RoueD, &Carte);
			RoueD.VitesseConsignePrecedente = RoueD.VitesseConsigne;
			
			SeuillageVitesse(&RoueG, &Carte);
			RoueG.VitesseConsignePrecedente = RoueG.VitesseConsigne;
			
			if(EstDansInterval(RoueD.VitesseMax - Etat.VitesseRoueD, MouvementActuel.PrecisionLongitudinale) && EstDansInterval(RoueG.VitesseMax - Etat.VitesseRoueG, MouvementActuel.PrecisionLongitudinale))
			{
				MouvementActuel.Termine = 1;
      			if(!MouvementActuel.TermineEnvoye)
              		SendTermine();
             	MouvementActuel.TermineEnvoye = 1;
		    } 
			break;
			
		case 15:
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne; 	

			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;
			
			if(Etat.VitesseLongitudinale == 0.0)
			{
				Longitudinal.Consigne = Etat.DistanceTotale;
				Angulaire.Consigne = Etat.AngleBrut;
			}
			break;
			
		case 16: //Contrôle trajectoire par le PC
			break;
			
		case 17: //Commande simultané des vitesses longitudinales et angulaires (pour le suivi webcam)
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			
			SeuillageVitesse(&Longitudinal, &Carte);
			Longitudinal.VitesseConsignePrecedente = Longitudinal.VitesseConsigne;
			
			SeuillageVitesse(&Angulaire, &Carte);
			Angulaire.VitesseConsignePrecedente = Angulaire.VitesseConsigne;
			break;

		default : //Arret d'urgence
			MouvementActuel.Type = 0;
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			break;
	}
	
	/*--------------------------------------------------------------------------------------*/
										/* POLAIRE */
	//Calcul correcteur de position longitudinale
	if (MouvementActuel.ConfigAsserv & AsservPosLong)
		AsservPositionLongitudinale(&Etat, &Longitudinal, &PidPositionLongitudinale, &Carte);
	
	if(MouvementActuel.Termine) //Les réglages PID varient selon si le robot est à l'arret ou pas
	{
		PidVitesseLongitudinale.Kp = 1200;
		PidVitesseAngulaire.Kp = 200;
	}
	else
	{
		PidVitesseLongitudinale.Kp = 2000;
		PidVitesseAngulaire.Kp = 300;
	}
	
	//Calcul correcteur de position angulaire
	if (MouvementActuel.ConfigAsserv & AsservPosAng)
		AsservPositionAngulaire(&Etat, &Angulaire, &PidPositionAngulaire, &Carte);
	
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
		
	//Log pour debug
	//EnregistrementDebug(&Debug, &Longitudinal, &Angulaire, &PidPositionLongitudinale, &PidPositionAngulaire, &PidVitesseLongitudinale, &PidVitesseAngulaire, &Etat, &Mecanique);
}//

//Propramme principal (initialisation)	
int main (void)
{		
	//Variables Debug
	Debug.i = 0;
	//Fin variables debug

	InitPWM();
	VARInitialize(&Carte, &Mecanique, &Longitudinal, &Angulaire, &Etat, &PidPositionLongitudinale, &PidPositionAngulaire, &PidVitesseLongitudinale, &PidVitesseAngulaire, &RoueD, &RoueG, &PidPositionD, &PidPositionG, &PidVitesseD, &PidVitesseG, &MouvementActuel); //Initialisation de toutes les variables du programme d'asservissement
	
	T1Initialize(Carte.Te, Carte.Fcy); //Periode du timer : Te
	ODOInitialize();
	ODOStart();
	CANInitialize(ID_CARTE);

	TRISEbits.TRISE4 = 0;
	TRISEbits.TRISE5 = 0;
	
	LATEbits.LATE4 = 0;
	LATEbits.LATE5 = 1;

	SendBloque();

	MouvementActuel.DistanceDemande = 0.0;
	MouvementActuel.AngleDemande = 0.0;
	MouvementActuel.Type = 0;
	MouvementActuel.Termine = 0;
	ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);

	while(1)
	{
		Nop();
	}
}//
