#include "fct_asserv.h"
#include "odo.h"
#include "donnees.h"
#include <math.h>

//Fonctions mathématiques utiles
float absf(float val)
{
	if (val >= 0)
		return val;
	return -val;
}

float Mod2Pi(float Angle)
{
    float resultat = Angle;
    
    while(resultat > PI || resultat <= -PI)
    {
    	if (resultat > PI)
    	{
    		resultat -= 2*PI;
    	} else if (resultat <= -PI)
    	{
    		resultat += 2*PI;
    	}
    }
    
    return resultat;
}

int EstDansInterval(float Val, float Abs)
{
	if (-Abs< Val && Val < Abs)
	{
		return 1;
	} else {
		return 0;
	}
}

float Seuiller(float Val, float Abs) //Fait en sorte que Val soit compris dans l'interval [-Abs ; Abs]
{
	if (Val > Abs)
	{
		return Abs;
	} else if (Val < -Abs)
	{
		return -Abs;
	} else {
		return Val;
	}
}//


void CalculerEtat(ETAT *Etat, MECANIQUE *Mecanique, CARTE *Carte)
{
	//Acquisition des mesures avec odo.h
	float DistanceRoueD_Temp = ODODistRoue0_N();
	float DistanceRoueG_Temp = QEIDistanceRoue1();
	
	float NouvelAngleBrut = (DistanceRoueD_Temp - DistanceRoueG_Temp) / Mecanique->EntreAxe;
	
	//Calcul des vitesses
	Etat->VitesseRoueD = (DistanceRoueD_Temp - Etat->DistanceRoueD) / Carte->Te;
	Etat->VitesseRoueG = (DistanceRoueG_Temp - Etat->DistanceRoueG) / Carte->Te;
	
	Etat->VitesseAngulaire = (NouvelAngleBrut - Etat->AngleBrut) / Carte->Te;
	Etat->VitesseLongitudinale = (Etat->VitesseRoueD + Etat->VitesseRoueG) / 2.0;
	
	//Calcul des coordonnés
	Etat->X += Etat->VitesseLongitudinale * Carte->Te * cos(Etat->AngleReel);
	Etat->Y += Etat->VitesseLongitudinale * Carte->Te * sin(Etat->AngleReel);
	
	//Calcul des données polaires
	Etat->AngleBrut = NouvelAngleBrut;
	Etat->AngleReel = Mod2Pi(Etat->AngleBrut);
	Etat->DistanceRoueD = DistanceRoueD_Temp;
	Etat->DistanceRoueG = DistanceRoueG_Temp;
	Etat->DistanceTotale = (Etat->DistanceRoueD + Etat->DistanceRoueG) / 2.0;
}//

//Fonctions utiles à l'applications des consignes
void CalculCorrecteur(char Type, ETAT *Etat, ASSERV *Param, CORRECTEUR *Pid, CARTE *Carte)
{
	//Calcul de l'erreur entre l'état et la consigne	
	switch(Type)
	{
		case 'L': //Paramètre Longitudinal de position
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->Consigne - Etat->DistanceTotale; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'A': //Paramètre Angulaire de position
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->Consigne - Etat->AngleBrut; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'l': //Paramètre Longitudinal de vitesse
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->VitesseConsigne - Etat->VitesseLongitudinale; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'a': //Paramètre Angulaire de vitesse
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->VitesseConsigne - Etat->VitesseAngulaire; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
			
		case 'D': //Paramètre RoueD de position
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->Consigne - Etat->DistanceRoueD; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'G': //Paramètre RoueG de position
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->Consigne - Etat->DistanceRoueG; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'd': //Paramètre RoueD de vitesse
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->VitesseConsigne - Etat->VitesseRoueD; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		case 'g': //Paramètre RoueG de vitesse
			Pid->ErreurPrecedente = Pid->Erreur;
			Pid->Erreur = Param->VitesseConsigne - Etat->VitesseRoueG; 
			Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->Te;
			Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->Te; //Methode des trapèzes
			break;
		default :
			return;
	}
	
	//Calcul du terme proportionnel
	Pid->CommandeProportionelle = Pid->Kp * Pid->Erreur;
	
	//Calcul du terme intégral
	Pid->CommandeIntegrale = Pid->Ki * Pid->IntegraleErreur;
	
	//Calcul du terme dérivé
	Pid->CommandeDerive = Pid->Kd * Pid->DeriveErreur;
	
	//Génération de la commande moteur relative à ce paramètre d'asservissement
	Pid->Commande = Pid->CommandeProportionelle + Pid->CommandeDerive + Pid->CommandeIntegrale;
	
}

void ActualiserMouvement(MOUVEMENT *Mouvement, ETAT *Etat, ASSERV *Longitudinal, ASSERV *Angulaire, ASSERV *RoueD, ASSERV *RoueG, MECANIQUE *Mecanique)
{
	switch(Mouvement->Type)
	{
		case 0: //Stop
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut;
			break;
			
		case 1: //Translation de DistanceDemande
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
			Angulaire->Consigne = Etat->AngleBrut;
			break;
			
		case 2: //Rotation relative de AngleDemande
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut + Mouvement->AngleDemande;
			break;
		
		case 3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;
			if (Mouvement->EnAvant)
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
			else
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
			break;
			
		case 4: //Pointe vers Xc,Yc (Avant / Arrière)
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;
			Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
			if (Mouvement->EnAvant)
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
			else
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
			break;
			
		case 5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
			//Etape 0 :
				//Rotation
			//Etape 1:
				//Translation
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			switch(Mouvement->Etape)
			{
				case 0: //Rotation
					Longitudinal->Consigne = Etat->DistanceTotale;
					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
					if (Mouvement->EnAvant)
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
					else
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
					break;
				case 1: //Translation
					Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
					if (Mouvement->EnAvant)
						Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
					else
						Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;
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
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			switch(Mouvement->Etape)
			{
				case 0: //Rotation
					Longitudinal->Consigne = Etat->DistanceTotale;
					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
					if (Mouvement->EnAvant)
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
					else
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
					break;
				case 1: //Translation
					Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
					if (Mouvement->EnAvant)
						Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
					else
						Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;
						
					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
					if (Mouvement->EnAvant)
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
					else
						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
					break;
				default :
					break;
			}	
			break;
			
		case 7: //VaZy Xc, Yc (Avant/Arriere)
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
		
			Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
			if (Mouvement->EnAvant)
				Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
			else
				Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;
				
			Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
			if (Mouvement->EnAvant)
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
			else
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);	
			break;
			
		case 8: //Translation à vitesse constante avec angle fixe
			Mouvement->ConfigAsserv = (AsservVitLong | AsservPosAng | AsservVitAng);
		
			if (Mouvement->EnAvant)
				Longitudinal->VitesseConsigne = Longitudinal->VitesseMax;
			else
				Longitudinal->VitesseConsigne = -Longitudinal->VitesseMax;
			break;
			
		case 9: //Rotation à vitesse constante sur place
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservVitAng);
		
			if (Mouvement->EnAvant)
				Angulaire->VitesseConsigne = Angulaire->VitesseMax;
			else
				Angulaire->VitesseConsigne = -Angulaire->VitesseMax;
			break;
			
		case 10: //Passer par le point Xc, Yc à vitesse constante
			//Calcul de la distance de consigne pour aller au point Xc, Yc (non utilisé pour l'asserv, uniquement pour dire au PC quand on est arrivé)
			Mouvement->ConfigAsserv = (AsservVitLong | AsservPosAng | AsservVitAng);
			
			Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
			if (Mouvement->EnAvant)
				Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
			else
				Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;
				
			//Deplacement à vitesse longitudinale constante
			if (Mouvement->EnAvant)
				Longitudinal->VitesseConsigne = Longitudinal->VitesseMax;
			else
				Longitudinal->VitesseConsigne = -Longitudinal->VitesseMax;
		
			//Reglage permanent de l'angle voulu
			Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
			if (Mouvement->EnAvant)
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
			else
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);	
			break;
			
			
		case 11: //Pivot D
			Mouvement->ConfigAsserv = (AsservPosD | AsservVitD | AsservPosG | AsservVitG);
			
			RoueD->Consigne = Etat->DistanceRoueD;
			
			if (Mouvement->EnAvant)
				RoueG->Consigne = Etat->DistanceRoueG - Mouvement->AngleDemande * Mecanique->EntreAxe;
			else
				RoueG->Consigne = Etat->DistanceRoueG + Mouvement->AngleDemande * Mecanique->EntreAxe;
			break;
			
		case 12: //Pivot G
			Mouvement->ConfigAsserv = (AsservPosD | AsservVitD | AsservPosG | AsservVitG);
			
			RoueG->Consigne = Etat->DistanceRoueG;
			
			if (Mouvement->EnAvant)
				RoueD->Consigne = Etat->DistanceRoueD + Mouvement->AngleDemande * Mecanique->EntreAxe;
			else
				RoueD->Consigne = Etat->DistanceRoueD - Mouvement->AngleDemande * Mecanique->EntreAxe;
			break;
			
		case 13: //Position D&G
			Mouvement->ConfigAsserv = (AsservPosD | AsservVitD | AsservPosG | AsservVitG);
			RoueD->Consigne = Etat->DistanceRoueD + Mouvement->DistanceDroiteDemande;
			RoueG->Consigne = Etat->DistanceRoueG + Mouvement->DistanceGaucheDemande;
			break;
			
		case 14: //VitesseD&G
			Mouvement->ConfigAsserv = (AsservVitD | AsservVitG);
		
			if (Mouvement->EnAvant)
				RoueD->VitesseConsigne = RoueD->VitesseMax;
			else
				RoueD->VitesseConsigne = -RoueD->VitesseMax;
				
			if (Mouvement->EnAvant)
				RoueG->VitesseConsigne = RoueG->VitesseMax;
			else
				RoueG->VitesseConsigne = -RoueG->VitesseMax;
			break;
			
		case 15:
			Mouvement->ConfigAsserv = (AsservVitLong | AsservVitAng);
			Longitudinal->VitesseConsigne = 0.0;
			Angulaire->VitesseConsigne = 0.0;
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut;
			break;
			
		case 16: //Contrôle de trajectoire par le PC
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
			if (Mouvement->EnAvant)
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
			else
				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
			break;
			
		case 17: //Commande vitesse long & ang
			Mouvement->ConfigAsserv = (AsservVitLong | AsservVitAng);
			
			//Toujours vers l'avant => corriger le bug
			Longitudinal->VitesseConsigne = Longitudinal->VitesseMax;
				
			if (Mouvement->EnAvant)
				Angulaire->VitesseConsigne = Angulaire->VitesseMax;
			else
				Angulaire->VitesseConsigne = -Angulaire->VitesseMax;
			break;
			
		default : //Stop
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut;
			break;
	}
}


void SeuillageVitesse(ASSERV *Param, CARTE *Carte)
{
	Param->VitesseConsigne = Seuiller(Param->VitesseConsigne, fabs(Param->VitesseMax));
	
	if (Param->VitesseConsigne - Param->VitesseConsignePrecedente > Param->Acceleration * Carte->Te)
		Param->VitesseConsigne = Param->VitesseConsignePrecedente + Param->Acceleration * Carte->Te;
	else if (Param->VitesseConsigne - Param->VitesseConsignePrecedente < -Param->Acceleration * Carte->Te)
		Param->VitesseConsigne = Param->VitesseConsignePrecedente - Param->Acceleration * Carte->Te;
}

void AsservPositionLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidPositionLongitudinale, CARTE *Carte)
{
	CalculCorrecteur('L', Etat, Longitudinal, PidPositionLongitudinale, Carte);
	Longitudinal->VitesseConsignePrecedente = Longitudinal->VitesseConsigne;
	Longitudinal->VitesseConsigne = PidPositionLongitudinale->Commande;
	SeuillageVitesse(Longitudinal, Carte);	
}

void AsservPositionAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidPositionAngulaire, CARTE *Carte)
{
	CalculCorrecteur('A', Etat, Angulaire, PidPositionAngulaire, Carte);
	Angulaire->VitesseConsignePrecedente = Angulaire->VitesseConsigne;
	Angulaire->VitesseConsigne = PidPositionAngulaire->Commande;
	SeuillageVitesse(Angulaire, Carte);
}

void AsservVitesseLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidVitesseLongitudinale, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur('l', Etat, Longitudinal, PidVitesseLongitudinale, Carte);
	PidVitesseLongitudinale->Commande = Seuiller(PidVitesseLongitudinale->Commande, Mecanique->PuissanceMaxLongitudinal);
}

void AsservVitesseAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidVitesseAngulaire, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur('a', Etat, Angulaire, PidVitesseAngulaire, Carte);
	PidVitesseAngulaire->Commande = Seuiller(PidVitesseAngulaire->Commande, Mecanique->PuissanceMaxAngulaire);	
}

void ApplicationPuissanceMoteurPolaire(CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, MECANIQUE *Mecanique)
{
	Mecanique->PuissanceMoteurD = Seuiller(PidVitesseLongitudinale->Commande + PidVitesseAngulaire->Commande, 500);
	Mecanique->PuissanceMoteurG = Seuiller(PidVitesseLongitudinale->Commande - PidVitesseAngulaire->Commande, 500);
}







void AsservPositionD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidPositionD, CARTE *Carte)
{
	CalculCorrecteur('D', Etat, RoueD, PidPositionD, Carte);
	RoueD->VitesseConsignePrecedente = RoueD->VitesseConsigne;
	RoueD->VitesseConsigne = PidPositionD->Commande;
	SeuillageVitesse(RoueD, Carte);	
}

void AsservPositionG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidPositionG, CARTE *Carte)
{
	CalculCorrecteur('G', Etat, RoueG, PidPositionG, Carte);
	RoueG->VitesseConsignePrecedente = RoueG->VitesseConsigne;
	RoueG->VitesseConsigne = PidPositionG->Commande;
	SeuillageVitesse(RoueG, Carte);	
}

void AsservVitesseD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidVitesseD, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur('d', Etat, RoueD, PidVitesseD, Carte);
	PidVitesseD->Commande = Seuiller(PidVitesseD->Commande, Mecanique->PuissanceMaxD);
}

void AsservVitesseG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidVitesseG, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur('g', Etat, RoueG, PidVitesseG, Carte);
	PidVitesseG->Commande = Seuiller(PidVitesseG->Commande, Mecanique->PuissanceMaxG);
}

void ApplicationPuissanceMoteurDG(CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MECANIQUE *Mecanique)
{
	Mecanique->PuissanceMoteurD = PidVitesseD->Commande;
	Mecanique->PuissanceMoteurG = PidVitesseG->Commande;
}
