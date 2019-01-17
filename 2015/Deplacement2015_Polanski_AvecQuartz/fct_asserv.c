#include "../../../../VieillesLibs/ECAN dsPIC33F/Librairie/ecan.h"
#include "../../../../VieillesLibs/QEI dsPIC33F/Librairie/qei.h"

#include "fct_asserv.h"
#include "variables.h"

extern ASSERV Longitudinal;
extern ASSERV Angulaire;

#include <math.h>

//Fonctions mathématiques utiles
float fabs(float Val)
{
	if (Val >= 0)
		return Val;
	return -Val;
}

// /!\ Fonction conservée pour des raisons de compatibilité avec l'ancien code.
// Malgré le nom, cette fonction ne fait pas un modulo 2PI, mais retourne l'angle
// Val dans l'intervalle [-PI, PI]
float Mod2Pi(float Val)
{
    return ToMoinsPiPi(Val);
}

// Retourne l'angle val dans l'intervalle [0, 2PI]
float ToZero2Pi(float Val)
{
	float temp = fmod(Val, 2 * PI);
	
	if(temp < 0)
		temp += 2.0 * PI;
	
	return temp;
}

// Retourne l'angle val dans l'intervalle [-PI, PI]
float ToMoinsPiPi(float Val)
{
	float temp = fmod(Val, 2 * PI);


	if(temp > PI)
		temp -= 2.0 * PI;
	if(temp <= -PI)
		temp +=  2.0 * PI;

	return temp;
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

float SeuillerDansInterval(float Val, float Borne) //Fait en sorte que Val soit compris dans l'interval [-Abs ; Abs]
{
	if (Val > Borne)
	{
		return Borne;
	} else if (Val < -Borne)
	{
		return -Borne;
	} else {
		return Val;
	}
}//


void CalculerEtat(ETAT *Etat, MECANIQUE *Mecanique, CARTE *Carte)
{
	//Acquisition des mesures avec qei.h
	//float DistanceRoueD_Temp = QEI1GetDistanceRoue();
	//float DistanceRoueG_Temp = QEI2GetDistanceRoue();
	float DistanceRoueD_Temp;
	float DistanceRoueG_Temp;
	QEIGetDistanceRoues(&DistanceRoueD_Temp, &DistanceRoueG_Temp);
	
	DistanceRoueD_Temp *= -1;
	DistanceRoueG_Temp *= -1;
	

	// Fait clignoter les LEDs quand les roues codeuses tournent
	LATBbits.LATB5 = (int)(DistanceRoueD_Temp*1000)&0x1;
	LATBbits.LATB6 = (int)(DistanceRoueG_Temp*1000)&0x1;

	float NouvelAngleBrut = (DistanceRoueG_Temp - DistanceRoueD_Temp) / Mecanique->EntreAxe;
	float NouvelAngleReel = Mod2Pi(NouvelAngleBrut);

	//Calcul des vitesses
	Etat->VitesseRoueD = (DistanceRoueD_Temp - Etat->DistanceRoueD) / Carte->TeOdometrie;
	Etat->VitesseRoueG = (DistanceRoueG_Temp - Etat->DistanceRoueG) / Carte->TeOdometrie;

	Etat->VitesseAngulaire = (NouvelAngleBrut - Etat->AngleBrut) / Carte->TeOdometrie;
	Etat->VitesseLongitudinale = (Etat->VitesseRoueD + Etat->VitesseRoueG) / 2.0;

	//Calcul des coordonnés
	if (Etat->VitesseAngulaire != 0) 
	{
		Etat->X += (Etat->VitesseLongitudinale / Etat->VitesseAngulaire) * (sin(NouvelAngleBrut) - sin(Etat->AngleBrut));
		Etat->Y -= (Etat->VitesseLongitudinale / Etat->VitesseAngulaire) * (cos(NouvelAngleBrut) - cos(Etat->AngleBrut));
	}
	else
	{
		Etat->X += Etat->VitesseLongitudinale * Carte->TeOdometrie * cos(Etat->AngleReel);
		Etat->Y += Etat->VitesseLongitudinale * Carte->TeOdometrie * sin(Etat->AngleReel);
	}
	//Etat->X += Etat->VitesseLongitudinale * Carte->TeOdometrie * cos(Etat->AngleReel);
	//Etat->Y += Etat->VitesseLongitudinale * Carte->TeOdometrie * sin(Etat->AngleReel);

	//Calcul des données polaires
	Etat->AngleBrut = NouvelAngleBrut;
	Etat->AngleReel = NouvelAngleReel;
	Etat->DistanceRoueD = DistanceRoueD_Temp;
	Etat->DistanceRoueG = DistanceRoueG_Temp;
	Etat->DistanceTotale = (Etat->DistanceRoueD + Etat->DistanceRoueG) / 2.0;
}//

//Fonctions utiles à l'applications des consignes
void CalculCorrecteur(float Mesure, float Consigne, CORRECTEUR *Pid, CARTE *Carte)
{
	//Calcul de l'erreur entre l'état et la consigne
	Pid->ErreurPrecedente = Pid->Erreur;
	Pid->Erreur = Consigne - Mesure;
	Pid->DeriveErreur = (Pid->Erreur - Pid->ErreurPrecedente) / Carte->TeAsserv;
	Pid->IntegraleErreur += (Pid->Erreur + Pid->ErreurPrecedente) / 2.0 * Carte->TeAsserv; //Methode des trapèzes


	//Calcul du terme proportionnel
	Pid->CommandeProportionelle = Pid->Kp * Pid->Erreur;

	//Calcul du terme intégral
	Pid->CommandeIntegrale = Pid->Ki * Pid->IntegraleErreur;

	//Calcul du terme dérivé
	Pid->CommandeDerive = Pid->Kd * Pid->DeriveErreur;

	//Génération de la commande moteur relative à ce paramètre d'asservissement
	Pid->Commande = Pid->CommandeProportionelle + Pid->CommandeDerive + Pid->CommandeIntegrale;

}

void InitierOuActualiserConsigneMouvement(MOUVEMENT *Mouvement, ETAT *Etat, ASSERV *Longitudinal, ASSERV *Angulaire, ASSERV *RoueD, ASSERV *RoueG, MECANIQUE *Mecanique)
{
    float diff;
    float sensTour;
    int isDirect;

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
                        // MODIFICATION 2015 validée
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;

                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                        // Si direct : on va à la consigne directement, sinon, on fait un tour.
                        isDirect = ((diff >= 0 && Mouvement->EnAvant) || ( diff < 0 && !Mouvement->EnAvant) || fabs(diff) <= 0.0001);
                        if(isDirect)
                            Angulaire->Consigne = Etat->AngleBrut + diff;
                        else
                        {
                            // On définit le sens du tour  : si on va en avant : sens trigo
                            // sinon, sens horaire.
                            sensTour = Mouvement->EnAvant ? 2*PI : -2*PI;
                            Angulaire->Consigne = Etat->AngleBrut + sensTour + diff;
                        }

                        /// @ deprecated
//                        if (Mouvement->EnAvant)
//				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//			else{
//                            float temp1 = Mod2Pi( Mouvement->AngleDemande - Etat->AngleReel );
//                            temp1 =  Mod2Pi( - temp1);
//                            Angulaire->Consigne = Etat->AngleBrut - temp1;
//                            temp1 = 42.0;
//                            //Angulaire->Consigne = Etat->AngleBrut - Mod2Pi( - Mod2Pi( Mouvement->AngleDemande - Etat->AngleReel ) );
//                        }
                        break;

		case 4: //Pointe vers Xc,Yc (Avant / Arrière)
                        // MODIFICATION 2015 validé
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Longitudinal->Consigne = Etat->DistanceTotale;
			Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);

                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                        // Si direct : on va à la consigne directement, sinon, on fait un tour.
                        isDirect = ((diff >= 0 && Mouvement->EnAvant) || ( diff < 0 && !Mouvement->EnAvant) || fabs(diff) <= 0.0001);
                        if(isDirect)
                            Angulaire->Consigne = Etat->AngleBrut + diff;
                        else
                        {
                            // On définit le sens du tour  : si on va en avant : sens trigo
                            // sinon, sens horaire.
                            sensTour = Mouvement->EnAvant ? 2*PI : -2*PI;
                            Angulaire->Consigne = Etat->AngleBrut + sensTour + diff;
                        }

                        /// @deprecated
//                        if (Mouvement->EnAvant)
//				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//			else
//				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
			break;

		case 5: //VaZy Xc, Yc (Decompose, Avant/Arriere)
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

                                        if (!Mouvement->EnAvant)
                                            Mouvement->AngleDemande += PI;

                                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                                        if(diff > PI)
                                        {
                                            diff -= 2*PI;
                                        }
                                        else if(diff < -PI)
                                        {
                                            diff += 2*PI;
                                        }
                                        Angulaire->Consigne = Etat->AngleBrut + diff;


                                        /// @deprecated
//                                        if (Mouvement->EnAvant)
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//					else
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);

                                        break;
				case 1: //Translation
					Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
					if (Mouvement->EnAvant)
						Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
					else
						Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;

                                        ///@deprecated
//                                        if (Mouvement->EnAvant)
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//					else
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
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

                                if (!Mouvement->EnAvant)
                                            Mouvement->AngleDemande += PI;

                                diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                                if(diff > PI)
                                {
                                    diff -= 2*PI;
                                }
                                else if(diff < -PI)
                                {
                                    diff += 2*PI;
                                }
                                Angulaire->Consigne = Etat->AngleBrut + diff;
                                        

                                        /// @deprecated
//                                        if (Mouvement->EnAvant)
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//					else
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);

                                        break;
				case 1: //Translation
					Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
					if (Mouvement->EnAvant)
						Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
					else
						Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;

					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);

                                        if (!Mouvement->EnAvant)
                                            Mouvement->AngleDemande += PI;

                                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                                        if(diff > PI)
                                        {
                                            diff -= 2*PI;
                                        }
                                        else if(diff < -PI)
                                        {
                                            diff += 2*PI;
                                        }
                                        Angulaire->Consigne = Etat->AngleBrut + diff;

                                        ///@deprecated
//                                        if (Mouvement->EnAvant)
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//					else
//						Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
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
			
                        if (!Mouvement->EnAvant)
                            Mouvement->AngleDemande += PI;

                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                        if(diff > PI)
                        {
                            diff -= 2*PI;
                        }
                        else if(diff < -PI)
                        {
                            diff += 2*PI;
                        }
                        Angulaire->Consigne = Etat->AngleBrut + diff;

                        /// @deprecated
//                        if (Mouvement->EnAvant)
//				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
//			else
//				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);

                        break;

		case 8: //Translation à vitesse constante avec angle fixe
			Mouvement->ConfigAsserv = (AsservVitLong | AsservPosAng | AsservVitAng);

			if (Mouvement->EnAvant)
				Longitudinal->VitesseConsigne = Longitudinal->VitesseDemande;
			else
				Longitudinal->VitesseConsigne = -Longitudinal->VitesseDemande;
			break;

		case 9: //Rotation à vitesse constante sur place
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservVitAng);

			if (Mouvement->EnAvant)
				Angulaire->VitesseConsigne = Angulaire->VitesseDemande;
			else
				Angulaire->VitesseConsigne = -Angulaire->VitesseDemande;
			break;

                case 10: //Passer par le point Xc, Yc à vitesse constante, en décomposant
			//Calcul de la distance de consigne pour aller au point Xc, Yc (non utilisé pour l'asserv, uniquement pour dire au PC quand on est arrivé)
			//Etape 0 :
				//Rotation
			//Etape 1:
				//Translation avec correction permanente de l'angle
                        // Etape 3 : avancer d'une petite distance en attendant l'ordre suivant du PC
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			switch(Mouvement->Etape)
			{
				case 0: //Rotation
					Longitudinal->Consigne = Etat->DistanceTotale;
					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);

                                        if (!Mouvement->EnAvant)
                                                    Mouvement->AngleDemande += PI;

                                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                                        if(diff > PI)
                                        {
                                            diff -= 2*PI;
                                        }
                                        else if(diff < -PI)
                                        {
                                            diff += 2*PI;
                                        }
                                        Angulaire->Consigne = Etat->AngleBrut + diff;

                                        break;
				case 1: // Translation
					Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
					if (Mouvement->EnAvant)
						Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
					else
						Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;

//                                        // Deplacement à vitesse longitudinale constante
                                        if (Mouvement->EnAvant)
                                                Longitudinal->VitesseConsigne = Longitudinal->VitesseDemande;
                                        else
                                                Longitudinal->VitesseConsigne = -Longitudinal->VitesseDemande;

					Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);

                                        if (!Mouvement->EnAvant)
                                            Mouvement->AngleDemande += PI;

                                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
                                        if(diff > PI)
                                        {
                                            diff -= 2*PI;
                                        }
                                        else if(diff < -PI)
                                        {
                                            diff += 2*PI;
                                        }
                                        Angulaire->Consigne = Etat->AngleBrut + diff;

                                        break;
				default :
					break;
			}
			break;

                /// @deprecated
//		case 10: //Passer par le point Xc, Yc à vitesse constante
//			//Calcul de la distance de consigne pour aller au point Xc, Yc (non utilisé pour l'asserv, uniquement pour dire au PC quand on est arrivé)
//			Mouvement->ConfigAsserv = (AsservVitLong | AsservPosAng | AsservVitAng);
//
//			Mouvement->DistanceDemande = sqrt((Mouvement->Yc - Etat->Y)*(Mouvement->Yc - Etat->Y) + (Mouvement->Xc - Etat->X)*(Mouvement->Xc - Etat->X));
//			if (Mouvement->EnAvant)
//				Longitudinal->Consigne = Etat->DistanceTotale + Mouvement->DistanceDemande;
//			else
//				Longitudinal->Consigne = Etat->DistanceTotale - Mouvement->DistanceDemande;
//
//			//Deplacement à vitesse longitudinale constante
//			if (Mouvement->EnAvant)
//				Longitudinal->VitesseConsigne = Longitudinal->VitesseDemande;
//			else
//				Longitudinal->VitesseConsigne = -Longitudinal->VitesseDemande;
//
//			//Reglage permanent de l'angle voulu
//			 Mouvement->AngleDemande = atan2f(Mouvement->Yc - Etat->Y, Mouvement->Xc - Etat->X);
//
//                        if (!Mouvement->EnAvant)
//                            Mouvement->AngleDemande += PI;
//
//                        diff = (ToMoinsPiPi(Mouvement->AngleDemande) - Etat->AngleReel ); //shift de l'angle demandé pour qu'il soit entre [-pi;pi]
//
//                        Angulaire->Consigne = Etat->AngleBrut + diff;
//
//                        /// @deprecated
////                        if (Mouvement->EnAvant)
////				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(Mouvement->AngleDemande - Etat->AngleReel);
////			else
////				Angulaire->Consigne = Etat->AngleBrut + Mod2Pi(PI + Mouvement->AngleDemande - Etat->AngleReel);
//			break;
//                      

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
				RoueD->VitesseConsigne = RoueD->VitesseDemande;
			else
				RoueD->VitesseConsigne = -RoueD->VitesseDemande;

			if (Mouvement->EnAvant)
				RoueG->VitesseConsigne = RoueG->VitesseDemande;
			else
				RoueG->VitesseConsigne = -RoueG->VitesseDemande;
			break;

		case 15:
			Mouvement->ConfigAsserv = (AsservVitLong | AsservVitAng);
			Longitudinal->VitesseConsigne = 0.0;
			Angulaire->VitesseConsigne = 0.0;
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut;
			break;

		case 17: //Commande vitesse long & ang
			Mouvement->ConfigAsserv = (AsservVitLong | AsservVitAng);

			//Toujours vers l'avant => corriger le bug
			Longitudinal->VitesseConsigne = Longitudinal->VitesseDemande;

			if (Mouvement->EnAvant)
				Angulaire->VitesseConsigne = Angulaire->VitesseDemande;
			else
				Angulaire->VitesseConsigne = -Angulaire->VitesseDemande;
			break;

		default : //Stop
			Mouvement->ConfigAsserv = (AsservPosLong | AsservVitLong | AsservPosAng | AsservVitAng);
			Mouvement->Type = 0;
			Longitudinal->Consigne = Etat->DistanceTotale;
			Angulaire->Consigne = Etat->AngleBrut;
			break;
	}
}


void SeuillageVitesse(ASSERV *Param, CARTE *Carte)
{
	Param->VitesseConsigne = SeuillerDansInterval(Param->VitesseConsigne, fabs(Param->VitesseMax));

	if (Param->VitesseConsigne - Param->VitesseConsignePrecedente > Param->Acceleration * Carte->TeAsserv)
		Param->VitesseConsigne = Param->VitesseConsignePrecedente + Param->Acceleration * Carte->TeAsserv;
	else if (Param->VitesseConsigne - Param->VitesseConsignePrecedente < -Param->Decceleration * Carte->TeAsserv)
		Param->VitesseConsigne = Param->VitesseConsignePrecedente - Param->Decceleration * Carte->TeAsserv;
}

void AsservPositionLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidPositionLongitudinale, CARTE *Carte)
{
	CalculCorrecteur(Etat->DistanceTotale, Longitudinal->Consigne, PidPositionLongitudinale, Carte);
	Longitudinal->VitesseConsignePrecedente = Longitudinal->VitesseConsigne;
	Longitudinal->VitesseConsigne = PidPositionLongitudinale->Commande;
	SeuillageVitesse(Longitudinal, Carte);
}

void AsservPositionAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidPositionAngulaire, CARTE *Carte)
{
	CalculCorrecteur(Etat->AngleBrut, Angulaire->Consigne, PidPositionAngulaire, Carte);
	Angulaire->VitesseConsignePrecedente = Angulaire->VitesseConsigne;
	Angulaire->VitesseConsigne = PidPositionAngulaire->Commande;
	SeuillageVitesse(Angulaire, Carte);
}

void AsservVitesseLongitudinale(ETAT *Etat, ASSERV *Longitudinal, CORRECTEUR *PidVitesseLongitudinale, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur(Etat->VitesseLongitudinale, Longitudinal->VitesseConsigne, PidVitesseLongitudinale, Carte);
	PidVitesseLongitudinale->Commande = SeuillerDansInterval(PidVitesseLongitudinale->Commande, Mecanique->PuissanceMaxLongitudinal);
}

void AsservVitesseAngulaire(ETAT *Etat, ASSERV *Angulaire, CORRECTEUR *PidVitesseAngulaire, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur(Etat->VitesseAngulaire, Angulaire->VitesseConsigne, PidVitesseAngulaire, Carte);
	PidVitesseAngulaire->Commande = SeuillerDansInterval(PidVitesseAngulaire->Commande, Mecanique->PuissanceMaxAngulaire);
}

void ApplicationPuissanceMoteurPolaire(CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, MECANIQUE *Mecanique)
{
	Mecanique->PuissanceMoteurD = SeuillerDansInterval(-PidVitesseLongitudinale->Commande - PidVitesseAngulaire->Commande, 500);
	Mecanique->PuissanceMoteurG = SeuillerDansInterval(-PidVitesseLongitudinale->Commande + PidVitesseAngulaire->Commande, 500);
}







void AsservPositionD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidPositionD, CARTE *Carte)
{
	CalculCorrecteur(Etat->DistanceRoueD, RoueD->Consigne, PidPositionD, Carte);
	RoueD->VitesseConsignePrecedente = RoueD->VitesseConsigne;
	RoueD->VitesseConsigne = PidPositionD->Commande;
	SeuillageVitesse(RoueD, Carte);
}

void AsservPositionG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidPositionG, CARTE *Carte)
{
	CalculCorrecteur(Etat->DistanceRoueG, RoueG->Consigne, PidPositionG, Carte);
	RoueG->VitesseConsignePrecedente = RoueG->VitesseConsigne;
	RoueG->VitesseConsigne = PidPositionG->Commande;
	SeuillageVitesse(RoueG, Carte);
}

void AsservVitesseD(ETAT *Etat, ASSERV *RoueD, CORRECTEUR *PidVitesseD, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur(Etat->VitesseRoueD, RoueD->VitesseConsigne, PidVitesseD, Carte);
	PidVitesseD->Commande = SeuillerDansInterval(PidVitesseD->Commande, Mecanique->PuissanceMaxD);
}

void AsservVitesseG(ETAT *Etat, ASSERV *RoueG, CORRECTEUR *PidVitesseG, CARTE *Carte, MECANIQUE *Mecanique)
{
	CalculCorrecteur(Etat->VitesseRoueG, RoueG->VitesseConsigne, PidVitesseG, Carte);
	PidVitesseG->Commande = SeuillerDansInterval(PidVitesseG->Commande, Mecanique->PuissanceMaxG);
}

void ApplicationPuissanceMoteurDG(CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MECANIQUE *Mecanique)
{
	Mecanique->PuissanceMoteurD = SeuillerDansInterval(PidVitesseD->Commande, 500);
	Mecanique->PuissanceMoteurG = SeuillerDansInterval(PidVitesseG->Commande, 500);
}
