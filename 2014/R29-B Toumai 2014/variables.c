#include "variables.h"

//fonction d'initialisation des variables?
void VARInitialize(CARTE *Carte, MECANIQUE *Mecanique, ASSERV *Longitudinal, ASSERV *Angulaire, ETAT *Etat, CORRECTEUR *PidPositionLongitudinale, CORRECTEUR *PidPositionAngulaire, CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, ASSERV *RoueD, ASSERV *RoueG, CORRECTEUR *PidPositionD, CORRECTEUR *PidPositionG, CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MOUVEMENT *MouvementActuel, int DemarageCarte)
{
	//Variables de configuration (variables servant au parametrage de la carte)
		Carte->Fcy = 39613750.0;
		Carte->Tcy = 1.0 / Carte->Fcy;

		Carte->TeAsserv = 0.01;
		Carte->TeOdometrie = 0.01;
		Carte->Fe = 1.0 / Carte->TeAsserv;

		Carte->Fpwm = 20000.0;
		Carte->Tpwm = 1.0 / Carte->Fpwm;
	// Fin Variables de configuration

	//Variables mécaniques (variables permettant de caractériser les grandeurs mécaniques utiles)
	if(DemarageCarte == 1)
	{
		Mecanique->EntreAxe = 0.223883; // Valeur au 23/05/2014 : 0.222869

		Mecanique->DiametreRoueD = 0.0636288; // Valeurs calibration 23/05/2014  : 0.0636158
		Mecanique->DiametreRoueG = 0.0638029;// Ancienne valeur au 23/05/2014 : 0.0637898

		Mecanique->PuissanceMoteurD = 0;
		Mecanique->PuissanceMoteurG = 0;

		Mecanique->PuissanceMaxD = 400.0;
		Mecanique->PuissanceMaxG = 400.0;
		Mecanique->PuissanceMaxLongitudinal = 500.0;
		Mecanique->PuissanceMaxAngulaire = 500.0;

		Mecanique->VitesseLongitudinaleMax = 1.5;
		Mecanique->VitesseAngulaireMax = 8.0;

		Mecanique->VitesseRoueDMax = 1.5;
		Mecanique->VitesseRoueGMax = 1.5;

		Mecanique->NombrePasD = 1024;
		Mecanique->NombrePasG = 1024;
	}
	//Fin variables mécaniques

	//Variables d'état (traduisent l'état réel du robot)
		Etat->DistanceTotale = 0;
		Etat->DistanceRoueD = 0;
		Etat->DistanceRoueG = 0;

		Etat->AngleBrut = 0;
		Etat->AngleReel = 0;

		Etat->VitesseLongitudinale = 0;
		Etat->VitesseAngulaire = 0;
		Etat->VitesseRoueD = 0;
		Etat->VitesseRoueG = 0;

		Etat->X = 0;
		Etat->Y = 0;

		Etat->EnvoiAuto = 0;
	//Fin variables d'état

	//Variables Asservissement (variables servant à la génération des consignes pour le robot)
		/*-------------------------------------------------*/
		Longitudinal->Desire = 0;
		Longitudinal->Consigne = 0;

		Longitudinal->VitesseMax = 1.4; //0.8
		Longitudinal->VitesseDemande = 0.0;
		Longitudinal->VitesseConsigne = 0;
		Longitudinal->VitesseConsignePrecedente = 0;

		Longitudinal->Acceleration = 3.0; //5.0
		Longitudinal->Decceleration = 3.0; //3.5;

		Longitudinal->Est_Active = 1;
		/*-------------------------------------------------*/
		Angulaire->Desire = 0;
		Angulaire->Consigne = 0;

		Angulaire->VitesseMax = 5.0;
		Angulaire->VitesseDemande = 0.0;
		Angulaire->VitesseConsigne = 0;
		Angulaire->VitesseConsignePrecedente = 0;

		Angulaire->Acceleration = 30.0;
		Angulaire->Decceleration = 30.0;

		Angulaire->Est_Active = 1;
		/*-------------------------------------------------*/

		RoueD->Desire = 0;
		RoueD->Consigne = 0;

		RoueD->VitesseMax = 1.0;
		RoueD->VitesseDemande = 0.0;
		RoueD->VitesseConsigne = 0;
		RoueD->VitesseConsignePrecedente = 0;

		RoueD->Acceleration = 2.0;
		RoueD->Decceleration = 2.0;

		RoueD->Est_Active = 1;
		/*-------------------------------------------------*/

		RoueG->Desire = 0;
		RoueG->Consigne = 0;

		RoueG->VitesseMax = 1.0;
		RoueG->VitesseDemande = 0.0;
		RoueG->VitesseConsigne = 0;
		RoueG->VitesseConsignePrecedente = 0;

		RoueG->Acceleration = 2.0;
		RoueG->Decceleration = 2.0;

		RoueG->Est_Active = 1;
		/*-------------------------------------------------*/
	//Fin variables Asservissement

	//Variables des correcteurs
		/*-------------------------------------------------*/
		PidPositionLongitudinale->Erreur = 0;
		PidPositionLongitudinale->ErreurPrecedente = 0;
		PidPositionLongitudinale->IntegraleErreur = 0;
		PidPositionLongitudinale->DeriveErreur = 0;

		PidPositionLongitudinale->CommandeProportionelle = 0;
		PidPositionLongitudinale->CommandeIntegrale = 0;
		PidPositionLongitudinale->CommandeDerive = 0;

		PidPositionLongitudinale->Kp = 4.8;
		PidPositionLongitudinale->Ki = 0;
		PidPositionLongitudinale->Kd = 0;

		PidPositionLongitudinale->KpArret = 7.0; //6.0;
		PidPositionLongitudinale->KiArret = 0;
		PidPositionLongitudinale->KdArret = 0;

		PidPositionLongitudinale->KpRoule = 7.0; //6.2;
		PidPositionLongitudinale->KiRoule = 0;
		PidPositionLongitudinale->KdRoule = 0;

		PidPositionLongitudinale->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionAngulaire->Erreur = 0;
		PidPositionAngulaire->ErreurPrecedente = 0;
		PidPositionAngulaire->IntegraleErreur = 0;
		PidPositionAngulaire->DeriveErreur = 0;

		PidPositionAngulaire->CommandeProportionelle = 0;
		PidPositionAngulaire->CommandeIntegrale = 0;
		PidPositionAngulaire->CommandeDerive = 0;

		PidPositionAngulaire->Kp = 4;
		PidPositionAngulaire->Ki = 0;
		PidPositionAngulaire->Kd = 0;

		PidPositionAngulaire->KpArret = 10; //9
		PidPositionAngulaire->KiArret = 0;
		PidPositionAngulaire->KdArret = 0;

		PidPositionAngulaire->KpRoule = 10; //9
		PidPositionAngulaire->KiRoule = 0;
		PidPositionAngulaire->KdRoule = 0;

		PidPositionAngulaire->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseLongitudinale->Erreur = 0;
		PidVitesseLongitudinale->ErreurPrecedente = 0;
		PidVitesseLongitudinale->IntegraleErreur = 0;
		PidVitesseLongitudinale->DeriveErreur = 0;

		PidVitesseLongitudinale->CommandeProportionelle = 0;
		PidVitesseLongitudinale->CommandeIntegrale = 0;
		PidVitesseLongitudinale->CommandeDerive = 0;

		PidVitesseLongitudinale->Kp = 1500;
		PidVitesseLongitudinale->Ki = 0;
		PidVitesseLongitudinale->Kd = 0;

		PidVitesseLongitudinale->KpArret = 1000;//2000
		PidVitesseLongitudinale->KiArret = 0;
		PidVitesseLongitudinale->KdArret = 0;

		PidVitesseLongitudinale->KpRoule = 1000;//2000
		PidVitesseLongitudinale->KiRoule = 0;
		PidVitesseLongitudinale->KdRoule = 0;

		PidVitesseLongitudinale->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseAngulaire->Erreur = 0;
		PidVitesseAngulaire->ErreurPrecedente = 0;
		PidVitesseAngulaire->IntegraleErreur = 0;
		PidVitesseAngulaire->DeriveErreur = 0;

		PidVitesseAngulaire->CommandeProportionelle = 0;
		PidVitesseAngulaire->CommandeIntegrale = 0;
		PidVitesseAngulaire->CommandeDerive = 0;

		PidVitesseAngulaire->Kp = 300;
		PidVitesseAngulaire->Ki = 0;
		PidVitesseAngulaire->Kd = 0;

		PidVitesseAngulaire->KpArret = 300;
		PidVitesseAngulaire->KiArret = 0;
		PidVitesseAngulaire->KdArret = 0;

		PidVitesseAngulaire->KpRoule = 300;
		PidVitesseAngulaire->KiRoule = 0;
		PidVitesseAngulaire->KdRoule = 0;

		PidVitesseAngulaire->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionD->Erreur = 0;
		PidPositionD->ErreurPrecedente = 0;
		PidPositionD->IntegraleErreur = 0;
		PidPositionD->DeriveErreur = 0;

		PidPositionD->CommandeProportionelle = 0;
		PidPositionD->CommandeIntegrale = 0;
		PidPositionD->CommandeDerive = 0;

		PidPositionD->Kp = 5.0;
		PidPositionD->Ki = 0;
		PidPositionD->Kd = 0;

		/* Non utilise
		PidPositionD->KpArret = 5.77;
		PidPositionD->KiArret = 0;
		PidPositionD->KdArret = 0;

		PidPositionD->KpRoule = 5.77;
		PidPositionD->KiRoule = 0;
		PidPositionD->KdRoule = 0;
		*/
		
		PidPositionD->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionG->Erreur = 0;
		PidPositionG->ErreurPrecedente = 0;
		PidPositionG->IntegraleErreur = 0;
		PidPositionG->DeriveErreur = 0;

		PidPositionG->CommandeProportionelle = 0;
		PidPositionG->CommandeIntegrale = 0;
		PidPositionG->CommandeDerive = 0;

		PidPositionG->Kp = 5.0;
		PidPositionG->Ki = 0;
		PidPositionG->Kd = 0;

		/* Non utilise
		PidPositionG->KpArret = 5.77;
		PidPositionG->KiArret = 0;
		PidPositionG->KdArret = 0;

		PidPositionG->KpRoule = 5.77;
		PidPositionG->KiRoule = 0;
		PidPositionG->KdRoule = 0;
		*/
		
		PidPositionG->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseD->Erreur = 0;
		PidVitesseD->ErreurPrecedente = 0;
		PidVitesseD->IntegraleErreur = 0;
		PidVitesseD->DeriveErreur = 0;

		PidVitesseD->CommandeProportionelle = 0;
		PidVitesseD->CommandeIntegrale = 0;
		PidVitesseD->CommandeDerive = 0;

		PidVitesseD->Kp = 900;
		PidVitesseD->Ki = 0;
		PidVitesseD->Kd = 0;

		PidVitesseD->KpArret = 600;
		PidVitesseD->KiArret = 0;
		PidVitesseD->KdArret = 0;

		PidVitesseD->KpRoule = 900;
		PidVitesseD->KiRoule = 0;
		PidVitesseD->KdRoule = 0;

		PidVitesseD->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseG->Erreur = 0;
		PidVitesseG->ErreurPrecedente = 0;
		PidVitesseG->IntegraleErreur = 0;
		PidVitesseG->DeriveErreur = 0;

		PidVitesseG->CommandeProportionelle = 0;
		PidVitesseG->CommandeIntegrale = 0;
		PidVitesseG->CommandeDerive = 0;

		PidVitesseG->Kp = 900;
		PidVitesseG->Ki = 0;
		PidVitesseG->Kd = 0;

		PidVitesseG->KpArret = 600;
		PidVitesseG->KiArret = 0;
		PidVitesseG->KdArret = 0;

		PidVitesseG->KpRoule = 900;
		PidVitesseG->KiRoule = 0;
		PidVitesseG->KdRoule = 0;

		PidVitesseG->Commande = 0;
		/*-------------------------------------------------*/
	//Fin variables des correcteurs

	//Variables de mouvement
		MouvementActuel->DistanceDemande = 0;
		MouvementActuel->AngleDemande = 0;

		MouvementActuel->DistanceDroiteDemande = 0;
		MouvementActuel->DistanceGaucheDemande = 0;

		MouvementActuel->Xc = 0;
		MouvementActuel->Yc = 0;

		MouvementActuel->PrecisionLongitudinale = 0.06; //6cm
		MouvementActuel->PrecisionAngulaire = 10.0 * PI / 180.0; // 10°

		MouvementActuel->Type = 0;
		MouvementActuel->EnAvant = 1;
		MouvementActuel->Etape = 0;
		MouvementActuel->Termine = 0;
		MouvementActuel->TermineEnvoye = 0;
		MouvementActuel->TempsBlocage = 0;

		MouvementActuel->ConfigAsserv = 0;
}
