#include "variables.h"
#include "configuration.h"

//fonction d'initialisation des variables?
void VARInitialize(CARTE *Carte, MECANIQUE *Mecanique, ASSERV *Longitudinal, ASSERV *Angulaire, ETAT *Etat, CORRECTEUR *PidPositionLongitudinale, CORRECTEUR *PidPositionAngulaire, CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, ASSERV *RoueD, ASSERV *RoueG, CORRECTEUR *PidPositionD, CORRECTEUR *PidPositionG, CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MOUVEMENT *MouvementActuel, int DemarageCarte)
{
	//Variables de configuration (variables servant au parametrage de la carte)
		Carte->Fcy = 40000000.0; // 39613750.0;
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
		Mecanique->EntreAxe = ENTREAXE; // Valeur au 23/05/2014 : 0.222869

		Mecanique->DiametreRoueD = DIAMETRE_ROUE_D; // Valeurs calibration 23/05/2014  : 0.0636158
		Mecanique->DiametreRoueG = DIAMETRE_ROUE_G;// Ancienne valeur au 23/05/2014 : 0.0637898

		Mecanique->PuissanceMoteurD = 0;
		Mecanique->PuissanceMoteurG = 0;

		Mecanique->PuissanceMaxD = 400.0;
		Mecanique->PuissanceMaxG = 400.0;
		Mecanique->PuissanceMaxLongitudinal = 500.0;
		Mecanique->PuissanceMaxAngulaire = 500.0;

		Mecanique->VitesseLongitudinaleMax = VIT_MAX_LONGITUDINAL;
		Mecanique->VitesseAngulaireMax = VIT_MAX_ANGULAIRE;

		Mecanique->VitesseRoueDMax = VIT_MAX_ROUE_D;
		Mecanique->VitesseRoueGMax = VIT_MAX_ROUE_G;

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

		Longitudinal->VitesseMax = VIT_MAX_LONGITUDINAL; //0.8
		Longitudinal->VitesseDemande = 0.0;
		Longitudinal->VitesseConsigne = 0;
		Longitudinal->VitesseConsignePrecedente = 0;

		Longitudinal->Acceleration = VIT_MAX_ANGULAIRE; //5.0
		Longitudinal->Decceleration = VIT_MAX_ANGULAIRE; //3.5;

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

		PidPositionLongitudinale->Kp = Kp_POS_LONGITUDINAL;
		PidPositionLongitudinale->Ki = Ki_POS_LONGITUDINAL;
		PidPositionLongitudinale->Kd = Kd_POS_LONGITUDINAL;

		PidPositionLongitudinale->KpArret = Kp_POS_LONGITUDINAL;
		PidPositionLongitudinale->KiArret = Ki_POS_LONGITUDINAL;
		PidPositionLongitudinale->KdArret = Kd_POS_LONGITUDINAL;

		PidPositionLongitudinale->KpRoule = Kp_POS_LONGITUDINAL;
		PidPositionLongitudinale->KiRoule = Ki_POS_LONGITUDINAL;
		PidPositionLongitudinale->KdRoule = Kd_POS_LONGITUDINAL;

		PidPositionLongitudinale->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionAngulaire->Erreur = 0;
		PidPositionAngulaire->ErreurPrecedente = 0;
		PidPositionAngulaire->IntegraleErreur = 0;
		PidPositionAngulaire->DeriveErreur = 0;

		PidPositionAngulaire->CommandeProportionelle = 0;
		PidPositionAngulaire->CommandeIntegrale = 0;
		PidPositionAngulaire->CommandeDerive = 0;

		PidPositionAngulaire->Kp = Kp_POS_ANGULAIRE;
		PidPositionAngulaire->Ki = Ki_POS_ANGULAIRE;
		PidPositionAngulaire->Kd = Kd_POS_ANGULAIRE;

		PidPositionAngulaire->KpArret = Kp_POS_ANGULAIRE;
		PidPositionAngulaire->KiArret = Ki_POS_ANGULAIRE;
		PidPositionAngulaire->KdArret = Kd_POS_ANGULAIRE;

		PidPositionAngulaire->KpRoule = Kp_POS_ANGULAIRE;
		PidPositionAngulaire->KiRoule = Ki_POS_ANGULAIRE;
		PidPositionAngulaire->KdRoule = Kd_POS_ANGULAIRE;

		PidPositionAngulaire->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseLongitudinale->Erreur = 0;
		PidVitesseLongitudinale->ErreurPrecedente = 0;
		PidVitesseLongitudinale->IntegraleErreur = 0;
		PidVitesseLongitudinale->DeriveErreur = 0;

		PidVitesseLongitudinale->CommandeProportionelle = 0;
		PidVitesseLongitudinale->CommandeIntegrale = 0;
		PidVitesseLongitudinale->CommandeDerive = 0;

		PidVitesseLongitudinale->Kp = Kp_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->Ki = Ki_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->Kd = 0;

		PidVitesseLongitudinale->KpArret = Kp_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->KiArret = Ki_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->KdArret = 0;

		PidVitesseLongitudinale->KpRoule = Kp_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->KiRoule = Ki_VIT_LONGITUDINAL;
		PidVitesseLongitudinale->KdRoule = Kd_VIT_LONGITUDINAL;

		PidVitesseLongitudinale->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseAngulaire->Erreur = 0;
		PidVitesseAngulaire->ErreurPrecedente = 0;
		PidVitesseAngulaire->IntegraleErreur = 0;
		PidVitesseAngulaire->DeriveErreur = 0;

		PidVitesseAngulaire->CommandeProportionelle = 0;
		PidVitesseAngulaire->CommandeIntegrale = 0;
		PidVitesseAngulaire->CommandeDerive = 0;

		PidVitesseAngulaire->Kp = Kp_VIT_ANGULAIRE;
		PidVitesseAngulaire->Ki = Ki_VIT_ANGULAIRE;
		PidVitesseAngulaire->Kd = Kd_VIT_ANGULAIRE;

		PidVitesseAngulaire->KpArret = Kp_VIT_ANGULAIRE;
		PidVitesseAngulaire->KiArret = Ki_VIT_ANGULAIRE;
		PidVitesseAngulaire->KdArret = Kd_VIT_ANGULAIRE;

		PidVitesseAngulaire->KpRoule = Kp_VIT_ANGULAIRE;
		PidVitesseAngulaire->KiRoule = Ki_VIT_ANGULAIRE;
		PidVitesseAngulaire->KdRoule = Kd_VIT_ANGULAIRE;

		PidVitesseAngulaire->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionD->Erreur = 0;
		PidPositionD->ErreurPrecedente = 0;
		PidPositionD->IntegraleErreur = 0;
		PidPositionD->DeriveErreur = 0;

		PidPositionD->CommandeProportionelle = 0;
		PidPositionD->CommandeIntegrale = 0;
		PidPositionD->CommandeDerive = 0;

		PidPositionD->Kp = Kp_ROUE_DROITE;
		PidPositionD->Ki = Ki_ROUE_DROITE;
		PidPositionD->Kd = Kd_ROUE_DROITE;

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

		PidPositionG->Kp = Kp_ROUE_GAUCHE;
		PidPositionG->Ki = Ki_ROUE_GAUCHE;
		PidPositionG->Kd = Kd_ROUE_GAUCHE;

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

		PidVitesseD->Kp = Kp_VIT_ROUE_DROITE; // 900
		PidVitesseD->Ki = Ki_VIT_ROUE_DROITE;
		PidVitesseD->Kd = Kd_VIT_ROUE_DROITE;

		PidVitesseD->KpArret = Kp_VIT_ROUE_DROITE; // 600
		PidVitesseD->KiArret = Ki_VIT_ROUE_DROITE;
		PidVitesseD->KdArret = Kd_VIT_ROUE_DROITE;

		PidVitesseD->KpRoule = Kp_VIT_ROUE_DROITE;
		PidVitesseD->KiRoule = Ki_VIT_ROUE_DROITE;
		PidVitesseD->KdRoule = Kd_VIT_ROUE_DROITE;

		PidVitesseD->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseG->Erreur = 0;
		PidVitesseG->ErreurPrecedente = 0;
		PidVitesseG->IntegraleErreur = 0;
		PidVitesseG->DeriveErreur = 0;

		PidVitesseG->CommandeProportionelle = 0;
		PidVitesseG->CommandeIntegrale = 0;
		PidVitesseG->CommandeDerive = 0;

		PidVitesseG->Kp = Kp_VIT_ROUE_GAUCHE; // 900
		PidVitesseG->Ki = Ki_VIT_ROUE_GAUCHE;
		PidVitesseG->Kd = Kd_VIT_ROUE_GAUCHE;

		PidVitesseG->KpArret = Kp_VIT_ROUE_GAUCHE; // 600
		PidVitesseG->KiArret = Ki_VIT_ROUE_GAUCHE;
		PidVitesseG->KdArret = Kd_VIT_ROUE_GAUCHE;

		PidVitesseG->KpRoule = Kp_VIT_ROUE_GAUCHE;
		PidVitesseG->KiRoule = Ki_VIT_ROUE_GAUCHE;
		PidVitesseG->KdRoule = Kd_VIT_ROUE_GAUCHE;

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

		MouvementActuel->PrecisionLongitudinale = PRECISION_LONGITUDINALE; 
		MouvementActuel->PrecisionAngulaire = PRECISION_ANGULAIRE; 

		MouvementActuel->Type = 0;
		MouvementActuel->EnAvant = 1;
		MouvementActuel->Etape = 0;
		MouvementActuel->Termine = 0;
		MouvementActuel->TermineEnvoye = 0;
		MouvementActuel->TempsBlocage = 0;

		MouvementActuel->ConfigAsserv = 0;
}
