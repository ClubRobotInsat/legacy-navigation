#include "variables.h"

//fonction d'initialisation des variables?
void VARInitialize(CARTE *Carte, MECANIQUE *Mecanique, ASSERV *Longitudinal, ASSERV *Angulaire, ETAT *Etat, CORRECTEUR *PidPositionLongitudinale, CORRECTEUR *PidPositionAngulaire, CORRECTEUR *PidVitesseLongitudinale, CORRECTEUR *PidVitesseAngulaire, ASSERV *RoueD, ASSERV *RoueG, CORRECTEUR *PidPositionD, CORRECTEUR *PidPositionG, CORRECTEUR *PidVitesseD, CORRECTEUR *PidVitesseG, MOUVEMENT *MouvementActuel)
{
	//Variables de configuration (variables servant au parametrage de la carte)
		Carte->Fcy = 29480000.0;
		Carte->Tcy = 1.0 / Carte->Fcy;

		Carte->Fe = 1.0/0.025;
		Carte->Te = 0.025;

		Carte->Fpwm = 20000.0;
		Carte->Tpwm = 0.00005;
	// Fin Variables de configuration

	//Variables mécaniques (variables permettant de caractériser les grandeurs mécaniques utiles)
		Mecanique->EntreAxe = 0.3461165159;

		Mecanique->DiametreRoueD = 0.0705187764;
		Mecanique->DiametreRoueG = 0.0705187764;

		Mecanique->PuissanceMoteurD = 0;
		Mecanique->PuissanceMoteurG = 0;

		Mecanique->PuissanceMaxD = 400.0;
		Mecanique->PuissanceMaxG = 400.0;
		Mecanique->PuissanceMaxLongitudinal = 500.0;
		Mecanique->PuissanceMaxAngulaire = 500.0;

		Mecanique->VitesseLongitudinaleMax = 1.5;
		Mecanique->VitesseAngulaireMax = 1.5 * 2.0 * PI;

		Mecanique->VitesseRoueDMax = 1.5;
		Mecanique->VitesseRoueGMax = 1.5;
		
		Mecanique->Nombre_Pas_D = 200;
		Mecanique->Nombre_Pas_G = 200;
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

		Etat->envoi_auto = 0;
	//Fin variables d'état

	//Variables Asservissement (variables servant à la génération des consignes pour le robot)
		/*-------------------------------------------------*/
		Longitudinal->Desire = 0;
		Longitudinal->Consigne = 0;

		Longitudinal->VitesseMax = 1.2;
		Longitudinal->VitesseDemande = 1.2;
		Longitudinal->VitesseConsigne = 0;
		Longitudinal->VitesseConsignePrecedente = 0;

		Longitudinal->Acceleration = 3.5;

		Longitudinal->Est_Active = 1;
		/*-------------------------------------------------*/
		Angulaire->Desire = 0;
		Angulaire->Consigne = 0;

		Angulaire->VitesseMax = 4.0;
		Angulaire->VitesseDemande = 4.0;
		Angulaire->VitesseConsigne = 0;
		Angulaire->VitesseConsignePrecedente = 0;

		Angulaire->Acceleration = 28.0;

		Angulaire->Est_Active = 1;
		/*-------------------------------------------------*/

		RoueD->Desire = 0;
		RoueD->Consigne = 0;

		RoueD->VitesseMax = 1.2;
		RoueD->VitesseDemande = 1.2;
		RoueD->VitesseConsigne = 0;
		RoueD->VitesseConsignePrecedente = 0;

		RoueD->Acceleration = 3.5;

		RoueD->Est_Active = 1;
		/*-------------------------------------------------*/

		RoueG->Desire = 0;
		RoueG->Consigne = 0;

		RoueG->VitesseMax = 1.2;
		RoueG->VitesseDemande = 1.2;
		RoueG->VitesseConsigne = 0;
		RoueG->VitesseConsignePrecedente = 0;

		RoueG->Acceleration = 3.5;

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

		PidPositionLongitudinale->Kp = 6;
		PidPositionLongitudinale->Ki = 0;
		PidPositionLongitudinale->Kd = 0;

		PidPositionLongitudinale->KpArret = 6;
		PidPositionLongitudinale->KiArret = 0;
		PidPositionLongitudinale->KdArret = 0;

		PidPositionLongitudinale->KpRoule = 6;
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

		PidPositionAngulaire->Kp = 6;
		PidPositionAngulaire->Ki = 0;
		PidPositionAngulaire->Kd = 0;

		PidPositionAngulaire->KpArret = 6;
		PidPositionAngulaire->KiArret = 0;
		PidPositionAngulaire->KdArret = 0;

		PidPositionAngulaire->KpRoule = 6;
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

		PidVitesseLongitudinale->Kp = 2000;
		PidVitesseLongitudinale->Ki = 0;
		PidVitesseLongitudinale->Kd = 0;

		PidVitesseLongitudinale->KpArret = 1200;
		PidVitesseLongitudinale->KiArret = 0;
		PidVitesseLongitudinale->KdArret = 0;

		PidVitesseLongitudinale->KpRoule = 2000;
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

		PidVitesseAngulaire->KpArret = 150;
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

		PidPositionD->Kp = 5;
		PidPositionD->Ki = 0;
		PidPositionD->Kd = 0;

		PidPositionD->KpArret = 5;
		PidPositionD->KiArret = 0;
		PidPositionD->KdArret = 0;

		PidPositionD->KpRoule = 5;
		PidPositionD->KiRoule = 0;
		PidPositionD->KdRoule = 0;

		PidPositionD->Commande = 0;
		/*-------------------------------------------------*/
		PidPositionG->Erreur = 0;
		PidPositionG->ErreurPrecedente = 0;
		PidPositionG->IntegraleErreur = 0;
		PidPositionG->DeriveErreur = 0;

		PidPositionG->CommandeProportionelle = 0;
		PidPositionG->CommandeIntegrale = 0;
		PidPositionG->CommandeDerive = 0;

		PidPositionG->Kp = 5;
		PidPositionG->Ki = 0;
		PidPositionG->Kd = 0;

		PidPositionG->KpArret = 5;
		PidPositionG->KiArret = 0;
		PidPositionG->KdArret = 0;

		PidPositionG->KpRoule = 5;
		PidPositionG->KiRoule = 0;
		PidPositionG->KdRoule = 0;

		PidPositionG->Commande = 0;
		/*-------------------------------------------------*/
		PidVitesseD->Erreur = 0;
		PidVitesseD->ErreurPrecedente = 0;
		PidVitesseD->IntegraleErreur = 0;
		PidVitesseD->DeriveErreur = 0;

		PidVitesseD->CommandeProportionelle = 0;
		PidVitesseD->CommandeIntegrale = 0;
		PidVitesseD->CommandeDerive = 0;

		PidVitesseD->Kp = 2000;
		PidVitesseD->Ki = 0;
		PidVitesseD->Kd = 0;

		PidVitesseD->KpArret = 1200;
		PidVitesseD->KiArret = 0;
		PidVitesseD->KdArret = 0;

		PidVitesseD->KpRoule = 2000;
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

		PidVitesseG->Kp = 2000;
		PidVitesseG->Ki = 0;
		PidVitesseG->Kd = 0;

		PidVitesseG->KpArret = 1200;
		PidVitesseG->KiArret = 0;
		PidVitesseG->KdArret = 0;

		PidVitesseG->KpRoule = 2000;
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
		MouvementActuel->PrecisionAngulaire = 2.0 * PI / 180.0; // 2° <=> x rad

		MouvementActuel->Type = 0;
		MouvementActuel->EnAvant = 1;
		MouvementActuel->Etape = 0;
		MouvementActuel->Termine = 0;
		MouvementActuel->TermineEnvoye = 0;
		MouvementActuel->TempsBlocage = 0;

		MouvementActuel->ConfigAsserv = 0;
}
