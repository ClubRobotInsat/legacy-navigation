


// ----------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ----------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: decision.c
// Auteurs              : Nicolas Dos Santos, José María Martin
// Description			: Ce fichier contient tous les élements de prise
//						  de décision du robot. Il ne contient pas l'interface
//						  matérielle avec le port de communication.
//                        Ce fichier est un peu le cerveau du projet,
//                        c'est lui qui analyse les trames reçus et active
//                        les déplacment. Cependant les messages de retour
//                        vers L'IA ne sont pas tous générés depuis ce fichier
//                        Par exemple lorsque un déplacement est terminé, c'est
//                        à l'intérieur du schéma d'asser que le message est envoyé
// Revisions			:
//			o 20070222 Creation du fichier
//
// ----------------------------------------------------------------

#include "p30fxxxx.h"
#include "communication.h"
#include "donnees.h"
#include "fct_asserv.h"
#include "can.h"

		//		TYPE_MSG      VALEUR
		#define ARRETURGENCE	15
		#define NonUtilise_14	14
		#define NonUtilise_13	13
		#define ENVOI_AUTO		12
		#define ASSERV_ON_OFF	11
		#define REGLER_COORD	10
		#define DEBUG			9
		#define LED				8
		#define COMMANDE		7
		#define TERMINE			6
		#define POSITION		5
	  	#define BLOQUE			4
	  	#define PARAM			3
	  	#define RECU			2
		#define IGNORE			1
		#define PING			0

// VARIABLE GLOBALES ----------------------------------------------


	extern MOUVEMENT MouvementActuel;

	extern ETAT Etat;

	extern ASSERV Longitudinal;
	extern ASSERV Angulaire;
	extern ASSERV RoueD;
	extern ASSERV RoueG;

	extern CORRECTEUR PidPositionLongitudinale;
	extern CORRECTEUR PidPositionAngulaire;
	extern CORRECTEUR PidVitesseLongitudinale;
	extern CORRECTEUR PidVitesseAngulaire;

	extern CORRECTEUR PidPositionD;
	extern CORRECTEUR PidPositionG;
	extern CORRECTEUR PidVitesseD;
	extern CORRECTEUR PidVitesseG;

	extern MECANIQUE Mecanique;

// FINS VARIABLE GLOBALES -----------------------------------------

void CANNouveauMessage(MESSAGE * MessageRecu)
{

	if ((MessageRecu->SID & 0x0F) == PING) {
		if (MessageRecu->Nombre_Data == 1 && MessageRecu->Data[0] == 0x55) { // PING-PONG [OK]
			SendPong();
		} else {SendIgnorer();};




	} else if ((MessageRecu->SID & 0x0F) == RECU) {
		//Cool je suis heureux




	} else if ((MessageRecu->SID & 0x0F) == ARRETURGENCE) { //Arret d'urgence
		SendRecu();




	} else if ((MessageRecu->SID & 0x0F) == POSITION) { //Demande des coordonnées (X,Y,Théta) du robot [OK]
		SendPosition();




	} else if ((MessageRecu->SID & 0x0F) == TERMINE) { //Envoi si le déplacement est terminé ou pas
		SendTermine();




	} else if ((MessageRecu->SID & 0x0F) == REGLER_COORD) { //Regle les coordonnees du robot
		if (MessageRecu->Nombre_Data == 6) {//Données : X_low | X_high | Y_low | Y_high | Angle_low | Angle_high
			Etat.X = (float)(MessageRecu->Data[0] + MessageRecu->Data[1]*256) / 1000.0;
			Etat.Y = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256) / 1000.0;
			Etat.DistanceRoueD = 0;
			Etat.DistanceRoueG = 0;
			Etat.AngleBrut = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256) / 1000.0;
			ODOSetDistanceRoue0_N(0.0);
			ODOSetDistanceRoue1_N(0.0);
			ODOAjouterAngle((float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256) / 1000.0);
			MouvementActuel.Type = 0;
			MouvementActuel.Termine = 0;
			MouvementActuel.TermineEnvoye = 0;
			ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
			SendRecu();
		} else { SendIgnorer();};


	} else if ((MessageRecu->SID & 0x0F) == ASSERV_ON_OFF) { //Permet d'activer ou de desactiver l'asservissement
		if (MessageRecu->Nombre_Data == 2) {//Données : Type->L,A,D,G | bool ON_OFF
			if(MessageRecu->Data[0] == 'L')
				Longitudinal.Est_Active = MessageRecu->Data[1];
			else if(MessageRecu->Data[0] == 'A')
				Angulaire.Est_Active = MessageRecu->Data[1];
			else if(MessageRecu->Data[0] == 'D')
				RoueD.Est_Active = MessageRecu->Data[1];
			else if(MessageRecu->Data[0] == 'G')
				RoueG.Est_Active = MessageRecu->Data[1];
			SendRecu();
		} else { SendIgnorer();};


	} else if ((MessageRecu->SID & 0x0F) == LED) { //Allume ou eteind les diode debug [OK]
		TRISEbits.TRISE4 = 0;
		TRISEbits.TRISE5 = 0;
		LATEbits.LATE4 = MessageRecu->Data[0];
		LATEbits.LATE5 = MessageRecu->Data[1];
		SendRecu();


	} else if ((MessageRecu->SID & 0x0F) == DEBUG) { //Envoi au PC les infos pour calibrer les PID
		SendMesure(MessageRecu->Data[0]);

	} else if ((MessageRecu->SID & 0x0F) == ENVOI_AUTO) { //Envoi automatique de la position toutes les 100ms
			Etat.envoi_auto = MessageRecu->Data[0];
		
	} else if ((MessageRecu->SID & 0x0F) == COMMANDE) { //On envoi un ordre de déplacement au robot
		// Format des données : OCT1 = Mode d'asservissement, [OCT2..OCT8] dépend du Mode, voir plus bas

		if (MessageRecu->Data[0] == 0) {//Stop
			if (MessageRecu->Nombre_Data == 1) {//Données : Mode
				MouvementActuel.Type = 0;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 1) {//Translation de DistanceDemande
			if (MessageRecu->Nombre_Data == 3) {//Données : Mode | DistanceDemande_Low(mm) | DistanceDemande_High(mm)
				MouvementActuel.DistanceDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Type = 1;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 2) {//Rotation relative de AngleDemande
			if (MessageRecu->Nombre_Data == 3) {//Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad)
				MouvementActuel.AngleDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Type = 2;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 3) {//Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
			if (MessageRecu->Nombre_Data == 4) {//Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
				MouvementActuel.AngleDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Type = 3;
				MouvementActuel.EnAvant = MessageRecu->Data[3];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 4) {//Pointe vers Xc,Yc (Avant / Arrière)
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
				MouvementActuel.Xc = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Yc = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 4;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 5) {//Va vers Xc, Yc (Decompose, Avant/Arriere)
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
				MouvementActuel.Xc = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Yc = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 5;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				MouvementActuel.Etape = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};
		} else if (MessageRecu->Data[0] == 6) {//VaZy Xc, Yc (Decompose, Avant/Arriere)
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
				MouvementActuel.Xc = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Yc = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 6;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				MouvementActuel.Etape = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 7) {//VaZy Xc, Yc (Avant/Arriere)
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
				MouvementActuel.Xc = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Yc = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 7;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 8) {//Translation à vitesse constante avec angle fixe
			if (MessageRecu->Nombre_Data == 2) {//Données : Mode | Sens
				MouvementActuel.Type = 8;
				MouvementActuel.EnAvant = MessageRecu->Data[1];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 9) {//Rotation à vitesse constante sur place
			if (MessageRecu->Nombre_Data == 2) {//Données : Mode | Sens
				MouvementActuel.Type = 9;
				MouvementActuel.EnAvant = MessageRecu->Data[1];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 10) {//Passer par le point Xc, Yc à vitesse constante
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
				MouvementActuel.Xc = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Yc = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 10;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 11) {//Pivot D
			if (MessageRecu->Nombre_Data == 4) {//Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
				MouvementActuel.AngleDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Type = 11;
				MouvementActuel.EnAvant = MessageRecu->Data[3];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 12) {//Pivot G
			if (MessageRecu->Nombre_Data == 4) {//Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
				MouvementActuel.AngleDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.Type = 12;
				MouvementActuel.EnAvant = MessageRecu->Data[3];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 13) {//Position D&G
			if (MessageRecu->Nombre_Data == 6) {//Données : Mode | DistDroite_Low(mm) | DisDroite_High(mm) | DistGauche_Low(mm) | DistGauche_High(mm) | Sens
				MouvementActuel.DistanceDroiteDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.DistanceGaucheDemande = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.Type = 13;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 14) {//VitesseD&G
			if (MessageRecu->Nombre_Data == 2) {//Donnees : Mode | Sens
				MouvementActuel.Type = 14;
				MouvementActuel.EnAvant = MessageRecu->Data[1];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 15) {//Arret
			if (MessageRecu->Nombre_Data == 1) {//Donnees : Mode
				MouvementActuel.Type = 15;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 16) {//Controle Trajectoire
			if (MessageRecu->Nombre_Data == 2) {//Donnees : Mode | D_low | D_high | A_low | A_high | Sens
				MouvementActuel.Type = 16;
				MouvementActuel.DistanceDemande = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				MouvementActuel.AngleDemande = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				MouvementActuel.EnAvant = MessageRecu->Data[5];
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};
				
		} else if (MessageRecu->Data[0] == 17) {//Commande vitesse long & ang
			if (MessageRecu->Nombre_Data == 5) {//Données : Mode | Vd_low | Vd_high | Va_low | Va_high
				MouvementActuel.Type = 17;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				Longitudinal.VitesseMax = (float)(MessageRecu->Data[2]*256 + MessageRecu->Data[1]) / 1000.0;
				Angulaire.VitesseMax = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
				ActualiserMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				SendRecu();
			} else { SendIgnorer();};

		} else {SendIgnorer();};



/*
Liste des paramètres modifiables :
- (Kp Ki Kd) * 8 : Mode->PID | Type->L,l,A,a,D,d,G,g | Kp, Ki, Kd
- (Vmax, Amax) * 4 : Mode->V | Type->L,A,D,G | | Type->v,a | Vmax, Amax
- (Precision) * 2 : Mode->P | Type->L,A | Precision
- (R, Dd, Dg) : Mode->Meca | Type->R,D,G | R, Dd, Dg
*/


	} else if ((MessageRecu->SID & 0x0F) == PARAM) { //On demande de changer un paramètre

		if (MessageRecu->Data[0] == 0) {// -> (Kp Ki Kd) * 8 : Mode->PID | Type->L,l,A,a,D,d,G,g | Kp, Ki, Kd
			if (MessageRecu->Nombre_Data == 8) { //Données : Mode | Type | Kp | Ki | Kd
				switch(MessageRecu->Data[1])
				{
					case 'L':
						PidPositionLongitudinale.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidPositionLongitudinale.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidPositionLongitudinale.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'l':
						PidVitesseLongitudinale.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidVitesseLongitudinale.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidVitesseLongitudinale.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'A':
						PidPositionAngulaire.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidPositionAngulaire.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidPositionAngulaire.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'a':
						PidVitesseAngulaire.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidVitesseAngulaire.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidVitesseAngulaire.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'D':
						PidPositionD.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidPositionD.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidPositionD.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'd':
						PidVitesseD.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidVitesseD.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidVitesseD.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'G':
						PidPositionG.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidPositionG.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidPositionG.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					case 'g':
						PidVitesseG.Kp = (float)(MessageRecu->Data[2] + MessageRecu->Data[3]*256);
						PidVitesseG.Ki = (float)(MessageRecu->Data[4] + MessageRecu->Data[5]*256);
						PidVitesseG.Kd = (float)(MessageRecu->Data[6] + MessageRecu->Data[7]*256);
						SendRecu();
						break;
					default :
						SendIgnorer();
						break;
				}
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 1) { // -> (Vmax, Amax) * 4 : Mode->V | Type->L,A,D,G | | Type->v,a | Vmax, Amax
			if (MessageRecu->Nombre_Data == 5) {//Données : Mode | Type | VouA | VouA_low | VouA_high
				switch(MessageRecu->Data[1])
				{
					case 'L':
						if(MessageRecu->Data[2] == 'v')
						{
							Longitudinal.VitesseMax = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
							if(Longitudinal.VitesseMax > Mecanique.VitesseLongitudinaleMax)
								Longitudinal.VitesseMax = Mecanique.VitesseLongitudinaleMax;
						}
						else
							Longitudinal.Acceleration = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
						SendRecu();
						break;
					case 'A':
						if(MessageRecu->Data[2] == 'v')
						{
							Angulaire.VitesseMax = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
							if(Angulaire.VitesseMax > Mecanique.VitesseAngulaireMax)
								Angulaire.VitesseMax = Mecanique.VitesseAngulaireMax;
						}
						else
							Angulaire.Acceleration = (float)(MessageRecu->Data[4]*256 + MessageRecu->Data[3]) / 1000.0;
						SendRecu();
						break;
					case 'D':
						if(MessageRecu->Data[2] == 'v')
						{
							RoueD.VitesseMax = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 1000.0;
							if(RoueD.VitesseMax > Mecanique.VitesseLongitudinaleMax)
								RoueD.VitesseMax = Mecanique.VitesseLongitudinaleMax;
						}
						else
							RoueD.Acceleration = (float)(MessageRecu->Data[5]*256 + MessageRecu->Data[4]) / 1000.0;
						SendRecu();
						break;
					case 'G':
						if(MessageRecu->Data[2] == 'v')
						{
							RoueG.VitesseMax = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 1000.0;
							if(RoueG.VitesseMax > Mecanique.VitesseLongitudinaleMax)
								RoueG.VitesseMax = Mecanique.VitesseLongitudinaleMax;
						}
						else
							RoueG.Acceleration = (float)(MessageRecu->Data[5]*256 + MessageRecu->Data[4]) / 1000.0;
						SendRecu();
						break;
					default :
						SendIgnorer();
						break;
				}
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 2) { // -> (Precision) * 2 : Mode->P | Type->L,A | Precision
			if (MessageRecu->Nombre_Data == 4) { // Données : Mode | Type | Precision_low | Precision_High
				switch(MessageRecu->Data[1])
				{
					case 'L':
					case 'D':
					case 'G':
						MouvementActuel.PrecisionLongitudinale = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 1000.0;
						SendRecu();
						break;
					case 'A':
						MouvementActuel.PrecisionAngulaire = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 1000.0;
						SendRecu();
						break;
					default :
						SendIgnorer();
						break;
				}
			} else { SendIgnorer();};

		} else if (MessageRecu->Data[0] == 3) { // -> (R, Dd, Dg) : Mode->Meca | Type->R,D,G | R, Dd, Dg
			if (MessageRecu->Nombre_Data == 4) { //Données : Mode | Type | Val_low | Val_High
				switch(MessageRecu->Data[1])
				{
					case 'R':
						Mecanique.EntreAxe = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 100000.0;
						SendRecu();
						break;
					case 'D':
						Mecanique.DiametreRoueD = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 100000.0;
						ODOCalculerDeltaX();
						SendRecu();
						break;
					case 'G':
						Mecanique.DiametreRoueG = (float)(MessageRecu->Data[3]*256 + MessageRecu->Data[2]) / 100000.0;
						ODOCalculerDeltaX();
						SendRecu();
						break;
					default :
						SendIgnorer();
						break;
				}
			} else { SendIgnorer();};

		}  else {SendIgnorer();};

	} else {SendIgnorer();};

}//


// ----------------------------------------------------------------
// Fonction : Senpong(void)
// But 	    : Envoie un message vers L'IA pour dire Pong
// ----------------------------------------------------------------
void SendPong(void)
{
	MESSAGE Reponse;

	Reponse.SID = PING + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 0xAA;
	CANSendMessage(Reponse, 0);
}//

// ----------------------------------------------------------------
// Fonction : SendTermine(void)
// But 	    : Envoie un message vers L'IA pour dire Terminé
// ----------------------------------------------------------------
void SendTermine(void)
{
	MESSAGE Reponse;

	Reponse.SID = TERMINE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = MouvementActuel.Termine;
	CANSendMessage(Reponse, 0);
}//

void SendBloque(void)
{
	MESSAGE Reponse;

	Reponse.SID = BLOQUE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 1;
	CANSendMessage(Reponse, 0);
}

void SendPosition() {
	int temp_x = (float)(Etat.X * 1000.0); //mm
	int temp_y = (float)(Etat.Y * 1000.0); //mm
	int temp_angle = (float)(Etat.AngleReel * 1000.0); //mrad

	MESSAGE Reponse;

	Reponse.SID = POSITION + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 6;
	Reponse.Data[0] = temp_x & 0x00FF;//X LSB
	Reponse.Data[1] = (temp_x>>8) & 0x00FF;//X MSB
	Reponse.Data[2] = temp_y & 0x00FF;//Y LSB
	Reponse.Data[3] = (temp_y>>8) & 0x00FF;//Y MSB
	Reponse.Data[4] = (temp_angle) & 0x00FF;//Theta LSB
	Reponse.Data[5] = (temp_angle>>8) & 0x00FF;//Theta MSB en Degrés
	CANSendMessage(Reponse, 0);
}

void SendMesure(unsigned char Type) {

	int temp_D;
	int temp_Vc;
	int temp_V;

	switch (Type)
	{
		case 0: //Longitudinal
			temp_D = Etat.DistanceTotale * 1000.0; //mm
			temp_Vc = Longitudinal.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseLongitudinale * 1000.0; //mm/s
			break;

		case 1: //Angulaire
			temp_D = Etat.AngleBrut * 1000.0; //mrad
			temp_Vc = Angulaire.VitesseConsigne * 1000.0; //mrad/s
			temp_V = Etat.VitesseAngulaire * 1000.0; //mrad/s
			break;

		case 2: //D
			temp_D = Etat.DistanceRoueD * 1000.0; //mm
			temp_Vc = RoueD.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseRoueD * 1000.0; //mm/s
			break;

		case 3: //G
			temp_D = Etat.DistanceRoueG * 1000.0; //mm
			temp_Vc = RoueG.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseRoueG * 1000.0; //mm/s
			break;
	}

	MESSAGE Reponse;

	Reponse.SID = DEBUG + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 8;
	Reponse.Data[0] = Type;
	Reponse.Data[1] = temp_D & 0x00FF;//D LSB
	Reponse.Data[2] = (temp_D>>8) & 0x00FF;//D MSB
	Reponse.Data[3] = temp_Vc & 0x00FF;//Vc LSB
	Reponse.Data[4] = (temp_Vc>>8) & 0x00FF;//Vc MSB
	Reponse.Data[5] = (temp_V) & 0x00FF;//V LSB
	Reponse.Data[6] = (temp_V>>8) & 0x00FF;//V MSB
	Reponse.Data[7]= (char)(fabs((float)(PidVitesseLongitudinale.Commande / 2.0)));
	CANSendMessage(Reponse, 0);
}

// ----------------------------------------------------------------
// Fonction : SendTermine(void)
// But 	    : Envoie un message vers L'IA pour dire Reçu
// ----------------------------------------------------------------

void SendRecu(void)
{
	MESSAGE Reponse;

	Reponse.SID = RECU + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 'R';
	CANSendMessage(Reponse, 0);
}//
// ----------------------------------------------------------------
// Fonction : SendTermine(void)
// But 	    : Envoie un message vers L'IA pour dire Reçu
// ----------------------------------------------------------------

void SendIgnorer(void)
{
	MESSAGE Reponse;

	Reponse.SID = IGNORE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 'I';
	CANSendMessage(Reponse, 0);

}//
