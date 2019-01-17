#include "p33fxxxx.h"

#include "..\..\..\..\Librairies Périphériques\ECAN dsPIC33F\Librairie\ecan.h"
#include "..\..\..\..\Librairies Périphériques\QEI dsPIC33F\Librairie\qei.h"
#include "..\..\..\..\Librairies Périphériques\TIMER dsPIC33F\Librairie\timer.h"

#include "communication.h"
#include "variables.h"
#include "fct_asserv.h"

	//		TYPE_MSG      VALEUR
	#define RESET			15
	#define GET_VARIABLE	13
	#define MESURE			12
	#define TERMINE			11
	#define POSITION		10
	#define BLOQUE			9
	#define LED				8
	#define ENVOI_AUTO		7
	#define ASSERV_ON_OFF	6
	#define ARRETURGENCE	5
	#define COMMANDE		4
	#define DEFINIR_COORD	3
	#define SET_VARIABLE	2
	#define ACK				1
	#define PING			0

// VARIABLE GLOBALES ----------------------------------------------


	extern MOUVEMENT MouvementActuel;

	extern ETAT Etat;

	extern CARTE Carte;

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

// FONCTION DE TRAITEMENT DES DONNEES -----------------------------

	// Extrait d'un tableau de data reçues, un int 16 bits à partir de 2 octets (méthode Little indian)
	int RecomposerInt(unsigned char donnees[], int indice)
	{
		return donnees[indice] + donnees[indice + 1] * 256;
	}

	// Ecrit dans un tableau de data l'octet de poid faible puis l'octet de poid fort d'un int 16 bits
	void DecomposerInt(int Valeur, unsigned char donnees[], int indice)
	{
		donnees[indice] = Valeur&0xFF;
		donnees[indice + 1] = (Valeur&0xFF00)>>8;
	}

	// Extrait un float d'un tableau de data, en considérant que le float a été transmis via un entier de 16 bits, avec un facteur 1000
	// Exemple : 0.236 (float) <=> 236 (int 16 bits)
	// On peut donc transmettre un float compris entre -32.768 et +32.768 <=> int 16 bits entre -32768 et +32768
	float RecomposerMiliFloat(unsigned char donnees[], int indice)
	{
		return (float)(RecomposerInt(donnees, indice) / 1000.0);
	}

	// Ecrit dans un tableau de data l'octet de poid faible et l'octet de poid fort d'un float transformé en int16, avec un facteur 1000 (voir ci dessus)
	void DecomposerMiliFloat(float Valeur, unsigned char donnees[], int indice)
	{
		DecomposerInt((int)(Valeur * 1000.0), donnees, indice);
	}

	// Extrait d'un tableau de data, un vrai float à partir de 4 octets
	float RecomposerFloat(unsigned char donnees[], int indice)
	{
		int i = 0;

		float Resultat;
		unsigned char * pResultat = (unsigned char *) &Resultat;

		for(i = 0; i < 4; i++)
			*(pResultat++) = donnees[indice + i];

		return Resultat;
	}

	// Ecrit dans un tableau de data, les 4 octets d'un float 32 bits.
	void DecomposerFloat(float Valeur, unsigned char donnees[], int indice)
	{
		int i = 0;

		unsigned char * pValeur = (unsigned char *) &Valeur;

		for(i = 0; i < 4; i++)
			donnees[indice + i] = *(pValeur++);
	}

// FIN DE FONCTION DE TRAITEMENT DES DONNEES ----------------------

void TraiterMessage(MESSAGE * MessageRecu)
{
	char Ack = 'R'; // on suppose que le message est correct, et donc que l'acquittement sera 'R' pour "Reçu". On le passera à 'I' si ce n'est pas le cas.
	
	float Valeur;

	switch(MessageRecu->SID & 0x0F)
	{
		case PING:
			if (MessageRecu->Nombre_Data == 1 && MessageRecu->Data[0] == 0x55) // PING-PONG
				SendPong();
			else
				Ack = 'I';
		break;


		case ARRETURGENCE: //Arret d'urgence
			MouvementActuel.Type = 15;
			MouvementActuel.Termine = 0;
			MouvementActuel.TermineEnvoye = 0;
			InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
		break;


		case POSITION: //Demande des coordonnées (X,Y,Théta) du robot
			SendPosition();
		break;


		case TERMINE: //Envoi si le déplacement est terminé ou pas
			SendTermine();
		break;


		case DEFINIR_COORD: //Regle les coordonnees du robot
			if (MessageRecu->Nombre_Data == 6) //Données : X_low | X_high | Y_low | Y_high | Angle_low | Angle_high
			{
				// On arrete la boucle dasserv
				T1SetStatus(0);
				
				// Données reçues
				float X_Recu = RecomposerMiliFloat(MessageRecu->Data, 0);
				float Y_Recu = RecomposerMiliFloat(MessageRecu->Data, 2);
				float A_Recu = RecomposerMiliFloat(MessageRecu->Data, 4);

				// On arrète le robot
				MouvementActuel.Type = 0;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);

				// On désactive l'asservissement
				Longitudinal.Est_Active = 0;
				Angulaire.Est_Active = 0;
				RoueD.Est_Active = 0;
				RoueG.Est_Active = 0;

				VARInitialize(&Carte, &Mecanique, &Longitudinal, &Angulaire, &Etat, &PidPositionLongitudinale, &PidPositionAngulaire, &PidVitesseLongitudinale, &PidVitesseAngulaire, &RoueD, &RoueG, &PidPositionD, &PidPositionG, &PidVitesseD, &PidVitesseG, &MouvementActuel, 0); //Initialisation de toutes les variables du programme d'asservissement

				//On réinitialise les encodeurs
				QEI1SetDistanceRoue(0.0);
				QEI2SetDistanceRoue(0.0);

				// On réécrit l'état du robot
				Etat.X = X_Recu;
				Etat.Y = Y_Recu;
				Etat.AngleBrut = A_Recu;
				Etat.AngleReel = Mod2Pi(A_Recu);

				//On ajoute un décalage en nombre de pas sur les roues codeuses pour reproduire l'angle demandé (angle <=> différence de nbr de pas sur les roues codeuses)
				QEI1SetDistanceRoue(Mecanique.EntreAxe * A_Recu / 2.0);
				QEI2SetDistanceRoue(-Mecanique.EntreAxe * A_Recu / 2.0);

				// On résactive l'asservissement
				Longitudinal.Est_Active = 1;
				Angulaire.Est_Active = 1;
				RoueD.Est_Active = 1;
				RoueG.Est_Active = 1;

				// On arrète à nouveau le robot (recalcul les nouvelles consignes d'asservissement)
				MouvementActuel.Type = 0;
				MouvementActuel.Termine = 0;
				MouvementActuel.TermineEnvoye = 0;
				InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
				
				// On relance la boucle dasserv
				T1SetStatus(1);
			}
			else
				Ack = 'I';
		break;


		case ASSERV_ON_OFF: //Permet d'activer ou de desactiver l'asservissement
			if (MessageRecu->Nombre_Data == 2) //Données : Type->L,A,D,G | bool ON_OFF
			{
				if(MessageRecu->Data[0] == 'L')
					Longitudinal.Est_Active = MessageRecu->Data[1];
				else if(MessageRecu->Data[0] == 'A')
					Angulaire.Est_Active = MessageRecu->Data[1];
				else if(MessageRecu->Data[0] == 'D')
					RoueD.Est_Active = MessageRecu->Data[1];
				else if(MessageRecu->Data[0] == 'G')
					RoueG.Est_Active = MessageRecu->Data[1];
			}
			else
				Ack = 'I';
		break;


		case LED: //Allume ou eteind les diode debug
			LATBbits.LATB5 = MessageRecu->Data[0];
			LATBbits.LATB6 = MessageRecu->Data[1];
		break;


		case MESURE: //Envoi au PC les mesures de l'état pour calibrer les PID
			SendMesure(MessageRecu->Data[0]);
		break;


		case ENVOI_AUTO: //Envoi automatique de la position toutes les 100ms
			Etat.EnvoiAuto = MessageRecu->Data[0];
			T2SetStatus(MessageRecu->Data[0]);
		break;


		case COMMANDE: //On envoi un ordre de déplacement au robot

		// Format des données : OCT1 = Mode d'asservissement, [OCT2..OCT8] dépend du Mode, voir plus bas

			switch(MessageRecu->Data[0])
			{
				case 0: // Stop
					if (MessageRecu->Nombre_Data == 1) //Données : Mode
					{
						MouvementActuel.Type = 0;
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 1: //Translation de DistanceDemande
					if (MessageRecu->Nombre_Data == 3) //Données : Mode | DistanceDemande_Low(mm) | DistanceDemande_High(mm)
					{
						MouvementActuel.DistanceDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Type = 1;
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 2: //Rotation relative de AngleDemande
					if (MessageRecu->Nombre_Data == 3) //Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad)
					{
						MouvementActuel.AngleDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Type = 2;
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
					if (MessageRecu->Nombre_Data == 4) //Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
					{
						MouvementActuel.AngleDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Type = 3;
						MouvementActuel.EnAvant = MessageRecu->Data[3];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 4: //Pointe vers Xc,Yc (Avant / Arrière)
					if (MessageRecu->Nombre_Data == 6) //Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
					{
						MouvementActuel.Xc = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Yc = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 4;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
					if (MessageRecu->Nombre_Data == 6) //Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
					{
						MouvementActuel.Xc = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Yc = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 5;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						MouvementActuel.Etape = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 6: //VaZy Xc, Yc (Decompose, Avant/Arriere)
					if (MessageRecu->Nombre_Data == 6) //Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
					{
						MouvementActuel.Xc = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Yc = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 6;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						MouvementActuel.Etape = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 7: //VaZy Xc, Yc (Avant/Arriere)
					if (MessageRecu->Nombre_Data == 6) //Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | Sens
					{
						MouvementActuel.Xc = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Yc = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 7;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 8: //Translation à vitesse constante avec angle fixe
					if (MessageRecu->Nombre_Data == 4) //Données : Mode | Vitesse |Sens
					{
						MouvementActuel.Type = 8;
						MouvementActuel.EnAvant = MessageRecu->Data[3];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						Longitudinal.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 9: //Rotation à vitesse constante sur place
					if (MessageRecu->Nombre_Data == 4) //Données : Mode | Vitesse |Sens
					{
						MouvementActuel.Type = 9;
						MouvementActuel.EnAvant = MessageRecu->Data[3];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						Angulaire.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 10: //Passer par le point Xc, Yc à vitesse constante
					if (MessageRecu->Nombre_Data == 8) //Données : Mode | Xc_Low(mm) | Xc_High(mm) | Yc_Low(mm) | Yc_High(mm) | V_low(mm/s) | V_high(mm/s) | Sens
					{
						MouvementActuel.Xc = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Yc = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 10;
						Longitudinal.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 5);
						MouvementActuel.EnAvant = MessageRecu->Data[7];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 11: //Pivot D
					if (MessageRecu->Nombre_Data == 4) //Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
					{
						MouvementActuel.AngleDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Type = 11;
						MouvementActuel.EnAvant = MessageRecu->Data[3];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 12: //Pivot G
					if (MessageRecu->Nombre_Data == 4) //Données : Mode | AngleDemande_Low(mrad) | AngleDemande_High(mrad) | Sens
					{
						MouvementActuel.AngleDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.Type = 12;
						MouvementActuel.EnAvant = MessageRecu->Data[3];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 13: //Position D&G
					if (MessageRecu->Nombre_Data == 6) //Données : Mode | DistDroite_Low(mm) | DisDroite_High(mm) | DistGauche_Low(mm) | DistGauche_High(mm) | Sens
					{
						MouvementActuel.DistanceDroiteDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						MouvementActuel.DistanceGaucheDemande = RecomposerMiliFloat(MessageRecu->Data, 3);
						MouvementActuel.Type = 13;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 14: //VitesseD&G
					if (MessageRecu->Nombre_Data == 6) //Donnees : Mode | Vd_low(mm/s) | Vd_high(mm/s) | Vg_low(mm/s) | Vg_high(mm/s) | Sens
					{
						MouvementActuel.Type = 14;
						MouvementActuel.EnAvant = MessageRecu->Data[5];
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						RoueD.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						RoueG.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 3);
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 15: //Arret
					if (MessageRecu->Nombre_Data == 1) //Donnees : Mode
					{
						MouvementActuel.Type = 15;
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 17: //Commande vitesse long & ang
					if (MessageRecu->Nombre_Data == 5) //Données : Mode | Vd_low (mm/s) | Vd_high (mm/s) | Va_low (mm/s) | Va_high (mm/s)
					{
						MouvementActuel.Type = 17;
						MouvementActuel.Termine = 0;
						MouvementActuel.TermineEnvoye = 0;
						Longitudinal.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 1);
						Angulaire.VitesseDemande = RecomposerMiliFloat(MessageRecu->Data, 3);
						InitierOuActualiserConsigneMouvement(&MouvementActuel, &Etat, &Longitudinal, &Angulaire, &RoueD, &RoueG, &Mecanique);
					}
					else
						Ack = 'I';
				break;

				case 18: //Regler tension moteur
					if (MessageRecu->Nombre_Data == 5) //Données : Mode | Vd_low (mm/s) | Vd_high (mm/s) | Va_low (mm/s) | Va_high (mm/s)
					{
						MouvementActuel.Type = 18;
						MouvementActuel.AngleDemande = 4*(float)(MessageRecu->Data[0]);
					}
					else
						Ack = 'I';
				break;

				default:
					Ack = 'I';
				break;
			}
		break;

		case SET_VARIABLE: // Permet de régler la valeur de toutes les variables de robot, indépendament les unes des autres

			// Format des données : OCT1 = ID de la variable à modifier, [OCT2..OCT3] valeur de la variable en MiliFloat

			Valeur = RecomposerFloat(MessageRecu->Data, 1);

			switch(MessageRecu->Data[0])
			{
				case Carte_Te:
					Carte.TeOdometrie = Valeur;
					T5Initialize(Carte.TeOdometrie, Carte.Fcy);
				break;

				case Mecanique_EntreAxe:
					Mecanique.EntreAxe = Valeur;
				break;

				case Mecanique_DiametreRoueD:
					Mecanique.DiametreRoueD = Valeur;
					QEI1ComputeDeltaX(Mecanique.NombrePasD, Mecanique.DiametreRoueD);
				break;

				case Mecanique_DiametreRoueG:
					Mecanique.DiametreRoueG = Valeur;
					QEI2ComputeDeltaX(Mecanique.NombrePasG, Mecanique.DiametreRoueG);
				break;

				case Mecanique_PuissanceMaxD:
					Mecanique.PuissanceMaxD = Valeur;
				break;
 
				case Mecanique_PuissanceMaxG:
					Mecanique.PuissanceMaxG = Valeur;
				break;

				case Mecanique_PuissanceMaxLongitudinal:
					Mecanique.PuissanceMaxLongitudinal = Valeur;
				break;

				case Mecanique_PuissanceMaxAngulaire:
					Mecanique.PuissanceMaxAngulaire = Valeur;
				break;

				case Mecanique_VitesseLongitudinaleMax:
					Mecanique.VitesseLongitudinaleMax = Valeur;
				break;

				case Mecanique_VitesseAngulaireMax:
					Mecanique.VitesseAngulaireMax = Valeur;
				break;

				case Mecanique_VitesseRoueDMax:
					Mecanique.VitesseRoueDMax = Valeur;
				break;

				case Mecanique_VitesseRoueGMax:
					Mecanique.VitesseRoueGMax = Valeur;
				break;

				case Mecanique_NombrePasD:
					Mecanique.NombrePasD = Valeur;
					QEI1ComputeDeltaX(Mecanique.NombrePasD, Mecanique.DiametreRoueD);
				break;

				case Mecanique_NombrePasG: 
					Mecanique.NombrePasG = Valeur; 
					QEI1ComputeDeltaX(Mecanique.NombrePasG, Mecanique.DiametreRoueG);
				break;

				case Longitudinal_VitesseMax:
					Longitudinal.VitesseMax = Valeur;
				break;

				case Longitudinal_Acceleration:
					Longitudinal.Acceleration = Valeur;
				break;
				
				case Longitudinal_Decceleration:
					Longitudinal.Decceleration = Valeur;
				break;

				case Angulaire_VitesseMax:
					Angulaire.VitesseMax = Valeur;
				break;
				
				case Angulaire_Acceleration:
					Angulaire.Acceleration = Valeur;
				break;
				
				case Angulaire_Decceleration:
					Angulaire.Decceleration = Valeur;
				break;

				case RoueD_VitesseMax:
					RoueD.VitesseMax = Valeur;
				break;

				case RoueD_Acceleration:
					RoueD.Acceleration = Valeur;
				break;
				
				case RoueD_Decceleration:
					RoueD.Decceleration = Valeur;
				break;

				case RoueG_VitesseMax:
					RoueG.VitesseMax = Valeur;
				break;

				case RoueG_Acceleration:
					RoueG.Acceleration = Valeur;
				break;
				
				case RoueG_Decceleration:
					RoueG.Decceleration = Valeur;
				break;

				case PidPositionLongitudinale_KpArret:
					PidPositionLongitudinale.KpArret = Valeur;
				break;

				case PidPositionLongitudinale_KiArret:
					PidPositionLongitudinale.KiArret = Valeur;
				break;

				case PidPositionLongitudinale_KdArret:
					PidPositionLongitudinale.KdArret = Valeur;
				break;

				case PidPositionLongitudinale_KpRoule:
					PidPositionLongitudinale.KpRoule = Valeur;
				break;

				case PidPositionLongitudinale_KiRoule:
					PidPositionLongitudinale.KiRoule = Valeur;
				break;

				case PidPositionLongitudinale_KdRoule:
					PidPositionLongitudinale.KdRoule = Valeur;
				break;

				case PidPositionAngulaire_KpArret:
					PidPositionAngulaire.KpArret = Valeur;
				break;

				case PidPositionAngulaire_KiArret:
					PidPositionAngulaire.KiArret = Valeur;
				break;

				case PidPositionAngulaire_KdArret:
					PidPositionAngulaire.KdArret = Valeur;
				break;

				case PidPositionAngulaire_KpRoule:
					PidPositionAngulaire.KpRoule = Valeur;
				break;

				case PidPositionAngulaire_KiRoule: 
					PidPositionAngulaire.KiRoule = Valeur;
				break;

				case PidPositionAngulaire_KdRoule:
					PidPositionAngulaire.KdRoule = Valeur;
				break;

				case PidVitesseLongitudinale_KpArret:
					PidVitesseLongitudinale.KpArret = Valeur;
				break;

				case PidVitesseLongitudinale_KiArret:
					PidVitesseLongitudinale.KiArret = Valeur;
				break;

				case PidVitesseLongitudinale_KdArret:
					PidVitesseLongitudinale.KdArret = Valeur;
				break;

				case PidVitesseLongitudinale_KpRoule:
					PidVitesseLongitudinale.KpRoule = Valeur;
				break;

				case PidVitesseLongitudinale_KiRoule:
					PidVitesseLongitudinale.KiRoule = Valeur;
				break;

				case PidVitesseLongitudinale_KdRoule:
					PidVitesseLongitudinale.KdRoule = Valeur;
				break;

				case PidVitesseAngulaire_KpArret:
					PidVitesseAngulaire.KpArret = Valeur;
				break;

				case PidVitesseAngulaire_KiArret:
					PidVitesseAngulaire.KiArret = Valeur;
				break;

				case PidVitesseAngulaire_KdArret:
					PidVitesseAngulaire.KdArret = Valeur;
				break;

				case PidVitesseAngulaire_KpRoule:
					PidVitesseAngulaire.KpRoule = Valeur;
				break;

				case PidVitesseAngulaire_KiRoule:
					PidVitesseAngulaire.KiRoule = Valeur;
				break;

				case PidVitesseAngulaire_KdRoule:
					PidVitesseAngulaire.KdRoule = Valeur;
				break;

				case PidPositionD_KpArret: 
					PidPositionD.KpArret = Valeur; 
				break;
				
				case PidPositionD_KiArret: 
					PidPositionD.KiArret = Valeur;
				break;
				
				case PidPositionD_KdArret: 
					PidPositionD.KdArret = Valeur; 
				break;
				
				case PidPositionD_KpRoule: 
					PidPositionD.KpRoule = Valeur;
				break;
				
				case PidPositionD_KiRoule: 
					PidPositionD.KiRoule = Valeur;
				break;
				
				case PidPositionD_KdRoule: 
					PidPositionD.KdRoule = Valeur; 
				break;
				
				case PidPositionG_KpArret: 
					PidPositionG.KpArret = Valeur; 
				break;
				
				case PidPositionG_KiArret: 
					PidPositionG.KiArret = Valeur; 
				break;
				
				case PidPositionG_KdArret: 
					PidPositionG.KdArret = Valeur; 
				break;
				
				case PidPositionG_KpRoule: 
					PidPositionG.KpRoule = Valeur; 
				break;
				
				case PidPositionG_KiRoule: 
					PidPositionG.KiRoule = Valeur; 
				break;
				
				case PidPositionG_KdRoule: 
					PidPositionG.KdRoule = Valeur;
				break;
				
				case PidVitesseD_KpArret: 
					PidVitesseD.KpArret = Valeur; 
				break;
				
				case PidVitesseD_KiArret: 
					PidVitesseD.KiArret = Valeur; 
				break;
				
				case PidVitesseD_KdArret: 
					PidVitesseD.KdArret = Valeur; 
				break;
				
				case PidVitesseD_KpRoule: 
					PidVitesseD.KpRoule = Valeur; 
				break;
				
				case PidVitesseD_KiRoule: 
					PidVitesseD.KiRoule = Valeur; 
				break;
				
				case PidVitesseD_KdRoule: 
					PidVitesseD.KdRoule = Valeur; 
				break;
				
				case PidVitesseG_KpArret: 
					PidVitesseG.KpArret = Valeur; 
				break;
				
				case PidVitesseG_KiArret: 
					PidVitesseG.KiArret = Valeur; 
				break;
				
				case PidVitesseG_KdArret: 
					PidVitesseG.KdArret = Valeur; 
				break;
				
				case PidVitesseG_KpRoule: 
					PidVitesseG.KpRoule = Valeur; 
				break;
				
				case PidVitesseG_KiRoule: 
					PidVitesseG.KiRoule = Valeur; 
				break;
				
				case PidVitesseG_KdRoule: 
					PidVitesseG.KdRoule = Valeur;
				break;
				
				case MouvementActuel_PrecisionLongitudinale: 
					MouvementActuel.PrecisionLongitudinale = Valeur; 
				break;
				
				case MouvementActuel_PrecisionAngulaire: 
					MouvementActuel.PrecisionAngulaire = Valeur; 
				break;
				
				default:
					Ack = 'I';
				break;
			}
		break;

		case GET_VARIABLE: // Permet de lire la valeur de toutes les variables de robot, indépendament les unes des autres

			// Format des données : OCT1 = ID de la variable à lire

			switch(MessageRecu->Data[0])
			{
				case Carte_Fcy :	Valeur = Carte.Fcy;	break;
				case Carte_Tcy :	Valeur = Carte.Tcy;	break;
				case Carte_Te :	Valeur = Carte.TeOdometrie;	break;
				case Carte_Fe :	Valeur = Carte.Fe;	break;
				case Carte_Fpwm :	Valeur = Carte.Fpwm;	break;
				case Carte_Tpwm :	Valeur = Carte.Tpwm;	break;
				case Mecanique_EntreAxe :	Valeur = Mecanique.EntreAxe;	break;
				case Mecanique_DiametreRoueD :	Valeur = Mecanique.DiametreRoueD;	break;
				case Mecanique_DiametreRoueG :	Valeur = Mecanique.DiametreRoueG;	break;
				case Mecanique_PuissanceMoteurD :	Valeur = Mecanique.PuissanceMoteurD;	break;
				case Mecanique_PuissanceMoteurG :	Valeur = Mecanique.PuissanceMoteurG;	break;
				case Mecanique_PuissanceMaxD :	Valeur = Mecanique.PuissanceMaxD;	break;
				case Mecanique_PuissanceMaxG :	Valeur = Mecanique.PuissanceMaxG;	break;
				case Mecanique_PuissanceMaxLongitudinal :	Valeur = Mecanique.PuissanceMaxLongitudinal;	break;
				case Mecanique_PuissanceMaxAngulaire :	Valeur = Mecanique.PuissanceMaxAngulaire;	break;
				case Mecanique_VitesseLongitudinaleMax :	Valeur = Mecanique.VitesseLongitudinaleMax;	break;
				case Mecanique_VitesseAngulaireMax :	Valeur = Mecanique.VitesseAngulaireMax;	break;
				case Mecanique_VitesseRoueDMax :	Valeur = Mecanique.VitesseRoueDMax;	break;
				case Mecanique_VitesseRoueGMax :	Valeur = Mecanique.VitesseRoueGMax;	break;
				case Mecanique_NombrePasD :	Valeur = Mecanique.NombrePasD;	break;
				case Mecanique_NombrePasG :	Valeur = Mecanique.NombrePasG;	break;
				case Etat_DistanceTotale :	Valeur = Etat.DistanceTotale;	break;
				case Etat_DistanceRoueD :	Valeur = Etat.DistanceRoueD;	break;
				case Etat_DistanceRoueG :	Valeur = Etat.DistanceRoueG;	break;
				case Etat_AngleBrut :	Valeur = Etat.AngleBrut;	break;
				case Etat_AngleReel :	Valeur = Etat.AngleReel;	break;
				case Etat_VitesseLongitudinale :	Valeur = Etat.VitesseLongitudinale;	break;
				case Etat_VitesseAngulaire :	Valeur = Etat.VitesseAngulaire;	break;
				case Etat_VitesseRoueD :	Valeur = Etat.VitesseRoueD;	break;
				case Etat_VitesseRoueG :	Valeur = Etat.VitesseRoueG;	break;
				case Etat_X :	Valeur = Etat.X;	break;
				case Etat_Y :	Valeur = Etat.Y;	break;
				case Etat_EnvoiAuto :	Valeur = Etat.EnvoiAuto;	break;
				case Longitudinal_Desire :	Valeur = Longitudinal.Desire;	break;
				case Longitudinal_Consigne :	Valeur = Longitudinal.Consigne;	break;
				case Longitudinal_VitesseMax :	Valeur = Longitudinal.VitesseMax;	break;
				case Longitudinal_VitesseDemande :	Valeur = Longitudinal.VitesseDemande;	break;
				case Longitudinal_VitesseConsigne :	Valeur = Longitudinal.VitesseConsigne;	break;
				case Longitudinal_VitesseConsignePrecedente :	Valeur = Longitudinal.VitesseConsignePrecedente;	break;
				case Longitudinal_Acceleration :	Valeur = Longitudinal.Acceleration;	break;
				case Longitudinal_Decceleration :	Valeur = Longitudinal.Decceleration;	break;
				case Longitudinal_Est_Active :	Valeur = Longitudinal.Est_Active;	break;
				case Angulaire_Desire :	Valeur = Angulaire.Desire;	break;
				case Angulaire_Consigne :	Valeur = Angulaire.Consigne;	break;
				case Angulaire_VitesseMax :	Valeur = Angulaire.VitesseMax;	break;
				case Angulaire_VitesseDemande :	Valeur = Angulaire.VitesseDemande;	break;
				case Angulaire_VitesseConsigne :	Valeur = Angulaire.VitesseConsigne;	break;
				case Angulaire_VitesseConsignePrecedente :	Valeur = Angulaire.VitesseConsignePrecedente;	break;
				case Angulaire_Acceleration :	Valeur = Angulaire.Acceleration;	break;
				case Angulaire_Decceleration :	Valeur = Angulaire.Decceleration;	break;
				case Angulaire_Est_Active :	Valeur = Angulaire.Est_Active;	break;
				case RoueD_Desire :	Valeur = RoueD.Desire;	break;
				case RoueD_Consigne :	Valeur = RoueD.Consigne;	break;
				case RoueD_VitesseMax :	Valeur = RoueD.VitesseMax;	break;
				case RoueD_VitesseDemande :	Valeur = RoueD.VitesseDemande;	break;
				case RoueD_VitesseConsigne :	Valeur = RoueD.VitesseConsigne;	break;
				case RoueD_VitesseConsignePrecedente :	Valeur = RoueD.VitesseConsignePrecedente;	break;
				case RoueD_Acceleration :	Valeur = RoueD.Acceleration;	break;
				case RoueD_Decceleration :	Valeur = RoueD.Decceleration;	break;
				case RoueD_Est_Active :	Valeur = RoueD.Est_Active;	break;
				case RoueG_Desire :	Valeur = RoueG.Desire;	break;
				case RoueG_Consigne :	Valeur = RoueG.Consigne;	break;
				case RoueG_VitesseMax :	Valeur = RoueG.VitesseMax;	break;
				case RoueG_VitesseDemande :	Valeur = RoueG.VitesseDemande;	break;
				case RoueG_VitesseConsigne :	Valeur = RoueG.VitesseConsigne;	break;
				case RoueG_VitesseConsignePrecedente :	Valeur = RoueG.VitesseConsignePrecedente;	break;
				case RoueG_Acceleration :	Valeur = RoueG.Acceleration;	break;
				case RoueG_Decceleration :	Valeur = RoueG.Decceleration;	break;
				case RoueG_Est_Active :	Valeur = RoueG.Est_Active;	break;
				case PidPositionLongitudinale_Erreur :	Valeur = PidPositionLongitudinale.Erreur;	break;
				case PidPositionLongitudinale_ErreurPrecedente :	Valeur = PidPositionLongitudinale.ErreurPrecedente;	break;
				case PidPositionLongitudinale_IntegraleErreur :	Valeur = PidPositionLongitudinale.IntegraleErreur;	break;
				case PidPositionLongitudinale_DeriveErreur :	Valeur = PidPositionLongitudinale.DeriveErreur;	break;
				case PidPositionLongitudinale_CommandeProportionelle :	Valeur = PidPositionLongitudinale.CommandeProportionelle;	break;
				case PidPositionLongitudinale_CommandeIntegrale :	Valeur = PidPositionLongitudinale.CommandeIntegrale;	break;
				case PidPositionLongitudinale_CommandeDerive :	Valeur = PidPositionLongitudinale.CommandeDerive;	break;
				case PidPositionLongitudinale_Kp :	Valeur = PidPositionLongitudinale.Kp;	break;
				case PidPositionLongitudinale_Ki :	Valeur = PidPositionLongitudinale.Ki;	break;
				case PidPositionLongitudinale_Kd :	Valeur = PidPositionLongitudinale.Kd;	break;
				case PidPositionLongitudinale_KpArret :	Valeur = PidPositionLongitudinale.KpArret;	break;
				case PidPositionLongitudinale_KiArret :	Valeur = PidPositionLongitudinale.KiArret;	break;
				case PidPositionLongitudinale_KdArret :	Valeur = PidPositionLongitudinale.KdArret;	break;
				case PidPositionLongitudinale_KpRoule :	Valeur = PidPositionLongitudinale.KpRoule;	break;
				case PidPositionLongitudinale_KiRoule :	Valeur = PidPositionLongitudinale.KiRoule;	break;
				case PidPositionLongitudinale_KdRoule :	Valeur = PidPositionLongitudinale.KdRoule;	break;
				case PidPositionLongitudinale_Commande :	Valeur = PidPositionLongitudinale.Commande;	break;
				case PidPositionAngulaire_Erreur :	Valeur = PidPositionAngulaire.Erreur;	break;
				case PidPositionAngulaire_ErreurPrecedente :	Valeur = PidPositionAngulaire.ErreurPrecedente;	break;
				case PidPositionAngulaire_IntegraleErreur :	Valeur = PidPositionAngulaire.IntegraleErreur;	break;
				case PidPositionAngulaire_DeriveErreur :	Valeur = PidPositionAngulaire.DeriveErreur;	break;
				case PidPositionAngulaire_CommandeProportionelle :	Valeur = PidPositionAngulaire.CommandeProportionelle;	break;
				case PidPositionAngulaire_CommandeIntegrale :	Valeur = PidPositionAngulaire.CommandeIntegrale;	break;
				case PidPositionAngulaire_CommandeDerive :	Valeur = PidPositionAngulaire.CommandeDerive;	break;
				case PidPositionAngulaire_Kp :	Valeur = PidPositionAngulaire.Kp;	break;
				case PidPositionAngulaire_Ki :	Valeur = PidPositionAngulaire.Ki;	break;
				case PidPositionAngulaire_Kd :	Valeur = PidPositionAngulaire.Kd;	break;
				case PidPositionAngulaire_KpArret :	Valeur = PidPositionAngulaire.KpArret;	break;
				case PidPositionAngulaire_KiArret :	Valeur = PidPositionAngulaire.KiArret;	break;
				case PidPositionAngulaire_KdArret :	Valeur = PidPositionAngulaire.KdArret;	break;
				case PidPositionAngulaire_KpRoule :	Valeur = PidPositionAngulaire.KpRoule;	break;
				case PidPositionAngulaire_KiRoule :	Valeur = PidPositionAngulaire.KiRoule;	break;
				case PidPositionAngulaire_KdRoule :	Valeur = PidPositionAngulaire.KdRoule;	break;
				case PidPositionAngulaire_Commande :	Valeur = PidPositionAngulaire.Commande;	break;
				case PidVitesseLongitudinale_Erreur :	Valeur = PidVitesseLongitudinale.Erreur;	break;
				case PidVitesseLongitudinale_ErreurPrecedente :	Valeur = PidVitesseLongitudinale.ErreurPrecedente;	break;
				case PidVitesseLongitudinale_IntegraleErreur :	Valeur = PidVitesseLongitudinale.IntegraleErreur;	break;
				case PidVitesseLongitudinale_DeriveErreur :	Valeur = PidVitesseLongitudinale.DeriveErreur;	break;
				case PidVitesseLongitudinale_CommandeProportionelle :	Valeur = PidVitesseLongitudinale.CommandeProportionelle;	break;
				case PidVitesseLongitudinale_CommandeIntegrale :	Valeur = PidVitesseLongitudinale.CommandeIntegrale;	break;
				case PidVitesseLongitudinale_CommandeDerive :	Valeur = PidVitesseLongitudinale.CommandeDerive;	break;
				case PidVitesseLongitudinale_Kp :	Valeur = PidVitesseLongitudinale.Kp;	break;
				case PidVitesseLongitudinale_Ki :	Valeur = PidVitesseLongitudinale.Ki;	break;
				case PidVitesseLongitudinale_Kd :	Valeur = PidVitesseLongitudinale.Kd;	break;
				case PidVitesseLongitudinale_KpArret :	Valeur = PidVitesseLongitudinale.KpArret;	break;
				case PidVitesseLongitudinale_KiArret :	Valeur = PidVitesseLongitudinale.KiArret;	break;
				case PidVitesseLongitudinale_KdArret :	Valeur = PidVitesseLongitudinale.KdArret;	break;
				case PidVitesseLongitudinale_KpRoule :	Valeur = PidVitesseLongitudinale.KpRoule;	break;
				case PidVitesseLongitudinale_KiRoule :	Valeur = PidVitesseLongitudinale.KiRoule;	break;
				case PidVitesseLongitudinale_KdRoule :	Valeur = PidVitesseLongitudinale.KdRoule;	break;
				case PidVitesseLongitudinale_Commande :	Valeur = PidVitesseLongitudinale.Commande;	break;
				case PidVitesseAngulaire_Erreur :	Valeur = PidVitesseAngulaire.Erreur;	break;
				case PidVitesseAngulaire_ErreurPrecedente :	Valeur = PidVitesseAngulaire.ErreurPrecedente;	break;
				case PidVitesseAngulaire_IntegraleErreur :	Valeur = PidVitesseAngulaire.IntegraleErreur;	break;
				case PidVitesseAngulaire_DeriveErreur :	Valeur = PidVitesseAngulaire.DeriveErreur;	break;
				case PidVitesseAngulaire_CommandeProportionelle :	Valeur = PidVitesseAngulaire.CommandeProportionelle;	break;
				case PidVitesseAngulaire_CommandeIntegrale :	Valeur = PidVitesseAngulaire.CommandeIntegrale;	break;
				case PidVitesseAngulaire_CommandeDerive :	Valeur = PidVitesseAngulaire.CommandeDerive;	break;
				case PidVitesseAngulaire_Kp :	Valeur = PidVitesseAngulaire.Kp;	break;
				case PidVitesseAngulaire_Ki :	Valeur = PidVitesseAngulaire.Ki;	break;
				case PidVitesseAngulaire_Kd :	Valeur = PidVitesseAngulaire.Kd;	break;
				case PidVitesseAngulaire_KpArret :	Valeur = PidVitesseAngulaire.KpArret;	break;
				case PidVitesseAngulaire_KiArret :	Valeur = PidVitesseAngulaire.KiArret;	break;
				case PidVitesseAngulaire_KdArret :	Valeur = PidVitesseAngulaire.KdArret;	break;
				case PidVitesseAngulaire_KpRoule :	Valeur = PidVitesseAngulaire.KpRoule;	break;
				case PidVitesseAngulaire_KiRoule :	Valeur = PidVitesseAngulaire.KiRoule;	break;
				case PidVitesseAngulaire_KdRoule :	Valeur = PidVitesseAngulaire.KdRoule;	break;
				case PidVitesseAngulaire_Commande :	Valeur = PidVitesseAngulaire.Commande;	break;
				case PidPositionD_Erreur :	Valeur = PidPositionD.Erreur;	break;
				case PidPositionD_ErreurPrecedente :	Valeur = PidPositionD.ErreurPrecedente;	break;
				case PidPositionD_IntegraleErreur :	Valeur = PidPositionD.IntegraleErreur;	break;
				case PidPositionD_DeriveErreur :	Valeur = PidPositionD.DeriveErreur;	break;
				case PidPositionD_CommandeProportionelle :	Valeur = PidPositionD.CommandeProportionelle;	break;
				case PidPositionD_CommandeIntegrale :	Valeur = PidPositionD.CommandeIntegrale;	break;
				case PidPositionD_CommandeDerive :	Valeur = PidPositionD.CommandeDerive;	break;
				case PidPositionD_Kp :	Valeur = PidPositionD.Kp;	break;
				case PidPositionD_Ki :	Valeur = PidPositionD.Ki;	break;
				case PidPositionD_Kd :	Valeur = PidPositionD.Kd;	break;
				case PidPositionD_KpArret :	Valeur = PidPositionD.KpArret;	break;
				case PidPositionD_KiArret :	Valeur = PidPositionD.KiArret;	break;
				case PidPositionD_KdArret :	Valeur = PidPositionD.KdArret;	break;
				case PidPositionD_KpRoule :	Valeur = PidPositionD.KpRoule;	break;
				case PidPositionD_KiRoule :	Valeur = PidPositionD.KiRoule;	break;
				case PidPositionD_KdRoule :	Valeur = PidPositionD.KdRoule;	break;
				case PidPositionD_Commande :	Valeur = PidPositionD.Commande;	break;
				case PidPositionG_Erreur :	Valeur = PidPositionG.Erreur;	break;
				case PidPositionG_ErreurPrecedente :	Valeur = PidPositionG.ErreurPrecedente;	break;
				case PidPositionG_IntegraleErreur :	Valeur = PidPositionG.IntegraleErreur;	break;
				case PidPositionG_DeriveErreur :	Valeur = PidPositionG.DeriveErreur;	break;
				case PidPositionG_CommandeProportionelle :	Valeur = PidPositionG.CommandeProportionelle;	break;
				case PidPositionG_CommandeIntegrale :	Valeur = PidPositionG.CommandeIntegrale;	break;
				case PidPositionG_CommandeDerive :	Valeur = PidPositionG.CommandeDerive;	break;
				case PidPositionG_Kp :	Valeur = PidPositionG.Kp;	break;
				case PidPositionG_Ki :	Valeur = PidPositionG.Ki;	break;
				case PidPositionG_Kd :	Valeur = PidPositionG.Kd;	break;
				case PidPositionG_KpArret :	Valeur = PidPositionG.KpArret;	break;
				case PidPositionG_KiArret :	Valeur = PidPositionG.KiArret;	break;
				case PidPositionG_KdArret :	Valeur = PidPositionG.KdArret;	break;
				case PidPositionG_KpRoule :	Valeur = PidPositionG.KpRoule;	break;
				case PidPositionG_KiRoule :	Valeur = PidPositionG.KiRoule;	break;
				case PidPositionG_KdRoule :	Valeur = PidPositionG.KdRoule;	break;
				case PidPositionG_Commande :	Valeur = PidPositionG.Commande;	break;
				case PidVitesseD_Erreur :	Valeur = PidVitesseD.Erreur;	break;
				case PidVitesseD_ErreurPrecedente :	Valeur = PidVitesseD.ErreurPrecedente;	break;
				case PidVitesseD_IntegraleErreur :	Valeur = PidVitesseD.IntegraleErreur;	break;
				case PidVitesseD_DeriveErreur :	Valeur = PidVitesseD.DeriveErreur;	break;
				case PidVitesseD_CommandeProportionelle :	Valeur = PidVitesseD.CommandeProportionelle;	break;
				case PidVitesseD_CommandeIntegrale :	Valeur = PidVitesseD.CommandeIntegrale;	break;
				case PidVitesseD_CommandeDerive :	Valeur = PidVitesseD.CommandeDerive;	break;
				case PidVitesseD_Kp :	Valeur = PidVitesseD.Kp;	break;
				case PidVitesseD_Ki :	Valeur = PidVitesseD.Ki;	break;
				case PidVitesseD_Kd :	Valeur = PidVitesseD.Kd;	break;
				case PidVitesseD_KpArret :	Valeur = PidVitesseD.KpArret;	break;
				case PidVitesseD_KiArret :	Valeur = PidVitesseD.KiArret;	break;
				case PidVitesseD_KdArret :	Valeur = PidVitesseD.KdArret;	break;
				case PidVitesseD_KpRoule :	Valeur = PidVitesseD.KpRoule;	break;
				case PidVitesseD_KiRoule :	Valeur = PidVitesseD.KiRoule;	break;
				case PidVitesseD_KdRoule :	Valeur = PidVitesseD.KdRoule;	break;
				case PidVitesseD_Commande :	Valeur = PidVitesseD.Commande;	break;
				case PidVitesseG_Erreur :	Valeur = PidVitesseG.Erreur;	break;
				case PidVitesseG_ErreurPrecedente :	Valeur = PidVitesseG.ErreurPrecedente;	break;
				case PidVitesseG_IntegraleErreur :	Valeur = PidVitesseG.IntegraleErreur;	break;
				case PidVitesseG_DeriveErreur :	Valeur = PidVitesseG.DeriveErreur;	break;
				case PidVitesseG_CommandeProportionelle :	Valeur = PidVitesseG.CommandeProportionelle;	break;
				case PidVitesseG_CommandeIntegrale :	Valeur = PidVitesseG.CommandeIntegrale;	break;
				case PidVitesseG_CommandeDerive :	Valeur = PidVitesseG.CommandeDerive;	break;
				case PidVitesseG_Kp :	Valeur = PidVitesseG.Kp;	break;
				case PidVitesseG_Ki :	Valeur = PidVitesseG.Ki;	break;
				case PidVitesseG_Kd :	Valeur = PidVitesseG.Kd;	break;
				case PidVitesseG_KpArret :	Valeur = PidVitesseG.KpArret;	break;
				case PidVitesseG_KiArret :	Valeur = PidVitesseG.KiArret;	break;
				case PidVitesseG_KdArret :	Valeur = PidVitesseG.KdArret;	break;
				case PidVitesseG_KpRoule :	Valeur = PidVitesseG.KpRoule;	break;
				case PidVitesseG_KiRoule :	Valeur = PidVitesseG.KiRoule;	break;
				case PidVitesseG_KdRoule :	Valeur = PidVitesseG.KdRoule;	break;
				case PidVitesseG_Commande :	Valeur = PidVitesseG.Commande;	break;
				case MouvementActuel_DistanceDemande :	Valeur = MouvementActuel.DistanceDemande;	break;
				case MouvementActuel_AngleDemande :	Valeur = MouvementActuel.AngleDemande;	break;
				case MouvementActuel_DistanceDroiteDemande :	Valeur = MouvementActuel.DistanceDroiteDemande;	break;
				case MouvementActuel_DistanceGaucheDemande :	Valeur = MouvementActuel.DistanceGaucheDemande;	break;
				case MouvementActuel_Xc :	Valeur = MouvementActuel.Xc;	break;
				case MouvementActuel_Yc :	Valeur = MouvementActuel.Yc;	break;
				case MouvementActuel_PrecisionLongitudinale :	Valeur = MouvementActuel.PrecisionLongitudinale;	break;
				case MouvementActuel_PrecisionAngulaire :	Valeur = MouvementActuel.PrecisionAngulaire;	break;
				case MouvementActuel_Type :	Valeur = MouvementActuel.Type;	break;
				case MouvementActuel_EnAvant :	Valeur = MouvementActuel.EnAvant;	break;
				case MouvementActuel_Etape :	Valeur = MouvementActuel.Etape;	break;
				case MouvementActuel_Termine :	Valeur = MouvementActuel.Termine;	break;
				case MouvementActuel_TermineEnvoye :	Valeur = MouvementActuel.TermineEnvoye;	break;
				case MouvementActuel_TempsBlocage :	Valeur = MouvementActuel.TempsBlocage;	break;
				case MouvementActuel_ConfigAsserv :	Valeur = MouvementActuel.ConfigAsserv;	break;
								

				default:
				break;
			}
			
			SendVariable(Valeur);
			
		break;

		case RESET:
			CPUReset();
		break;

		default:
			Ack = 'I';
		break;
	}

	if(MessageRecu->SID != POSITION && MessageRecu->SID != TERMINE && MessageRecu->SID != MESURE && MessageRecu->SID != GET_VARIABLE)
		SendAck(Ack);
}//

int CPUReset(void)
{
	int i = 1;
	
	return i / (i - 1); // Réalise une division par zéro => Reset du CPU
}

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
	ECANSendMessage(&Reponse);
}//

// ----------------------------------------------------------------
// Fonction : SendTermine(void)
// But 	    : Envoie un message vers L'IA pour dire Terminé - Robot arreté
// ----------------------------------------------------------------
void SendTermine(void)
{
	MESSAGE Reponse;

	Reponse.SID = TERMINE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = MouvementActuel.Termine;
	ECANSendMessage(&Reponse);
}//

// ----------------------------------------------------------------
// Fonction : SendTermine(void)
// But 	    : Envoie un message vers L'IA pour dire Precision_Atteinte - robot non arreté
// ----------------------------------------------------------------
void SendAtteint(void)
{
	MESSAGE Reponse;

	Reponse.SID = TERMINE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 2;
	ECANSendMessage(&Reponse);
}//

void SendBloque(void)
{
	MESSAGE Reponse;

	Reponse.SID = BLOQUE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = 1;
	ECANSendMessage(&Reponse);
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
	ECANSendMessage(&Reponse);
}

void SendMesure(char Type) {

	int temp_D;
	int temp_Vc;
	int temp_V;
	int temp_Cmd;
	
	switch (Type)
	{
		case 'L': //Longitudinal
			temp_D = Etat.DistanceTotale * 1000.0; //mm
			temp_Vc = Longitudinal.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseLongitudinale * 1000.0; //mm/s
			temp_Cmd = (char)(PidVitesseLongitudinale.Commande / 4.0);
			break;

		case 'A': //Angulaire
			temp_D = Etat.AngleBrut * 1000.0; //mrad
			temp_Vc = Angulaire.VitesseConsigne * 1000.0; //mrad/s
			temp_V = Etat.VitesseAngulaire * 1000.0; //mrad/s
			temp_Cmd = (char)(PidVitesseAngulaire.Commande / 4.0);
			break;

		case 'D': //D
			temp_D = Etat.DistanceRoueD * 1000.0; //mm
			temp_Vc = RoueD.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseRoueD * 1000.0; //mm/s
			temp_Cmd = (char)(PidVitesseLongitudinale.Commande / 4.0);
			break;

		case 'G': //G
			temp_D = Etat.DistanceRoueG * 1000.0; //mm
			temp_Vc = RoueG.VitesseConsigne * 1000.0; //mm/s
			temp_V = Etat.VitesseRoueG * 1000.0; //mm/s
			temp_Cmd = (char)(PidVitesseLongitudinale.Commande / 4.0);
			break;

		default:
			temp_D = 0.0; //mm
			temp_Vc = 0.0; //mm/s
			temp_V = 0.0; //mm/s
			break;
	}

	MESSAGE Reponse;

	Reponse.SID = MESURE + (ID_CARTE << 4);
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
	Reponse.Data[7] = temp_Cmd;
	ECANSendMessage(&Reponse);
}

// ----------------------------------------------------------------
// Fonction : SendAck(void)
// But 	    : Envoie un message vers L'IA pour dire si la commande a été executée ou pas
// ----------------------------------------------------------------

void SendAck(char ack)
{
	MESSAGE Reponse;

	Reponse.SID = ACK + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 1;
	Reponse.Data[0] = ack;
	//ECANSendMessage(&Reponse);
}//

void SendVariable(float val)
{
	MESSAGE Reponse;

	Reponse.SID = GET_VARIABLE + (ID_CARTE << 4);
	Reponse.EID = 0;
	Reponse.Est_Extended = 0;
	Reponse.Nombre_Data = 4;
	DecomposerFloat(val, Reponse.Data, 0);
	ECANSendMessage(&Reponse);
}
