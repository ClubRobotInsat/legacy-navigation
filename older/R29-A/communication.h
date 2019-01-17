#ifndef COMMUNICATION_H
#define COMMUNICATION_H
// ---------------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ---------------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: decision.h	
// Auteurs              : Nicolas Dos Santos, José María Martin
// Description			: Fichier d'entête
// Revisions			: 
//			o 20070222 Creation du fichier
//						
// ---------------------------------------------------------------------
//void SendMsg(unsigned char MsgToSend[]); //Envoie un message vers L'IA
void SendTermine(void);//Envoie le message terminé
void SendAtteint(void);
void SendBloque(void);
void SendArretUrgenceTermine(void);//Terminé avec en plus l'info sur la distance ou l'angle parcourue
void SendPong(void);//Envoie le message Pong
void SendRecu(void);//Envoie le message Reçu
void SendIgnorer(void);//Envoie le message Ignoré
void SendPosition();  //Envoi les coordonnées (X,Y,Théta) du robot
void SendMesure(unsigned char Type); //Envoi au PC les infos pour calibrer les PID

#define ID_CARTE 0x1

#endif
