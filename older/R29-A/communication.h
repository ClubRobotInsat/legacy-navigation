#ifndef COMMUNICATION_H
#define COMMUNICATION_H
// ---------------------------------------------------------------------
// 			INSA Club Robot 2006/2007
// ---------------------------------------------------------------------
// Projet               : Carte Asservissement Moteurs
// Fichier				: decision.h	
// Auteurs              : Nicolas Dos Santos, Jos� Mar�a Martin
// Description			: Fichier d'ent�te
// Revisions			: 
//			o 20070222 Creation du fichier
//						
// ---------------------------------------------------------------------
//void SendMsg(unsigned char MsgToSend[]); //Envoie un message vers L'IA
void SendTermine(void);//Envoie le message termin�
void SendAtteint(void);
void SendBloque(void);
void SendArretUrgenceTermine(void);//Termin� avec en plus l'info sur la distance ou l'angle parcourue
void SendPong(void);//Envoie le message Pong
void SendRecu(void);//Envoie le message Re�u
void SendIgnorer(void);//Envoie le message Ignor�
void SendPosition();  //Envoi les coordonn�es (X,Y,Th�ta) du robot
void SendMesure(unsigned char Type); //Envoi au PC les infos pour calibrer les PID

#define ID_CARTE 0x1

#endif
