/*
 * BufferCirculaire.h
 *
 *  Created on: 30 mars 2010
 *      Author: Jeremie GRAULLE
 */

#ifndef BUFFERCIRCULAIRE_H_
#define BUFFERCIRCULAIRE_H_

#include "..\..\..\..\Librairies Périphériques\ECAN dsPIC33F\Librairie\donnees.h"

#define BUFFER_CIRCULAIRE_MESSAGE_TAILLE 50
#define BUFFER_CIRCULAIRE_OCTET_TAILLE BUFFER_CIRCULAIRE_MESSAGE_TAILLE*8
#define BUFFER_CIRCULAIRE_MESSAGE_TAILLE_WARNING 40
#define BUFFER_CIRCULAIRE_OCTET_TAILLE_WARNING BUFFER_CIRCULAIRE_MESSAGE_TAILLE_WARNING*8

#define BUFFER_CIRCULAIRE_STRUCT(type, taille) \
	int numDebut; \
	int numFin; \
	int nbrElem; \
	type buffer[taille];

typedef struct _BufferCirulaireMessage
{
	BUFFER_CIRCULAIRE_STRUCT(MESSAGE, BUFFER_CIRCULAIRE_MESSAGE_TAILLE)
} BufferCirulaireMessage;

typedef struct _BufferCirulaireOctet
{
	BUFFER_CIRCULAIRE_STRUCT(unsigned char, BUFFER_CIRCULAIRE_OCTET_TAILLE)
} BufferCirulaireOctet;

// initialise le buffer circulaire (en le vidant)
void bufferCirculaireMessageInit(BufferCirulaireMessage * b);
void bufferCirculaireOctetInit(BufferCirulaireOctet * b);

// retourne 0 si le buffer n'est pas vide
int bufferCirculaireMessageEstVide(const BufferCirulaireMessage * b);
int bufferCirculaireOctetEstVide(const BufferCirulaireOctet * b);

// retourne 0 si le buffer n'est pas plein
int bufferCirculaireMessageEstPlein(const BufferCirulaireMessage * b);
int bufferCirculaireOctetEstPlein(const BufferCirulaireOctet * b);

// rajoute un message dans le buffer
// retourne 0 si l'ajout est impossible car la file est pleine
int bufferCirculaireMessageInsert(BufferCirulaireMessage * b, const MESSAGE * m);
int bufferCirculaireOctetInsert(BufferCirulaireOctet * b, const unsigned char * o);

// enleve le dernier message du buffer
// retourne 0 si le retrait est impossible car la file est vide
int bufferCirculaireMessageRemove(BufferCirulaireMessage * b, MESSAGE * m);
int bufferCirculaireOctetRemove(BufferCirulaireOctet * b, unsigned char * o);

// retourne le nombre de message dans le buffer
int bufferCirculaireMessageGetNbrMessage(const BufferCirulaireMessage * b);
int bufferCirculaireOctetGetNbrMessage(const BufferCirulaireOctet * b);

#endif /* BUFFERCIRCULAIRE_H_ */
