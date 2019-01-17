/*
 * BufferCirculaire.c
 *
 *  Created on: 30 mars 2010
 *      Author: Jeremie GRAULLE
 */

#include "BufferCirculaire.h"
#include "p33Fxxxx.h"

void pauseInterrupt()
{
	asm("DISI #16000");
	while(INTCON2bits.DISI == 0); // Attend que les interruptions soient desactivees
}	

void enableInterrupt()
{
	asm("DISI #0");
	while(INTCON2bits.DISI == 1); // Attend que les interruptions soient activees
}

// initialise le buffer circulaire (en le vidant)
#define BUFFER_CIRCULAIRE_INIT \
	pauseInterrupt(); \
	b->numDebut = 0; \
	b->numFin = 0; \
	b->nbrElem = 0; \
	enableInterrupt();
void bufferCirculaireMessageInit(BufferCirulaireMessage * b)
{
	BUFFER_CIRCULAIRE_INIT
}

void bufferCirculaireOctetInit(BufferCirulaireOctet * b)
{
	BUFFER_CIRCULAIRE_INIT
}

// retourne 0 si le buffer n'est pas vide
#define BUFFER_CIRCULAIRE_EST_VIDE \
	int retour; \
	pauseInterrupt(); \
	retour = b->nbrElem==0; \
	enableInterrupt(); \
	return retour;
int bufferCirculaireMessageEstVide(const BufferCirulaireMessage * b)
{
	BUFFER_CIRCULAIRE_EST_VIDE
}

int bufferCirculaireOctetEstVide(const BufferCirulaireOctet * b)
{
	BUFFER_CIRCULAIRE_EST_VIDE
}

// retourne 0 si le buffer n'est pas plein
#define BUFFER_CIRCULAIRE_EST_PLEIN(taille) \
	int retour; \
	pauseInterrupt(); \
	retour = b->nbrElem==taille; \
	enableInterrupt(); \
	return retour;
int bufferCirculaireMessageEstPlein(const BufferCirulaireMessage * b)
{
	BUFFER_CIRCULAIRE_EST_PLEIN(BUFFER_CIRCULAIRE_MESSAGE_TAILLE)
}

int bufferCirculaireOctetEstPlein(const BufferCirulaireOctet * b)
{
	BUFFER_CIRCULAIRE_EST_PLEIN(BUFFER_CIRCULAIRE_OCTET_TAILLE)
}

// rajoute un message dans le buffer
// retourne 0 si l'ajout est impossible car la file est pleine
#define BUFFER_CIRCULAIRE_INSERT(taille) \
	pauseInterrupt(); \
	/* si le buffer est plein */ \
	if (b->nbrElem == taille) \
	{ \
		/* erreur */ \
		enableInterrupt(); \
		return 0; \
	} \
	/* ajouter l'element dans le buffer */ \
	b->buffer[b->numFin] = *element; \
	/* incrementer l'indice de fin du buffer */ \
	b->numFin++; \
	if (b->numFin==taille) \
			b->numFin=0; \
	/* mettre a jour le nombre de message */ \
	b->nbrElem++; \
	/* l'opperation est reussi */  \
	enableInterrupt(); \
	return 1;

int bufferCirculaireMessageInsert(BufferCirulaireMessage * b, const MESSAGE * element)
{
	BUFFER_CIRCULAIRE_INSERT(BUFFER_CIRCULAIRE_MESSAGE_TAILLE)
}
int bufferCirculaireOctetInsert(BufferCirulaireOctet * b, const unsigned char * element)
{
	BUFFER_CIRCULAIRE_INSERT(BUFFER_CIRCULAIRE_OCTET_TAILLE)
}

// enleve le dernier message du buffer
// retourne 0 si le retrait est impossible car la file est vide
#define BUFFER_CIRCULAIRE_MESSAGE_REMOVE(taille) \
	pauseInterrupt(); \
	/* si le buffer est vide */ \
	if (b->nbrElem == 0) \
	{ \
		/* erreur */ \
		enableInterrupt(); \
		return 0; \
	} \
	/* copier le message a enlever */ \
	*element = b->buffer[b->numDebut]; \
	/* incrementer l'indice de debut du buffer */ \
	b->numDebut++; \
	if (b->numDebut==taille) \
		b->numDebut=0; \
	/* mettre a jour le nombre de message */ \
	b->nbrElem--; \
	/* l'opperation est reussi */ \
	enableInterrupt(); \
	return 1;

int bufferCirculaireMessageRemove(BufferCirulaireMessage * b, MESSAGE * element)
{
	BUFFER_CIRCULAIRE_MESSAGE_REMOVE(BUFFER_CIRCULAIRE_MESSAGE_TAILLE)
}

int bufferCirculaireOctetRemove(BufferCirulaireOctet * b, unsigned char * element)
{
	BUFFER_CIRCULAIRE_MESSAGE_REMOVE(BUFFER_CIRCULAIRE_OCTET_TAILLE)
}

// retourne le nombre de message dans le buffer
#define BUFFER_CIRCULAIRE \
	int retour; \
	pauseInterrupt(); \
	retour = b->nbrElem; \
	enableInterrupt(); \
	return retour;
int bufferCirculaireMessageGetNbrMessage(const BufferCirulaireMessage * b)
{
	BUFFER_CIRCULAIRE
}
int bufferCirculaireOctetGetNbrMessage(const BufferCirulaireOctet * b)
{
	BUFFER_CIRCULAIRE
}
