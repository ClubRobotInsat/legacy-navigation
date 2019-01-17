#ifndef INPUTS_H
#define INPUTS_H

	#include "p30fxxxx.h"
	#include "math.h"
	
	#define RB0 1
	#define RB1 2
	#define RB2 4
	#define RB3 8
	#define RB4 16
	#define RB5 32
	#define RB6 64
	#define RB7 128
	
	int INPUTInitialize(void);
	
	//Accesseurs
	float ODODistRoue0(void);
	float ODODistRoue1(void);

#endif
