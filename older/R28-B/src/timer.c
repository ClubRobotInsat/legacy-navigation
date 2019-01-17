#include "timer.h"
#include "p30fxxxx.h"

void _ISRFAST _T1Interrupt(void)
{
	ExecutionToutLesTe();
	IFS0bits.T1IF = 0;
}
void T1Initialize(float Te, float Fcy)
{
	float Tcy = 1.0/Fcy;
	
	//Configuration générale
	T1CONbits.TSIDL = 1; //Arrete de compter en mode CPU IDLE
	T1CONbits.TGATE = 0; //Pas de mode Gate Accumulation
	
	//Configuration de la période
	T1CONbits.TCS = 0; //Horloge source : Fcy
	//T1CONbits.TCKPS = 0; //Prescaler : 0 => 1:1, 1 => 1:8, 2 => 1:64, 3 => 1:256
	//PR1 = 1; //Définit la période du timer : Te = PR1*Tcy/Prescaler
	
	//Autoconfig : OK fonctionne bien
	if (Te <= 1 * 65535 * Tcy) //Prescaler de 1
	{
		T1CONbits.TCKPS = 0;
		PR1 = (int)(Te / Tcy) - 7; //le -7 est là pour compenser les 7 cycles supplémentaires ajoutés par l'interruption
	} else if (Te <= 8 * 65535 * Tcy) //Prescaler de 8
	{
		T1CONbits.TCKPS = 1;
		PR1 = (int)(Te / (Tcy * 8)) - 1; //le -1 (=> -8 avec le prescaler) est là pour compenser les 7 cycles supplémentaires ajoutés par l'interruption
	} else if (Te <= 64 * 65535 * Tcy) //Prescaler de 64
	{
		T1CONbits.TCKPS = 2;
		PR1 = (int)(Te / (Tcy * 64)); //A noter que pour un prescaler de 64, les 7 cycles en plus de l'interruption ne sont plus compensables
	} else if (Te <= 256 * 65535 * Tcy) //Prescaler de 256
	{
		T1CONbits.TCKPS = 3;
		PR1 = (int)(Te / (Tcy * 256)); //A noter que pour un prescaler de 256, les 7 cycles en plus de l'interruption ne sont plus compensables
	} else { //Impossible d'obtenir ce Te avec ce Tcy
		return;
	}
	
	//Configuration interruptions
	IFS0bits.T1IF = 0; //On rabaisse le flag de l'interruption
	IEC0bits.T1IE = 1; //On active l'interruption
	IPC0bits.T1IP = 6; //Priorité 6/7 : très prioritaire mais moins que le comptage des odomètres
	
	//Activation du timer
	T1CONbits.TON = 1; 
}
