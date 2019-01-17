#include "can.h"
#include "p30fxxxx.h"

MESSAGE CanMessageRX0;
MESSAGE CanMessageRX1;

void CANNouveauMessage(MESSAGE * MessageRecu);

void _ISRFAST _C1Interrupt(void) //Corps général de gestion de l'interruption mis en place, reste à remplir
{	
	while(C1CTRLbits.ICODE != 0) 
	{
		//Ordre de priorité des interrupt : ERR, TX2, TX1, TX0, RX1, RX0, WAK (ERR = max, WAK = min)
		//ICODE = 0 => Rien
		//ICODE = 1 => ERR
		//ICODE = ...
		//ICODE = 7 => WAK
		switch(C1CTRLbits.ICODE) //Gère les interruptions pas ordre de priorité jusqu'à ce qu'il n'y en ait plus
		{
			case 0: //Rien
				break;
				
			case 1: //Erreur
				//Etudier les différents flags d'erreur
								
				//Flag d'erreur
				C1INTFbits.RX0OVR = 0;
				C1INTFbits.RX1OVR = 0;
				C1INTFbits.TXBO = 0;
				C1INTFbits.TXEP = 0;
				C1INTFbits.RXEP = 0;
				C1INTFbits.TXWAR = 0;
				C1INTFbits.RXWAR = 0;
				C1INTFbits.EWARN = 0;
				
				//Plus d'erreur
				C1INTFbits.ERRIF = 0;
				break;
				
			case 2: //Un des TX à fini d'envoyer correctement un message
			case 3:
			case 4:
				C1INTFbits.TX2IF = 0;
				C1INTFbits.TX1IF = 0;
				C1INTFbits.TX0IF = 0;
				break;
				
			case 5: //RX1 à reçu un message
				//Traiter le message
				CANGetMessage(&CanMessageRX1,1);
				CANNouveauMessage(&CanMessageRX1);
				C1INTFbits.RX1IF = 0;
				break;
				
			case 6: //RX0 à reçu un message
				//Traiter le message
				CANGetMessage(&CanMessageRX0,0);
				CANNouveauMessage(&CanMessageRX0);
				C1INTFbits.RX0IF = 0;
				break;
				
			case 7: //Demande de wake up
				C1INTFbits.WAKIF = 0;
				break;
				
			default :
				while(1);
				break;
		}
	}	
	C1INTF = 0;
	IFS1bits.C1IF = 0;
}


void CANSetFilter(char filter_no, unsigned int sid, unsigned long eid) //OK, testé, fonctionne
{  
	unsigned int eidh = eid>>6;
	unsigned int eidl = (eid<<10)&0xFC00;

    switch(filter_no)
    {
    case 0:
      C1RXF0SIDbits.SID = sid;
      C1RXF0SIDbits.EXIDE = 0;
      C1RXF0EIDH = eidh;
      C1RXF0EIDL = eidl;
      break;
    case 1:
      C1RXF1SIDbits.SID = sid;
      C1RXF1SIDbits.EXIDE = 0;
      C1RXF1EIDH = eidh;
      C1RXF1EIDL = eidl;
      break;
    case 2:
      C1RXF2SIDbits.SID = sid;
      C1RXF2SIDbits.EXIDE = 0;
      C1RXF2EIDH = eidh;
      C1RXF2EIDL = eidl;
      break;
    case 3:
      C1RXF3SIDbits.SID = sid;
      C1RXF3SIDbits.EXIDE = 0;
      C1RXF3EIDH = eidh;
      C1RXF3EIDL = eidl;
      break;
    case 4:
      C1RXF4SIDbits.SID = sid;
      C1RXF4SIDbits.EXIDE = 0;
      C1RXF4EIDH = eidh;
      C1RXF4EIDL = eidl;
      break;
    case 5:
      C1RXF5SIDbits.SID = sid;
      C1RXF5SIDbits.EXIDE = 0;
      C1RXF5EIDH = eidh;
      C1RXF5EIDL = eidl;
      break;
    default:
	 
      break;
    }
}



void CANSetMask(char mask_no, unsigned int sid, unsigned long eid) //OK,testé,fonctionne
{ 
	unsigned int eidh = eid>>6;
	unsigned int eidl = (eid<<10)&0xFC00;

    switch(mask_no)
    {
    case 0:
      C1RXM0SIDbits.SID  = sid;
      C1RXM0SIDbits.MIDE = 1;
      C1RXM0EIDH = eidh;
      C1RXM0EIDL = eidl;
      break;
    case 1:
      C1RXM1SIDbits.SID  = sid;
      C1RXM1SIDbits.MIDE = 1;
      C1RXM1EIDH = eidh;
      C1RXM1EIDL = eidl;
      break;
    default:
		while(1);
      break;
    }
}



char CANIsTXReady(char buffno) //OK
{  
    switch(buffno)
    {
    case 0:
        return !(C1TX0CONbits.TXREQ);
        break;
    case 1:
        return !(C1TX1CONbits.TXREQ);
        break;
    case 2:
        return !(C1TX2CONbits.TXREQ);
        break;
	default:
		while(1);
    }
    return 0;

}
  
void CANSendMessage(MESSAGE CanMessage, unsigned char Priorite) //OK, testé, fonctionne
{
	unsigned char i = 0;
	
	//Verifie que les paramètres sont tous valides (inférieures à leurs valeurs max)
	if (CanMessage.SID > 2047 || CanMessage.EID > 262143 || CanMessage.Nombre_Data > 8 || CanMessage.Nombre_Data < 1 || Priorite > 3 || CanMessage.Est_Extended > 1)
	{
		while(1);
	}
	
	//Cherche un buffer vide
	char No_TX = -1;
	while (No_TX == -1) //On essaye jusqu'à trouver une place
	{
		if (CANIsTXReady(0))
			No_TX = 0;
			
		if (CANIsTXReady(1))
			No_TX = 1;
			
		if (CANIsTXReady(2))
			No_TX = 2;	
	}
	
	//On remplit le buffer libre
	switch(No_TX)
	{
		case 0:
			//On dit si il s'agit d'un message standard ou extended
			C1TX0SIDbits.TXIDE = CanMessage.Est_Extended;
			
			//Il s'agit d'une data_frame et pas d'une remote_frame (possibilité de gérer ça, un jour...)
			C1TX0SIDbits.SRR = 0;
			C1TX0DLCbits.TXRTR = 0;
			
			//On définit le SID
			C1TX0SIDbits.SID5_0 = (CanMessage.SID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX0SIDbits.SID10_6 = (CanMessage.SID&0x7C0)>>6; //Pour avoir les 5 bits de poid fort
			
			//On définit le EID
			C1TX0DLCbits.EID5_0 = (CanMessage.EID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX0EIDbits.EID13_6 = (CanMessage.EID&0x3FC0)>>6; //Pour avoir les 8 bits du milieu
			C1TX0EIDbits.EID17_14 = (CanMessage.EID&0x3C000)>>14; //Pour avoir les 4 bits de poid fort
			
			//On définit le nombre de data
			//NB : On ne peut pas envoyer 0 data => c'est une Data_Frame !! si on veut envoyer 0 data il faut envoyer une Remote_Frame (à gérer)
			C1TX0DLCbits.DLC = CanMessage.Nombre_Data;
			
			//On place les data dans les transmit buffer byte
			for(i = 0;i < (CanMessage.Nombre_Data);i++)
    		{
	    		*((unsigned char *)&C1TX0B1+i)= CanMessage.Data[i]; //A chaque fois on incrémente l'adresse de 1 pour passer à l'octet suivant
    		}
    		
    		//On définit la priorité du message (de 0 à 3)
    		C1TX0CONbits.TXPRI = Priorite;
    		
    		//On place le message en file d'attente
    		C1TX0CONbits.TXREQ = 1;
    		
    		//NB : On pourra essayer de gérer le TXERR et TXLARB...à voir
			break;
			
		case 1:
			//On dit si il s'agit d'un message standard ou extended
			C1TX1SIDbits.TXIDE = CanMessage.Est_Extended;
			
			//Il s'agit d'une data_frame et pas d'une remote_frame (possibilité de gérer ça, un jour...)
			C1TX1SIDbits.SRR = 0;
			C1TX1DLCbits.TXRTR = 0;
			
			//On définit le SID
			C1TX1SIDbits.SID5_0 = (CanMessage.SID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX1SIDbits.SID10_6 = (CanMessage.SID&0x7C0)>>6; //Pour avoir les 5 bits de poid fort
			
			//On définit le EID
			C1TX1DLCbits.EID5_0 = (CanMessage.EID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX1EIDbits.EID13_6 = (CanMessage.EID&0x3FC0)>>6; //Pour avoir les 8 bits du milieu
			C1TX1EIDbits.EID17_14 = (CanMessage.EID&0x3C000)>>14; //Pour avoir les 4 bits de poid fort
			
			//On définit le nombre de data
			//NB : On ne peut pas envoyer 0 data => c'est une Data_Frame !! si on veut envoyer 0 data il faut envoyer une Remote_Frame (à gérer)
			C1TX1DLCbits.DLC = CanMessage.Nombre_Data;
			
			//On place les data dans les transmit buffer byte
			for(i = 0;i < (CanMessage.Nombre_Data);i++)
    		{
	    		*((unsigned char *)&C1TX1B1+i)= CanMessage.Data[i]; //A chaque fois on incrémente l'adresse de 1 pour passer à l'octet suivant
    		}
    		
    		//On définit la priorité du message (de 0 à 3)
    		C1TX1CONbits.TXPRI = Priorite;
    		
    		//On place le message en file d'attente
    		C1TX1CONbits.TXREQ = 1;
    			
       		//NB : On pourra essayer de gérer le TXERR et TXLARB...à voir		
			break;
			
		case 2:
			//On dit si il s'agit d'un message standard ou extended
			C1TX2SIDbits.TXIDE = CanMessage.Est_Extended;
			
			//Il s'agit d'une data_frame et pas d'une remote_frame (possibilité de gérer ça, un jour...)
			C1TX2SIDbits.SRR = 0;
			C1TX2DLCbits.TXRTR = 0;
			
			//On définit le SID
			C1TX2SIDbits.SID5_0 = (CanMessage.SID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX2SIDbits.SID10_6 = (CanMessage.SID&0x7C0)>>6; //Pour avoir les 5 bits de poid fort
			
			//On définit le EID
			C1TX2DLCbits.EID5_0 = (CanMessage.EID&0x3F); //Pour avoir les 6 bits de poid faible
			C1TX2EIDbits.EID13_6 = (CanMessage.EID&0x3FC0)>>6; //Pour avoir les 8 bits du milieu
			C1TX2EIDbits.EID17_14 = (CanMessage.EID&0x3C000)>>14; //Pour avoir les 4 bits de poid fort
			
			//On définit le nombre de data
			//NB : On ne peut pas envoyer 0 data => c'est une Data_Frame !! si on veut envoyer 0 data il faut envoyer une Remote_Frame (à gérer)
			C1TX2DLCbits.DLC = CanMessage.Nombre_Data;
			
			//On place les data dans les transmit buffer byte
			for(i = 0;i < (CanMessage.Nombre_Data);i++)
    		{
	    		*((unsigned char *)&C1TX2B1+i)= CanMessage.Data[i]; //A chaque fois on incrémente l'adresse de 1 pour passer à l'octet suivant
    		}
    		
    		//On définit la priorité du message (de 0 à 3)
    		C1TX2CONbits.TXPRI = Priorite;
    		
    		//On place le message en file d'attente
    		C1TX2CONbits.TXREQ = 1;	
   
      		//NB : On pourra essayer de gérer le TXERR et TXLARB...à voir	
			break;
			
		default :
			while(1);
			break;
	}
}	

void CANSetOperationMode(unsigned char OP_MODE)  //OK, testé, fonctionne
{
	//NB : Fonction bloquante
	if (OP_MODE <= 7)  //Si il s'agit d'un mode valide
	{
		C1CTRLbits.REQOP = OP_MODE; //On demande le changement de mode
		while(C1CTRLbits.OPMODE != OP_MODE); //On attend qu'il soit effectif
	} else {
		while(1); //Erreur
	}
}	
	
	
void CANInitialize(unsigned char ID_Carte) //OK, testé, fonctionne, et il faut rajouter le calcul du baud rate
{
	//Passage en mode CONFIG
	CANSetOperationMode(CAN_OP_MODE_CONFIG);

	//Config Générale
	C1CTRLbits.CANCAP = 0; //Désactive le mode CAN Capture
	C1CTRLbits.CSIDL = 0; //Le module continue même si le périphérique est en IDLE_MODE
	C1CTRLbits.ABAT = 0; //Abord All Transmission => Non
	C1CTRLbits.CANCKS = 1; //Fcan = Fcy
	
	//Config Baud Rate (environ 416.6kbits/sec avec Fcy = 10MHz)
	/* Ajouter ici les calculs et explications */
	C1CFG1bits.SJW = 1;
	C1CFG1bits.BRP = 2;
	C1CFG2bits.WAKFIL = 0; //On ne gère pas encore la mise en veille
	C1CFG2bits.SEG2PH = 3;
	C1CFG2bits.SEG2PHTS = 1;	//On gère nous même la config de SEG2PH
	C1CFG2bits.SEG1PH = 2;
	C1CFG2bits.PRSEG = 2;
	C1CFG2bits.SAM = 1; //On fait 3 mesures du sample point et on vote à la majorité
	
	//Config de la réception
	C1RX0CONbits.RXFUL = 0; //Le buffer0 est vide et prêt
	C1RX0CONbits.DBEN = 1; //Si le buffer 0 est occupé alors essayer sur le buffer 1
	C1RX1CONbits.RXFUL = 0; //Le buffer1 est vide et prêt
	/* Mettre ici la configuration des masques et des filtres */
	CANSetMask(0,0x3F0,0x00000);
	CANSetMask(1,0x3F0,0x00000);
	
	CANSetFilter(0,ID_Carte<<4,0x00000);
	CANSetFilter(1,ID_Carte<<4,0x00000);
	
	CANSetFilter(2,(ID_Carte<<4)&0x3F0,0x00000);
	CANSetFilter(3,(ID_Carte<<4)&0x3F0,0x00000);
	CANSetFilter(4,(ID_Carte<<4)&0x3F0,0x00000);
	CANSetFilter(5,(ID_Carte<<4)&0x3F0,0x00000);
	
	//Mise à zéro des compteurs d'erreurs
	C1EC = 0;
	
	//Config des interruptions
    C1INTF = 0; 		  //Rabaisse tous les Interruption Flag
	C1INTEbits.IVRIE = 1; //Invalid Message Interrupt
	C1INTEbits.WAKIE = 0; //Wake Up Interrupt    
	C1INTEbits.ERRIE = 1; //Error Interrupt   
	C1INTEbits.TX2IE = 0; //TX2 => Message envoyé  
	C1INTEbits.TX1IE = 0; //TX1 => Message envoyé     
	C1INTEbits.TX0IE = 0; //TX0 => Message envoyé      
	C1INTEbits.RX1IE = 1; //RX1 => Message reçu   
	C1INTEbits.RX0IE = 1; //RX0 => Message reçu     
    IPC6bits.C1IP = 4; //C1Interrupt priorité de 0 à 7 (0 = min, 7 = max)
    IEC1bits.C1IE = 1; //enable C1Interrupt
    IFS1bits.C1IF = 0; //Rabaisse le flag général d'interruption CAN

	//Passage en mode NORMAL
	CANSetOperationMode(CAN_OP_MODE_NORMAL);
}

void CANGetMessage(MESSAGE *CanMessage, unsigned char No_RX) //OK
{
	unsigned char i = 0;
	
	switch(No_RX)
	{
		case 0:
			//On regarde si c'est un message extended ou pas
			CanMessage->Est_Extended = C1RX0SIDbits.RXIDE;
			
			//On peut traiter differement si il s'agit d'une remote frame en utilisant C1RX0DLCbits.RXRTR
			
			//On récupère le SID
			CanMessage->SID = C1RX0SIDbits.SID;
			
			//On récupère le EID
			CanMessage->EID = C1RX0DLCbits.EID5_0 + (C1RX0EID<<6);
			
			//On récupère le nombre de données
			CanMessage->Nombre_Data = C1RX0DLCbits.DLC;
			
			//On récupère les données
			for(i = 0;i < (CanMessage->Nombre_Data);i++)
    		{
	    		CanMessage->Data[i] = *((unsigned char *)&C1RX0B1+i); //A chaque fois on incrémente l'adresse de 1 pour passer à l'octet suivant
    		}	
    		
    		//On signifie que le buffer est disponible
    		C1RX0CONbits.RXFUL = 0;
    		
    		//NB : on peut voir une utilisation à FILHIT (donne le numéro du filtre qui a laissé passer le message)
    		// De même on pourra gérer le RXERR... à voir		
			break;
			
		case 1:
			//On regarde si c'est un message extended ou pas
			CanMessage->Est_Extended = C1RX1SIDbits.RXIDE;
			
			//On peut traiter differement si il s'agit d'une remote frame en utilisant C1RX0DLCbits.RXRTR
			
			//On récupère le SID
			CanMessage->SID = C1RX1SIDbits.SID;
			
			//On récupère le EID
			if (CanMessage->Est_Extended == 1)
				CanMessage->EID = C1RX1DLCbits.EID5_0 + (C1RX1EID<<6);
			
			//On récupère le nombre de données
			CanMessage->Nombre_Data = C1RX1DLCbits.DLC;
			
			//On récupère les données
			for(i = 0;i < (CanMessage->Nombre_Data);i++)
    		{
	    		CanMessage->Data[i] = *((unsigned char *)&C1RX1B1+i); //A chaque fois on incrémente l'adresse de 1 pour passer à l'octet suivant
    		}	
    		
    		//On signifie que le buffer est disponible
    		C1RX1CONbits.RXFUL = 0;
    		
    		//NB : on peut voir une utilisation à FILHIT (donne le numéro du filtre qui a laissé passer le message)
    		// De même on pourra gérer le RXERR... à voir			
			break;
			
		default :
			while(1);
			break;
	}
}
