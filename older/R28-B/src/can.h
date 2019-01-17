#include "donnees.h"

#define CAN_OP_MODE_NORMAL 0
#define CAN_OP_MODE_DISABLE 1
#define CAN_OP_MODE_LOOPBACK 2
#define CAN_OP_MODE_LISTEN_ONLY 3 //Valeur fausse => cherchez la vrai
#define CAN_OP_MODE_CONFIG 4
#define CAN_OP_MODE_LISTEN_ALL 5 //Valeur fausse => cherchez la vrai

void CANSetOperationMode(unsigned char OP_MODE);
void CANGetMessage(MESSAGE *CanMessage, unsigned char No_RX);
void CANSendMessage(MESSAGE CanMessage, unsigned char Priorite);
void CANInitialize(unsigned char ID_Carte);

