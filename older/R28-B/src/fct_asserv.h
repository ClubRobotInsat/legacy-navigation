#ifndef FCT_ASSERV_H
#define FCT_ASSERV_H

#define AsservPosLong	1
#define AsservVitLong	2
#define AsservPosAng	4
#define AsservVitAng	8

#define AsservPosD		16
#define AsservVitD		32
#define AsservPosG		64
#define AsservVitG		128

float Seuiller(float Val, float Abs);
float Mod2Pi(float Angle);
float absf(float val);
int EstDansInterval(float Val, float Abs);

#endif
