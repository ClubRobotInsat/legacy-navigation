/* 
 * File:   configuration.h
 * Author: Robot
 *
 * Created on 2 avril 2015, 16:39
 */

#ifndef CONFIGURATION_H
#define	CONFIGURATION_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ENTREAXE 0.210000; // Valeur au 14/05/2015 : 0.329324 // 08-05 : 0.329886
#define DIAMETRE_ROUE_D 0.0636288; // Valeurs calibration 14/05/2015  : 0.0636288
#define DIAMETRE_ROUE_G 0.0635156;// Ancienne valeur au 14/05/2015 : 0.0635156

#define PRECISION_ANGULAIRE     10.0*PI/180.0 // 10°
#define PRECISION_LONGITUDINALE 0.06 //6cm

    // Longueur du déplacement une fois la position atteinte par le passer par,
    // en attendant l'ordre suivant de l'IA
#define DEPASSEMENT_PASSER_PAR  1 // mètres

#define VIT_MAX_LONGITUDINAL 0.2
#define VIT_MAX_ANGULAIRE PI/4
#define VIT_MAX_ROUE_D 1.5
#define VIT_MAX_ROUE_G 1.5

#define Kp_POS_LONGITUDINAL 6   // 6 ?
#define Ki_POS_LONGITUDINAL 0
#define Kd_POS_LONGITUDINAL 0

#define Kp_POS_ANGULAIRE 5  // 7 ?
#define Ki_POS_ANGULAIRE 0
#define Kd_POS_ANGULAIRE 0

#define Kp_VIT_LONGITUDINAL 500    // 700 ?
#define Ki_VIT_LONGITUDINAL 0
#define Kd_VIT_LONGITUDINAL 0

#define Kp_VIT_ANGULAIRE 150    // 150 ?
#define Ki_VIT_ANGULAIRE 0
#define Kd_VIT_ANGULAIRE 0

#define Kp_ROUE_DROITE 4
#define Ki_ROUE_DROITE 0
#define Kd_ROUE_DROITE 0

#define Kp_ROUE_GAUCHE 4
#define Ki_ROUE_GAUCHE 0
#define Kd_ROUE_GAUCHE 0

#define Kp_VIT_ROUE_DROITE 600
#define Ki_VIT_ROUE_DROITE 0
#define Kd_VIT_ROUE_DROITE 0

#define Kp_VIT_ROUE_GAUCHE 600
#define Ki_VIT_ROUE_GAUCHE 0
#define Kd_VIT_ROUE_GAUCHE 0

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIGURATION_H */

