/* 
 * @file   configuration.h
 * @author Romain TAPREST
 *
 * @date : 14/05/2015
 */

#ifndef CONFIGURATION_H
#define	CONFIGURATION_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ENTREAXE 0.329615; // Valeur au 07/05/2014 : 0.329324 // 08-05 : 0.329886
#define DIAMETRE_ROUE_D 0.0636288; // Valeurs calibration 23/05/2014  : 0.0636158
#define DIAMETRE_ROUE_G 0.0635128;// Ancienne valeur au 23/05/2014 : 0.0637898

#define PRECISION_ANGULAIRE     4.0*PI/180.0 // 4°
#define PRECISION_LONGITUDINALE 0.05 //5cm

    // Longueur du déplacement une fois la position atteinte par le passer par,
    // en attendant l'ordre suivant de l'IA
#define DEPASSEMENT_PASSER_PAR  1 // mètres

#define VIT_MAX_LONGITUDINAL 1.4
#define VIT_MAX_ANGULAIRE 5.0
#define VIT_MAX_ROUE_D 1.5
#define VIT_MAX_ROUE_G 1.5

#define Kp_POS_LONGITUDINAL 7
#define Ki_POS_LONGITUDINAL 0
#define Kd_POS_LONGITUDINAL 0

#define Kp_POS_ANGULAIRE 12
#define Ki_POS_ANGULAIRE 0
#define Kd_POS_ANGULAIRE 0

#define Kp_VIT_LONGITUDINAL 800
#define Ki_VIT_LONGITUDINAL 0
#define Kd_VIT_LONGITUDINAL 0

#define Kp_VIT_ANGULAIRE 200
#define Ki_VIT_ANGULAIRE 0
#define Kd_VIT_ANGULAIRE 0

#define Kp_ROUE_DROITE 5
#define Ki_ROUE_DROITE 0
#define Kd_ROUE_DROITE 0

#define Kp_ROUE_GAUCHE 5
#define Ki_ROUE_GAUCHE 0
#define Kd_ROUE_GAUCHE 0

#define Kp_VIT_ROUE_DROITE 700
#define Ki_VIT_ROUE_DROITE 0
#define Kd_VIT_ROUE_DROITE 0

#define Kp_VIT_ROUE_GAUCHE 700
#define Ki_VIT_ROUE_GAUCHE 0
#define Kd_VIT_ROUE_GAUCHE 0

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIGURATION_H */

