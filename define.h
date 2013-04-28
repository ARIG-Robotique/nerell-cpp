/*
 * define.h
 *
 *  Created on: 27 janv. 2013
 *      Author: mythril
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <robot/system/capteurs/CapteurDefine.h>

#define VERSION			1

#define DEBUG_MODE

#define TPS_MATCH			89500 // 89,5 sec pour palier au pb de réaction du bonhomme
#define START_GONFLAGE		90500 // 90,5 sec pour démarrer le gonflage des ballon
#define	PREPARE_GONFLAGE	88000 // Au bout de 88 sec on allume le gonfleur pour faire montée la pression

// --------------------------------- //
// Configuration de l'asservissement //
// --------------------------------- //

#define CYCLE_ASSERV				78			// Asserv 128 us * 78 = 9,984 ms
#define TIME_ASSERV					0.009984	// En s
#define TIME_ASSERV_MS				10

/*
POUR INFOS
#define RESOLUTION_CODEUR			512
#define PPR 						512 		// pulse per rotation : 512 (d = 40mm)

#define PERIMETRE_ROUE				124			// Perimetre de la roue en mm (PI * 40)
#define RAYON 						19			// Rayon des roues en mm (~2cm)
#define ENTRAXE						161			// Entraxe des roue codeuse en mm
#define ENTRAXE_PULSE				651.084		// Entraxe en pulse
*/

#define RAMPE_ACC_DISTANCE			500.0 		// en mm/s2
#define RAMPE_DEC_DISTANCE			100.0 		// en mm/s2

#define RAMPE_ACC_ORIENTATION		500.0 		// en mm/s2
#define RAMPE_DEC_ORIENTATION		100.0 		// en mm/s2

// -------------- //
// Parametres PID //
// -------------- //

#define K_P_DISTANCE   		1.50
#define K_I_DISTANCE   		0.00
#define K_D_DISTANCE   		0.00

#define K_P_ORIENTATION		1.50
#define K_I_ORIENTATION		0.00
#define K_D_ORIENTATION		0.00

// ------------------------------- //
// Configuration des servo moteurs //
// ------------------------------- //

#define SERVO_BRAS_DROIT		1
#define SERVO_BRAS_GAUCHE		2
#define SERVO_PORTE_DROITE		3
#define SERVO_PORTE_GAUCHE		5

#define BRAS_DROIT_HOME			1870
#define BRAS_DROIT_CDX_HAUT		1040
#define BRAS_DROIT_BOUG_HAUT	1500 // TBD
#define BRAS_DROIT_BOUG_BAS		1500 // TBD

#define BRAS_GAUCHE_HOME		790
#define BRAS_GAUCHE_CDX_HAUT	1620
#define BRAS_GAUCHE_BOUG_HAUT	1500 // TBD
#define BRAS_GAUCHE_BOUG_BAS	1500 // TBD

#define PORTE_DROITE_CLOSE		850
#define PORTE_DROITE_OPEN		1860
#define PORTE_DROITE_INTERM		1500

#define PORTE_GAUCHE_CLOSE		1850
#define PORTE_GAUCHE_OPEN		950
#define PORTE_GAUCHE_INTERM		1310

// --------------- //
// IO des capteurs //
// --------------- //

// OUTPUTS
#define ELECTRO_VANNE	48
#define GONFLEUR		30

// INPUTS
#define EQUIPE_PIN		15

#define TIRETTE			SWITCH_01
#define TIRETTE_PIN		26

#define PROX1			GP2D_03
#define PROX1_PIN		24

#define PROX2			GP2D_04
#define PROX2_PIN		18

#define PROX3			GP2D_05
#define PROX3_PIN		14

#define PROX4			GP2D_06
#define PROX4_PIN		22

#define PROX5			GP2D_07
#define PROX5_PIN		17

#define PROX6			GP2D_08
#define PROX6_PIN		28

#define PROX7			GP2D_09
#define PROX7_PIN		19

#define PROX8			GP2D_10
#define PROX8_PIN		16

#endif /* DEFINE_H_ */
