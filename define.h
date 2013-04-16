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

#define TPS_MATCH		89000 // 89 sec pour palier au pb de réaction du bonhomme
#define START_GONFLAGE	92000 // 92 sec pour démarrer le gonflage des ballon

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
#define SERVO_RAMASSE_VERRE		3

#define BRAS_DROIT_HOME			800
#define BRAS_DROIT_CDX_HAUT		1200
#define BRAS_DROIT_BOG_HAUT		1800
#define BRAS_DROIT_BOG_BAS		1000

#define BRAS_GAUCHE_HOME		800
#define BRAS_GAUCHE_CDX_HAUT	1200
#define BRAS_GAUCHE_BOG_HAUT	1800
#define BRAS_GAUCHE_BOG_BAS		1000

#define RAMASSE_VERRE_CLOSE		1200
#define RAMASSE_VERRE_OPEN		1800

// --------------- //
// IO des capteurs //
// --------------- //

#define EQUIPE_PIN		9
#define TIRETTE			SWITCH_01
#define TIRETTE_PIN		8
// TODO : Autre capteurs

#endif /* DEFINE_H_ */
