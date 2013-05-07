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
#define START_GONFLAGE		90000 // 90 sec pour démarrer le gonflage des ballon
#define	PREPARE_GONFLAGE	88000 // Au bout de 88 sec on allume le gonfleur pour faire monter les tours
#define END_TOUT			100000 // 100 sec c'est vraiment la fin de tout

// --------------------------------- //
// Configuration de l'asservissement //
// --------------------------------- //

//#define CYCLE_ASSERV				78			// Asserv 128 us * 78 = 9,984 ms
//#define TIME_ASSERV				0.009984	// En s
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

// ------------------------------- //
// Configuration des servo moteurs //
// ------------------------------- //

#define SPEED_BRAS				10
#define SPEED_PORTE				5

#define SERVO_BRAS_DROIT		1
#define SERVO_BRAS_GAUCHE		2
#define SERVO_PORTE_DROITE		3
#define SERVO_PORTE_GAUCHE		5

#define BRAS_DROIT_HOME			1865
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
#define EQUIPE_PIN		19

#define TIRETTE			SWITCH_01
#define TIRETTE_PIN		28

#define LATERAL_ARRIERE_GAUCHE		GP2D_03
#define LATERAL_ARRIERE_GAUCHE_PIN	16

#define LATERAL_ARRIERE_DROIT		GP2D_02
#define LATERAL_ARRIERE_DROIT_PIN	24

#define ARRIERE_GAUCHE				GP2D_05
#define ARRIERE_GAUCHE_PIN			14

#define ARRIERE_DROIT				GP2D_06
#define ARRIERE_DROIT_PIN			26

#define LATERAL_AVANT_DROIT			GP2D_07
#define LATERAL_AVANT_DROIT_PIN		18

#define AVANT_DROIT					GP2D_08
#define AVANT_DROIT_PIN				15

#define AVANT_GAUCHE				GP2D_09
#define AVANT_GAUCHE_PIN			17

#define LATERAL_AVANT_GAUCHE		GP2D_10
#define LATERAL_AVANT_GAUCHE_PIN	22

#endif /* DEFINE_H_ */
