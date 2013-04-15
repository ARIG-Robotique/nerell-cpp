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

// IO des capteurs //
// --------------- //
#define EQUIPE_PIN		9
#define TIRETTE			SWITCH_01
#define TIRETTE_PIN		8

#endif /* DEFINE_H_ */
