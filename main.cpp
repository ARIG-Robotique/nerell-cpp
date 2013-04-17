#include <Arduino.h>
#include <Wire.h>
#include <robot/system/capteurs/Board2007NoMux.h>
#include <robot/system/capteurs/CapteurDefine.h>
#include <robot/system/servos/SD21.h>
#include <robot/utils/Convertion.h>
#include <robot/RobotManager.h>

#include "define.h"

// variable globales
unsigned long startMatch;

// Prototype des fonctions
void setup();
void matchLoop();
void endMatch();

// Classe de convertion
Convertion Conv = Convertion(4.044, 11.36);

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();
SD21 servoManager = SD21();
Board2007NoMux capteurs = Board2007NoMux();

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	// Procédure d'initialisation Robot (calage, tirette, etc).
#ifdef DEBUG_MODE
	Serial.println(" == INIT MATCH ==");
	Serial.println(" - Attente tirette ....");
#endif
	while(capteurs.readCapteurValue(TIRETTE));

#ifdef DEBUG_MODE
	Serial.print(" - Equipe : ");
#endif
	if (capteurs.readCapteurValue(EQUIPE)) {
#ifdef DEBUG_MODE
		Serial.println("ROUGE");
#endif
	} else {
#ifdef DEBUG_MODE
		Serial.println("BLEU");
#endif
	}

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");
#endif
	startMatch = millis();
	do {
		matchLoop();

		// Gestion du temps
	} while(millis() - startMatch <= TPS_MATCH);
	robotManager.stop();

#ifdef DEBUG_MODE
	Serial.println(" == FIN DU MATCH ==");
#endif
	// Attente du temps de démarrage de la fin du match
	while(millis() - startMatch <= START_GONFLAGE);
	endMatch();

	// Action de clignotement de la la led built-in pour montrer que la programme fonctionne toujours.
	while(true) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(1000);
		digitalWrite(LED_BUILTIN, LOW);
		delay(1000);
	}
}

// Methode de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
#ifdef DEBUG_MODE
	Serial.begin(115200);
	Serial.println(" == INITIALISATION ROBOT RECYCLE ==");
#endif

	// ---------- //
	// Config I2C //
	// ---------- //
	Wire.begin();
#ifdef DEBUG_MODE
	Serial.println(" - I2C [OK] (Master)");
#endif

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// ------------- //
	// Robot manager //
	// ------------- //
	robotManager.init();
	robotManager.setSampleTime(TIME_ASSERV_MS);
	robotManager.setPIDDistance(K_P_DISTANCE, K_I_DISTANCE, K_D_DISTANCE);
	robotManager.setPIDOrientation(K_P_ORIENTATION, K_I_ORIENTATION, K_D_ORIENTATION);
	robotManager.setRampAcc(RAMPE_ACC_DISTANCE, RAMPE_ACC_ORIENTATION);
	robotManager.setRampDec(RAMPE_DEC_DISTANCE, RAMPE_DEC_ORIENTATION);

	// -- //
	// IO //
	// -- //
	pinMode(LED_BUILTIN, OUTPUT);
	capteurs.setPinForCapteur(TIRETTE, TIRETTE_PIN, true);
	capteurs.setPinForCapteur(EQUIPE, EQUIPE_PIN);
#ifdef DEBUG_MODE
	Serial.println(" - I/O [OK]");
#endif
}

// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé.
void matchLoop() {
	// Processing de l'asservissement.
	//robotManager.process();
}

// Méthode appelé pour la fin du match.
void endMatch() {
#ifdef DEBUG_MODE
	Serial.println(" == GONFLAGE BALLONS ==");
#endif
	// TODO : Gonflage ballon
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

/*
 * Méthode retournant l'information de présence d'un obstacle (adversaire ???)
 */
boolean hasObstacle() {
	// TODO : Liste de capteurs indiquant que le robot est face à un autre.
	return false;
}
