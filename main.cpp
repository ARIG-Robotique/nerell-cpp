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
RobotManager RM = RobotManager();
SD21 servoManager = SD21();
Board2007NoMux capteurs = Board2007NoMux();

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customisé tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");

	Serial.println(" - Attente tirette ....");
	while(capteurs.readCapteurValue(TIRETTE));

	Serial.print(" - Equipe : ");
	if (capteurs.readCapteurValue(EQUIPE)) {
		Serial.println("ROUGE");
	} else {
		Serial.println("BLEU");
	}

	Serial.println(" == DEBUT DU MATCH ==");

	startMatch = millis();
	do {
		matchLoop();

		// Gestion du temps
	} while(millis() - startMatch <= TPS_MATCH);
	RM.stop();

	Serial.println(" == FIN DU MATCH ==");
	endMatch();

	int servo = 1;
	while(true) {
		digitalWrite(LED_BUILTIN, HIGH);
		servoManager.setPosition(servo, 600);
		delay(1000);
		digitalWrite(LED_BUILTIN, LOW);
		servoManager.setPosition(servo, 1800);
		delay(1000);

		servo++;
		if (servo > 3) {
			servo = 1;
		}
	}
}

// Method de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
	Serial.begin(115200);
	Serial.println(" == INITIALISATION ROBOT RECYCLE ==");

	// ---------- //
	// Config I2C //
	// ---------- //
	Wire.begin();
	Serial.println(" - I2C [OK] (Master)");

	// ------------- //
	// Servo manager //
	// ------------- //
	servoManager.printVersion();

	// ------------- //
	// Robot manager //
	// ------------- //
	RM.init();

	// -- //
	// IO //
	// -- //
	pinMode(LED_BUILTIN, OUTPUT);
	capteurs.setPinForCapteur(TIRETTE, TIRETTE_PIN, true);
	capteurs.setPinForCapteur(EQUIPE, EQUIPE_PIN);
	Serial.println(" - I/O [OK]");
}

// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé.
void matchLoop() {
	// Processing de l'asservissement.
	RM.process();
}

// Méthode appelé pour la fin du match.
void endMatch() {
	// TODO : Gonflage ballon
	Serial.println(" == GONFLAGE BALLONS ==");
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //
