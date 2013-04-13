#include <Arduino.h>
#include <Wire.h>
#include <robot/utils/Convertion.h>
#include <robot/RobotManager.h>
#include <robot/system/servos/SD21.h>
#include <robot/system/motors/MD22.h>

#include "define.h"

// Prototype des fonctions
void setup();
void loop();

// Classe de convertion
//Convertion Conv = Convertion(4.044, 11.36);

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
//RobotManager RM = RobotManager();
//SD21 servoManager;
//MD22 mot;

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customisé tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

	while(true) {
		// Boucle infinie pour le fonctionnement.
		loop();
	}
}

// Method de configuration pour le fonctionnement du programme
void setup() {
	// ------------------------------------------------------------- //
	// Initialisation du port série en debug seulement (cf define.h) //
	// ------------------------------------------------------------- //
	Serial.begin(115200);
	Serial.println(" == INITIALISATION ROBOT RECYCLE ==");
#ifdef DEBUG_MODE
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
	//servoManager = SD21();

	pinMode(LED_BUILTIN, OUTPUT);
}

// Méthode appelé encore et encore, tant que la carte reste alimenté.
void loop() {
	//servoManager.setPositionAndSpeed(1, 10, 600);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(5000);

	//servoManager.setSpeed(1, 20);
	//servoManager.setPosition(1, 1800);
	digitalWrite(LED_BUILTIN, LOW);
	delay(5000);

	// Processing de l'asservissement.
	//RM.process();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //
