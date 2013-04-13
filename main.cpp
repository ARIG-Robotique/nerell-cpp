#include <Arduino.h>
#include <Wire.h>
#include <robot/system/servos/SD21.h>
#include <robot/utils/Convertion.h>
#include <robot/RobotManager.h>

#include "define.h"

// Prototype des fonctions
void setup();
void loop();

// Classe de convertion
//Convertion Conv = Convertion(4.044, 11.36);

// Classe de gestion du robot (asserv, odom�trie, pathfinding, evittement, etc...)
//RobotManager RM = RobotManager();
SD21 servoManager = SD21();

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

// Point d'entr�e du programme
int main(void) {
	// Initialisation du SDK Arduino. A r��crire si on veut customis� tout le bouzin.
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
	// Initialisation du port s�rie en debug seulement (cf define.h) //
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

	// -- //
	// IO //
	// -- //
	Serial.println(" - I/O [OK]");
}

// M�thode appel� encore et encore, tant que la carte reste aliment�.
void loop() {
	// Processing de l'asservissement.
	//RM.process();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //
