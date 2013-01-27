#include <Arduino.h>
#include <Wire.h>

#include "define.h"

// Prototype des fonctions
void setup();
void loop();

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
	// Initialisation du port s�rie en debug seulement (cf define.h)
	if (DEBUG_MODE == 1) {
		Serial.begin(115200);
		Serial.println(" == INITIALISATION GRAND ROBOT ==");
	}

	Wire.begin();
	if (DEBUG_MODE == 1) {
		Serial.println(" - I2C [OK] (Master)");
	}
}

// M�thode appel� encore et encore, tant que la carte reste aliment�.
void loop() {
	// TODO : IA pour le robot

	// Processing de l'asservissement.
	//RM.process();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //
