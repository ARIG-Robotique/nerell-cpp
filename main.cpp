#include <Arduino.h>
#include <Wire.h>
#include <robot/system/capteurs/Board2007NoMux.h>
#include <robot/system/capteurs/CapteurDefine.h>
#include <robot/system/servos/SD21.h>
#include <robot/utils/Convertion.h>
#include <robot/RobotManager.h>

#include "define.h"

// Prototype des fonctions principale
void setup();
void matchLoop();
void startFunnyAction();
void endMatch();
void nextEtape();

// Prototype des fonctions business
void heartBeat();
void brasHome();
void closeDoors();
void openVanne();
void closeVanne();
void startGonfleur();
void stopGonfleur();
boolean hasObstacle();

// Heartbeat variables
int heartTimePrec;
int heartTime;
boolean heart;

// Valeur de tempo servo
unsigned int servoTime;
const int tempoServo = 700;
byte servoOpen;


// Classe de convertion
Convertion Conv = Convertion(3.97887357729738, 11.1805555555556);

// Classe de gestion du robot (asserv, odométrie, pathfinding, evittement, etc...)
RobotManager robotManager = RobotManager();
SD21 servoManager = SD21();
Board2007NoMux capteurs = Board2007NoMux();

// Gestion des étapes
int gestEtapes;


// Position des zones cadeaux pour les servos
const int cdx1Center = 600;
const int cdx2Center = 1200;
const int cdx3Center = 1800;
const int cdx4Center = 2400;
const int cdxStartOffset = 150;
const int cdxStopOffset = 220;

// ------------------------ //
// Configuration des rampes //
// ------------------------ //
const int rampAccDistance = 300.0; // en mm/s2
const int rampDecDistance = 100.0; // en mm/s2

const double rampAccOrientation = 300.0; // en mm/s2
const double rampDecOrientation = 200.0; // en mm/s2

// -------------- //
// Parametres PID //
// -------------- //
const double kpDistance = 1.10;
const double kiDistance = 0.80;
const double kdDistance = 0.00;

const double kpOrientation = 1.20;
const double kiOrientation = 0.80;
const double kdOrientation = 0.00;

// Variable pour l'équipe
byte team;

// ------------------------------------------------------- //
// ------------------------- MAIN ------------------------ //
// ------------------------------------------------------- //

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

	// Tempo attente pour boot autres cartes
	delay(4000);

	// ------------- //
	// Servo manager //
	// ------------- //
#ifdef DEBUG_MODE
	servoManager.printVersion();
#endif

	// Configuration des vitesses
	servoManager.setSpeed(SERVO_BRAS_DROIT, SPEED_BRAS);
	servoManager.setSpeed(SERVO_BRAS_GAUCHE, SPEED_BRAS);
	servoManager.setSpeed(SERVO_PORTE_DROITE, SPEED_PORTE);
	servoManager.setSpeed(SERVO_PORTE_GAUCHE, SPEED_PORTE);

	// ------------- //
	// Robot manager //
	// ------------- //
	robotManager.init();
	robotManager.setSampleTime(TIME_ASSERV_MS);
	robotManager.setPIDDistance(kpDistance, kiDistance, kdDistance);
	robotManager.setPIDOrientation(kpOrientation, kiOrientation, kdOrientation);
	robotManager.setRampAcc(rampAccDistance, rampAccOrientation);
	robotManager.setRampDec(rampDecDistance, rampDecOrientation);
	robotManager.setHasObstacle(hasObstacle);

#ifdef DEBUG_MODE
	Serial.println(" - Robot manager [OK]");
#endif

	// -- //
	// IO //
	// -- //

	// Inputs
	capteurs.setPinForCapteur(TIRETTE, TIRETTE_PIN, true);
	capteurs.setPinForCapteur(EQUIPE, EQUIPE_PIN, true);
	capteurs.setPinForCapteur(LATERAL_ARRIERE_DROIT, LATERAL_ARRIERE_DROIT_PIN, true, true);
	capteurs.setPinForCapteur(LATERAL_ARRIERE_GAUCHE, LATERAL_ARRIERE_GAUCHE_PIN, true, true);
	capteurs.setPinForCapteur(LATERAL_AVANT_DROIT, LATERAL_AVANT_DROIT_PIN, true, true);
	capteurs.setPinForCapteur(LATERAL_AVANT_GAUCHE, LATERAL_AVANT_GAUCHE_PIN, true, true);
	capteurs.setPinForCapteur(ARRIERE_DROIT, ARRIERE_DROIT_PIN, true, true);
	capteurs.setPinForCapteur(ARRIERE_GAUCHE, ARRIERE_GAUCHE_PIN, true, true);
	capteurs.setPinForCapteur(AVANT_DROIT, AVANT_DROIT_PIN, true, true);
	capteurs.setPinForCapteur(AVANT_GAUCHE, AVANT_GAUCHE_PIN, true, true);
#ifdef DEBUG_MODE
	Serial.println(" - Capteurs [OK]");
#endif

	// Outputs
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(GONFLEUR, OUTPUT);
	pinMode(ELECTRO_VANNE, OUTPUT);
#ifdef DEBUG_MODE
	Serial.println(" - Outputs [OK]");
#endif

	// Configuration par défaut des variables
	heartTime = heartTimePrec = millis();
	heart = false;

	brasHome();
	closeDoors();
	stopGonfleur();
	closeVanne();

	// Ini Gestion Etapes
	gestEtapes = 0;
	servoOpen = BRAS_FERME;
}

// Point d'entrée du programme
int main(void) {
	// Initialisation du SDK Arduino. A réécrire si on veut customiser tout le bouzin.
	init();

	// Initialisation de l'application
	setup();

#ifdef DEBUG_MODE
	// Affichage de la couleur de l'équipe
	team = capteurs.readCapteurValue(EQUIPE);
	Serial.print(" ========================= > ");
	Serial.println((team == ROUGE) ? "ROUGE" : "BLEU");

	// Procédure d'initialisation Robot (calage, tirette, etc).
	Serial.println(" == INIT MATCH ==");
	Serial.println(" - Attente tirette ....");
#endif

	if (!capteurs.readCapteurValue(TIRETTE)) {
#ifdef DEBUG_MODE
		Serial.println(" /!\\ La tirette n'est pas présente il faut d'abord la mettre !");
		while(!capteurs.readCapteurValue(TIRETTE));
		delay(1000);
#endif
	}

	while(capteurs.readCapteurValue(TIRETTE)) {
		heartBeat();
		if (Serial.available()) {
			if (Serial.read() == 's') { // La touche s de la liaison série est égal à la tirette
				break;
			}
		}
	}

	// Démarrage du comptage
	unsigned long startMatch = millis();
	unsigned long t;

	// Reset des valeurs codeurs lors des différents mouvements de positionnement
	robotManager.resetEncodeurs();

	team = capteurs.readCapteurValue(EQUIPE);
#ifdef DEBUG_MODE
	Serial.print(" - Equipe : ");
#endif
	if (team == ROUGE) {
#ifdef DEBUG_MODE
		Serial.println("ROUGE");
#endif
		robotManager.setPosition(Conv.mmToPulse(2850), Conv.mmToPulse(250), Conv.degToPulse(135));
	} else {
#ifdef DEBUG_MODE
		Serial.println("BLEU");
#endif
		robotManager.setPosition(Conv.mmToPulse(150), Conv.mmToPulse(250), Conv.degToPulse(45));
	}

	// Pour tester //
	// TODO : A supprimer
	//robotManager.setPosition(0, 0, 0);
	robotManager.setPosition(Conv.mmToPulse(150), Conv.mmToPulse(150), 0);

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");

	// En tête de log
	Serial.println("X;Y;A;Type;Cons. Dist.;Cons. Orient.;PID Dist. setPoint;PID Dist. In;PID Dist. sumErr;PID Dist. Out;PID O setPoint;PID O In;PID O sumErr;PID O Out;Approche;Atteint");
#endif

	// Test avec une consigne linéaire.
	boolean g = false;
	do {
		heartBeat();
		matchLoop();

		// Gestion du temps
		t = millis();

		if (t - startMatch >= PREPARE_GONFLAGE && !g) {
			startGonfleur();
			g = true;
		}
	} while(t - startMatch <= TPS_MATCH);
	robotManager.stop();

#ifdef DEBUG_MODE
	Serial.println(" == FIN DU MATCH ==");
#endif
	// Attente du temps de démarrage de la fin du match
	while(millis() - startMatch <= START_GONFLAGE);
	startFunnyAction();
	while(millis() - startMatch <= END_TOUT);
	endMatch();

	// Action de clignotement de la la led built-in pour montrer que la programme fonctionne toujours.
	while(true) {
		heartBeat();
	}
}

// ---------------------------------------------------------------------------- //
// Méthode appelé encore et encore, tant que le temps du match n'est pas écoulé //
// ---------------------------------------------------------------------------- //
void matchLoop() {
	if(robotManager.getTrajetAtteint()) {
		nextEtape();
	}

	// Processing de l'asservissement.
	robotManager.process();
}

void nextEtape(){
	RobotConsigne rc = RobotConsigne();
	ConsignePolaire pol = ConsignePolaire();
	RobotPosition p = RobotPosition();
	robotManager.setVitesse(400.0, 400.0);
	robotManager.setRampAcc(100.0, 100.0);
	robotManager.setRampDec(100.0, 100.0);
	switch (gestEtapes) {
	/*case 0:
		rc.setType(CONSIGNE_POLAIRE);
		pol.setConsigneDistance(Conv.mmToPulse(1000));
		pol.setConsigneOrientation(0);
		pol.enableFrein();
		rc.setConsignePolaire(pol);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;*/

	case 0 :
		p.updatePosition(Conv.mmToPulse(800), Conv.mmToPulse(500), 0);
		rc.setType(CONSIGNE_ODOMETRIE);
		rc.setPosition(p);
		rc.enableFrein();
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;
	case 1 :
		p.updatePosition(Conv.mmToPulse(1300), Conv.mmToPulse(350), 0);
		rc.setType(CONSIGNE_ODOMETRIE);
		rc.setPosition(p);
		rc.enableFrein();
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;
	case 2 :
		p.updatePosition(Conv.mmToPulse(150), Conv.mmToPulse(150), 0);
		rc.setType(CONSIGNE_ODOMETRIE);
		rc.setPosition(p);
		rc.enableFrein();
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;
	}
}

// ----------------------------------- //
// Méthode appelé pour la fin du match //
// ----------------------------------- //
void startFunnyAction() {
#ifdef DEBUG_MODE
	Serial.println(" == START FUNNY ACTION ==");
#endif
	startGonfleur();
	openVanne();
}

void endMatch() {
#ifdef DEBUG_MODE
	Serial.println(" == FIN FUNNY ACTION ==");
#endif
	stopGonfleur();
	closeVanne();
}

// ------------------------------------------------------- //
// -------------------- BUSINESS METHODS ----------------- //
// ------------------------------------------------------- //

/*
 * Méthode pour le battement de coeur
 */
void heartBeat() {
	heartTime = millis();
	if (heartTime - heartTimePrec > 1000) {
		heartTimePrec = heartTime;
		digitalWrite(LED_BUILTIN, (heart) ? HIGH : LOW);
		heart = !heart;
	}
}

/*
 * Méthode pour placer les bras à la maison
 */
void brasHome() {
	servoManager.setPosition(SERVO_BRAS_DROIT, BRAS_DROIT_HOME);
	servoManager.setPosition(SERVO_BRAS_GAUCHE, BRAS_GAUCHE_HOME);
}

/*
 * Méthode pour fermer les portes
 */
void closeDoors() {
	servoManager.setPosition(SERVO_PORTE_DROITE, PORTE_DROITE_CLOSE);
	servoManager.setPosition(SERVO_PORTE_GAUCHE, PORTE_GAUCHE_CLOSE);
}

/*
 * Ouverture de l'electrovanne
 */
void openVanne() {
	digitalWrite(ELECTRO_VANNE, HIGH);
}

/*
 * Fermeture de la vanne
 */
void closeVanne() {
	digitalWrite(ELECTRO_VANNE, LOW);
}

/*
 * Allumage du gonfleur
 */
void startGonfleur() {
	digitalWrite(GONFLEUR, HIGH);
}

/*
 * Arret du gonfleur
 */
void stopGonfleur() {
	digitalWrite(GONFLEUR, LOW);
}

/*
 * Méthode retournant l'information de présence d'un obstacle (adversaire ???)
 */
boolean hasObstacle() {
	// Juste les deux de devant
	boolean obstacle = capteurs.readCapteurValue(AVANT_DROIT)
			|| capteurs.readCapteurValue(AVANT_GAUCHE);
	if (team == BLEU && gestEtapes <= 13) {
		// Les cadeaux sont a droite
		obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_GAUCHE);
	} else if (team == ROUGE && gestEtapes <= 13) {
		// Les cadeaux sont a gauche
		obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_DROIT);
	}

	if (gestEtapes > 13) {
		obstacle = obstacle || capteurs.readCapteurValue(LATERAL_AVANT_GAUCHE)
				|| capteurs.readCapteurValue(LATERAL_AVANT_DROIT);
	}
	return obstacle;
}
