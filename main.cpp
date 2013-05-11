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

#ifdef DEBUG_MODE
	Serial.println(" == DEBUT DU MATCH ==");
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

	// Gestion des servos de bras en fonction de la position en X
	RobotPosition currentPosition = robotManager.getPosition();
	int startOffset = cdxStartOffset;
	if (team == BLEU) {
		startOffset = startOffset * -1;
	}
	int stopOffset = cdxStopOffset;
	if (team == ROUGE) {
		stopOffset = stopOffset * -1;
	}
	if (gestEtapes > 6 && gestEtapes < 14) { // Les étapes
		if (team == BLEU) {
			if (currentPosition.getY() < Conv.mmToPulse(230) // La zone CDX en Y
					&& servoOpen == BRAS_FERME
					&& (
						(currentPosition.getX() > Conv.mmToPulse(cdx1Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx1Center + stopOffset)) // CDX 1

						|| (currentPosition.getX() > Conv.mmToPulse(cdx2Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx2Center + stopOffset)) // CDX 2

						|| (currentPosition.getX() > Conv.mmToPulse(cdx3Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx3Center + stopOffset)) // CDX 3

						|| (currentPosition.getX() > Conv.mmToPulse(cdx4Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx4Center + stopOffset)) // CDX 4
				)
			) {
				// Ouverture
				servoManager.setPosition(SERVO_BRAS_DROIT, BRAS_DROIT_CDX_HAUT);
				servoOpen = BRAS_OUVERT;

			} else if (currentPosition.getY() < Conv.mmToPulse(230) // La zone CDX en Y
					&& servoOpen == BRAS_OUVERT
					&& (
						(currentPosition.getX() > Conv.mmToPulse(cdx1Center + stopOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx2Center + startOffset)) // CDX 1 -> CDX 2

						|| (currentPosition.getX() > Conv.mmToPulse(cdx2Center + stopOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx3Center + startOffset)) // CDX 2 -> CDX 3

						|| (currentPosition.getX() > Conv.mmToPulse(cdx3Center + stopOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx4Center + startOffset)) // CDX 3 -> CDX 4
				)

			) {
				// Fermeture
				brasHome();
				servoOpen = BRAS_FERME;
			}
		} else {
			if (currentPosition.getY() < Conv.mmToPulse(250) // La zone CDX en Y
					&& servoOpen == BRAS_FERME
					&& (
						(currentPosition.getX() < Conv.mmToPulse(cdx1Center + startOffset)
								&& currentPosition.getX() > Conv.mmToPulse(cdx1Center + stopOffset)) // CDX 1

						|| (currentPosition.getX() < Conv.mmToPulse(cdx2Center + startOffset)
								&& currentPosition.getX() > Conv.mmToPulse(cdx2Center + stopOffset)) // CDX 2

						|| (currentPosition.getX() < Conv.mmToPulse(cdx3Center + startOffset)
								&& currentPosition.getX() > Conv.mmToPulse(cdx3Center + stopOffset)) // CDX 3

						|| (currentPosition.getX() < Conv.mmToPulse(cdx4Center + startOffset)
								&& currentPosition.getX() > Conv.mmToPulse(cdx4Center + stopOffset)) // CDX 4
				)
			) {
				// Ouverture
				servoManager.setPosition(SERVO_BRAS_GAUCHE, BRAS_GAUCHE_CDX_HAUT);
				servoOpen = BRAS_OUVERT;

			} else if (currentPosition.getY() < Conv.mmToPulse(250) // La zone CDX en Y
					&& servoOpen == BRAS_OUVERT
					&& (
						(currentPosition.getX() > Conv.mmToPulse(cdx1Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx2Center + stopOffset)) // CDX 2 -> CDX 1

						|| (currentPosition.getX() > Conv.mmToPulse(cdx2Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx3Center + stopOffset)) // CDX 3 -> CDX 2

						|| (currentPosition.getX() > Conv.mmToPulse(cdx3Center + startOffset)
								&& currentPosition.getX() < Conv.mmToPulse(cdx4Center + stopOffset)) // CDX 4 -> CDX 3
				)

			) {
				// Fermeture
				brasHome();
				servoOpen = BRAS_FERME;
			}
		}
	} else {
		if (servoOpen) {
			brasHome();
			servoOpen = BRAS_FERME;
		}
	}
}

void nextEtape(){
	RobotConsigne rc = RobotConsigne();
	rc.setType(CONSIGNE_POLAIRE);
	ConsignePolaire p = ConsignePolaire();
	p.enableFrein();
	p.setVitesseDistance(600);
	p.setVitesseOrientation(600);

	switch (gestEtapes) {
	case 0 :
			p.setConsigneDistance(Conv.mmToPulse(141));
			p.setConsigneOrientation(0);
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;
	case 1 :
		    p.setConsigneDistance(0);
		    if (team == BLEU) {
		    	p.setConsigneOrientation(-Conv.degToPulse(45));
		    } else {
		    	p.setConsigneOrientation(Conv.degToPulse(45));
		    }
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;
	case 2 :
		    p.setConsigneDistance(Conv.mmToPulse(320));
			p.setConsigneOrientation(0);
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;
	case 3 :
			p.setConsigneDistance(0);
		    if (team == BLEU) {
		    	p.setConsigneOrientation(-Conv.degToPulse(90));
		    } else {
		    	p.setConsigneOrientation(Conv.degToPulse(90));
		    }
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;
	case 4 :
			p.setConsigneDistance(Conv.mmToPulse(130));
			p.setConsigneOrientation(0);
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;
	case 5 :
			p.setConsigneDistance(0);
		    if (team == BLEU) {
		    	p.setConsigneOrientation(Conv.degToPulse(90));
		    } else {
		    	p.setConsigneOrientation(-Conv.degToPulse(90));
		    }
			rc.setConsignePolaire(p);
			robotManager.setConsigneTable(rc);
			gestEtapes++;
			break;

	case 6 :
	case 8 :
	case 10 :
	case 12 :
		gestEtapes++;
		break;

	case 7:
	case 9:
	case 11:
		p.setConsigneDistance(Conv.mmToPulse(600));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 13:
	case 15:
	case 20:
	case 22:
		p.setConsigneDistance(0);
		if (team == BLEU) {
			p.setConsigneOrientation(Conv.degToPulse(90));
		} else {
			p.setConsigneOrientation(-Conv.degToPulse(90));
		}
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		if (gestEtapes == 15 || gestEtapes == 22) {
			servoManager.setPosition(SERVO_PORTE_DROITE, PORTE_DROITE_OPEN);
			servoManager.setPosition(SERVO_PORTE_GAUCHE, PORTE_GAUCHE_OPEN);
		}

		gestEtapes++;
		break;

	case 14:
		p.setConsigneDistance(Conv.mmToPulse(455));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 16:
	case 23:
		p.setConsigneDistance(Conv.mmToPulse(1900));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 17:
	case 24:
		p.setConsigneDistance(-Conv.mmToPulse(130));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 18:
	case 25:
		p.setConsigneDistance(0);
		p.setConsigneOrientation(Conv.degToPulse(180));
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		closeDoors();
		gestEtapes++;
		break;

	case 19:
		p.setConsigneDistance(Conv.mmToPulse(1770));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 21:
		p.setConsigneDistance(Conv.mmToPulse(320));
		p.setConsigneOrientation(0);
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 26:
		p.setConsigneDistance(Conv.mmToPulse(500));
		if (team == BLEU) {
			p.setConsigneOrientation(-Conv.degToPulse(90));
		} else {
			p.setConsigneOrientation(Conv.degToPulse(90));
		}
		rc.setConsignePolaire(p);
		robotManager.setConsigneTable(rc);
		gestEtapes++;
		break;

	case 27:
			p.setConsigneDistance(0);
			p.setConsigneOrientation(Conv.degToPulse(360));
			rc.setConsignePolaire(p);
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
