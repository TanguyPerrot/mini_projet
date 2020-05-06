#include "ch.h"
#include "hal.h"
#include <detector.h>
#include <main.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>
#include <chprintf.h>
#include <math.h>
#include <stdbool.h>

#define FORWARD			100  //Robot goes forward
#define LEFT 			200  //Robot turns to the left
#define RIGHT 			300  //Robot turns to the right
#define STOP 			400  //Robot stops
#define MOTOR			600
#define AVANT_DROITE	0    //proximity sensor avant droite
#define DIAG_DROITE		1    //proximity sensor diagonale droite
#define LAT_DROITE		2    //proximity sensor latéral droite
#define LAT_GAUCHE		5    //proximity sensor latéral gauche
#define DIAG_GAUCHE		6    //proximity sensor diagonal gauche
#define AVANT_GAUCHE	7    //proximity sensor avant gauche
#define MUR				150  //distance à laquelle il detecte un mur
#define MUR_CDS			120	 //distance à laquelle il détecte un mur pour le cul de sac
#define MUR_OMBRE		100
#define VIDE			80   //distance à laquelle il detecte du vide
#define MUR_STAB		90
#define WAIT			200  //temps en milliseconde
#define CINQ_DEGREE		29   //temps en ms pour tourner de 5°
#define UN_CM			130  //temps en ms pour parcourir 1 cm
#define DELTA1_6		10   //intervalle [-10,10] dans lequel le robot reste droit à ±1.5°
#define DELTA1_6_GRAND  30	 //intervalle [-30,30] dans lequel le robot reste droit à ±3.5°
#define DELTA2_5		75   //intervalle [-75,75] dans lequel le robot reste au centre à ±0.5cm
#define OKAY			1  	 //si le robot est stabilisé
#define NOT_OKAY		0	 // si le robot n'est pas stabilisé

static int var0 = 0;
static int var1 = 0;
static int var2 = 0;
static int var5 = 0;
static int var6 = 0;
static int var7 = 0;
static int compteur = 0;
static bool stablizer = 0;

void guidage(int a)
{
	switch(a){
	case FORWARD:
		left_motor_set_speed(MOTOR);
		right_motor_set_speed(MOTOR);
		break;

	case LEFT:
		left_motor_set_speed(-MOTOR);
		right_motor_set_speed(MOTOR);
		break;

	case RIGHT:
		left_motor_set_speed(MOTOR);
		right_motor_set_speed(-MOTOR);
		break;
		case STOP:
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		break;
		default:
		break;
	}
}

void obstacle()
{

	if(compteur == 0){
		set_led(LED1, 0);
	}
	else{
		set_led(LED1, 1);
	}
	var0 = get_prox(AVANT_DROITE);
	var1 = get_prox(DIAG_DROITE);
	var2 = get_prox(LAT_DROITE);
	var5 = get_prox(LAT_GAUCHE);
	var6 = get_prox(DIAG_GAUCHE);
	var7 = get_prox(AVANT_GAUCHE);
	guidage(FORWARD);

	//Open on the left and an obstacle in the front
	if ((var0 > MUR) & (var7 > MUR) & (var5 < VIDE)){
		reglage_angle_gauche(18.5*CINQ_DEGREE);
		reglage_distance(10*UN_CM);
		var5 = get_prox(LAT_GAUCHE);
		if(var5 > MUR_STAB){
			stablizer = NOT_OKAY;
			while(stablizer != OKAY){
				stabilisateur();
			}
		}
	}

	//Open on the right and an obstacle in the front
	else if((var0 > MUR) & (var7 > MUR) & (var2 < VIDE)){
		reglage_angle_droite(18.5*CINQ_DEGREE);
		reglage_distance(10*UN_CM);
		var2 = get_prox(LAT_DROITE);
		if(var2 > MUR_STAB){
			stablizer = NOT_OKAY;
			while(stablizer != OKAY){
				stabilisateur();
			}
		}
	}

	//Cul-de-sac
	else if((var0 > MUR_CDS) & (var7 > MUR_CDS) & (var5 > MUR_CDS) & (var2 > MUR_CDS)){
		reglage_angle_gauche(36*CINQ_DEGREE);
		compteur++;
		stablizer = NOT_OKAY;
		while(stablizer != OKAY){
			stabilisateur();
		}
	}

	//Open on the left without an obstacle in the front
	else if((var5 < VIDE) & (var0 < VIDE) & (var7 < VIDE) & (var2 > (MUR_OMBRE))){
		if(compteur == 0){
			reglage_distance(3*UN_CM);
			reglage_angle_gauche(18.5*CINQ_DEGREE);
			reglage_distance(8*UN_CM);
			stablizer = NOT_OKAY;
			while(stablizer != OKAY){
					stabilisateur();
			}
		}
		else{
			compteur--;
			reglage_distance(14*UN_CM);
			stabilisateur();
		}
	}

	//Open on the right without an obstacle in the front
	else if((var2 < VIDE) & (var0 < VIDE) & (var7 < VIDE) & (var5 > (MUR_OMBRE))){
		if(compteur == 0){
			reglage_distance(3*UN_CM);
			reglage_angle_droite(18.5*CINQ_DEGREE);
			reglage_distance(8*UN_CM);
			stablizer = NOT_OKAY;
			while(stablizer != OKAY){
				stabilisateur();
			}
		}
		else{
			compteur--;
			reglage_distance(14*UN_CM);
			stabilisateur();
		}
	}

	//Exit of the maze: proximity sensors detect nothing
	else if((var0 < VIDE) & (var7 < VIDE) & (var1 < VIDE) & (var2 < VIDE) & (var5 < VIDE) & (var6 < VIDE)){
		finish();
	}
	else{
		guidage(FORWARD);
	}
}

//Rotation of the robot to the left
void reglage_angle_gauche(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		guidage(GAUCHE);
	}
	start = chVTGetSystemTime();
	end = start + MS2ST(WAIT);

	while (chVTIsSystemTimeWithin(start, end)){
		guidage(STOP);
	}
}

//Rotation of the robot to the right
void reglage_angle_droite(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		guidage(DROITE);
	}
	start = chVTGetSystemTime();
	end = start + MS2ST(WAIT);

	while (chVTIsSystemTimeWithin(start, end)){
		guidage(STOP);
	}
}

//Robot moves forward of n times 2-3 cm
void reglage_distance(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		guidage(FORWARD);
	}
}

void stabilisateur()
{
	int delta1_6;
	int	delta2_5;

	var1 = get_prox(DIAG_DROITE);
	var2 = get_prox(LAT_DROITE);
	var5 = get_prox(LAT_GAUCHE);
	var6 = get_prox(DIAG_GAUCHE);

	delta1_6 = var1-var6;
	delta2_5 = var2-var5;


	//Adjusment of the angle of the robot when it is turned more to the right
	if(delta1_6 > DELTA1_6){
		while(delta1_6 > DELTA1_6){
			if(delta1_6 > DELTA1_6_GRAND){
				reglage_angle_gauche(0.5*CINQ_DEGREE);
			}
			else{
				reglage_angle_gauche(0.2*CINQ_DEGREE);
			}
			var1 = get_prox(DIAG_DROITE);
			var6 = get_prox(DIAG_GAUCHE);
			delta1_6 = var1-var6;
		}
	}

	//Adjusment of the angle of the robot when it is turned more to the left
	else if (delta1_6 < -DELTA1_6){
		while(delta1_6 < -DELTA1_6){
			if(delta1_6 < -DELTA1_6_GRAND){
				reglage_angle_droite(0.5*CINQ_DEGREE);
			}
			else{
				reglage_angle_droite(0.2*CINQ_DEGREE);
			}
			var1 = get_prox(DIAG_DROITE);
			var6 = get_prox(DIAG_GAUCHE);
			delta1_6 = var1-var6;
			}
	}

	//Adjusment of the robot when it is not centered on its path: closer to the left wall
	else if (delta2_5 < -DELTA2_5){
		reglage_angle_droite(10*CINQ_DEGREE);

		reglage_distance(0.75*WAIT);

		reglage_angle_gauche(10*CINQ_DEGREE);
	}

	//Adjusment of the robot when it is not centered on its path: closer to the right wall
	else if (delta2_5 > DELTA2_5){

		reglage_angle_gauche(10*CINQ_DEGREE);

		reglage_distance(0.75*WAIT);

		reglage_angle_droite(10*CINQ_DEGREE);
	}
	else{
		stablizer = OKAY;
	}
}


void finish()
{
	reglage_distance(5*UN_CM);
	while(1){
		guidage(DROITE);
		set_body_led(OKAY);
	}
}
void test_stab()
{
	var0 = get_prox(0);
	var1 = get_prox(1);
	var2 = get_prox(2);
	var5 = get_prox(5);
	var6 = get_prox(6);
	var7 = get_prox(7);
	chprintf((BaseSequentialStream *)&SD3, "%d %d %d %d\r\n", var1, var6, var2, var5);
}
