/*
 * detector.c
 *
 *  Created on: 16 avr. 2020
 *      Author: Tanguy Perrot
 */
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

	static int var0 = 0;
	static int var1 = 0;
	static int var2 = 0;
	static int var5 = 0;
	static int var6 = 0;
	static int var7 = 0;
	static int compteur = 0;
	static bool stab = 0;

	void guidage(int a)
	{
		switch(a){
		case AVANCE:
			left_motor_set_speed(MOTOR);
			right_motor_set_speed(MOTOR);
			break;

		case GAUCHE:
			left_motor_set_speed(-MOTOR);
			right_motor_set_speed(MOTOR);
			break;

		case DROITE:
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

	guidage(AVANCE);

	//Open on the left and an obstacle in the front
	if ((var0 > MUR) & (var7 > MUR) & (var5 < VIDE)){
		reglage_angle_gauche(18.5*CINQ_DEGREE);

		reglage_distance(10*UN_CM);
		var5 = get_prox(LAT_GAUCHE);
		if(var5 > MUR_STAB){
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
		}
	}
	//Open on the right and an obstacle in the front
	else if((var0 > MUR) & (var7 > MUR) & (var2 < VIDE)){
		reglage_angle_droite(19*CINQ_DEGREE);

		reglage_distance(10*UN_CM);
		var2 = get_prox(LAT_DROITE);
		if(var2 > MUR_STAB){
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
			if(compteur > 0){
				reglage_angle_droite(CINQ_DEGREE);
			}
		}
	}
	// cul de sac
	else if((var0 > MUR_CDS) & (var7 > MUR_CDS) & (var5 > MUR_CDS) & (var2 > MUR_CDS)){
		reglage_angle_gauche(36*CINQ_DEGREE);
		compteur++;
		stab = NOT_OKAY;
		while(stab != OKAY){
			stabilisateur();
		}
	}
	//Open on the left without an obstacle in the front
	else if((var5 < VIDE) & (var0 < VIDE) & (var7 < VIDE) & (var2 > (MUR_OMBRE))){
		if(compteur == 0){
			reglage_distance(3*UN_CM);
			reglage_angle_gauche(18.5*CINQ_DEGREE);
			reglage_distance(8*UN_CM);
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
		}
		else{
			compteur--;
			reglage_distance(14*UN_CM);
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
		}
	}
	//Open on the right without an obstacle in the front
	else if((var2 < VIDE) & (var0 < VIDE) & (var7 < VIDE) & (var5 > (MUR_OMBRE))){
		if(compteur == 0){
			reglage_distance(3*UN_CM);
			reglage_angle_droite(18.5*CINQ_DEGREE);
			reglage_distance(8*UN_CM);
			stab = NOT_OKAY;
			while(stab != OKAY){
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
		guidage(AVANCE);
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
		guidage(AVANCE);

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
				guidage(GAUCHE);
			}
			else{
				reglage_angle_gauche(0.1*CINQ_DEGREE);
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
				guidage(DROITE);
			}
			else{
				reglage_angle_droite(0.1*CINQ_DEGREE);
			}
			var1 = get_prox(DIAG_DROITE);
			var6 = get_prox(DIAG_GAUCHE);
			delta1_6 = var1-var6;
			}
	}

	//Adjusment of the robot when it is not centered on its path: closer to the left wall
	else if ((delta2_5 < -DELTA2_5)){
		reglage_angle_droite(10*CINQ_DEGREE);

		reglage_distance(0.75*WAIT);

		reglage_angle_gauche(10*CINQ_DEGREE);
	}

	//Adjusment of the robot when it is not centered on its path: closer to the right wall
	else if ((delta2_5 > DELTA2_5)){

		reglage_angle_gauche(10*CINQ_DEGREE);

		reglage_distance(0.75*WAIT);

		reglage_angle_droite(10*CINQ_DEGREE);
	}
	else {
		stab = OKAY;
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



