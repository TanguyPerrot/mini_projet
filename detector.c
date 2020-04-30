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

#define AVANCE 			100  //le robot avance
#define GAUCHE 			200  //le robot tourne à gauche
#define DROITE 			300  //le robot tourne à droite
#define STOP 			400  //le robot s'arrête
#define AVANT_DROITE	0    //proximity sensor avant droite
#define DIAG_DROITE		1    //proximity sensor diagonale droite
#define LAT_DROITE		2    //proximity sensor latéral droite
#define LAT_GAUCHE		5    //proximity sensor latéral gauche
#define DIAG_GAUCHE		6    //proximity sensor diagonal gauche
#define AVANT_GAUCHE	7    //proximity sensor avant gauche
#define MUR				150  //distance à laquelle il detecte un mur
#define VIDE			80   //distance à laquelle il detecte du vide
#define WAIT			200  //temps en milliseconde
#define CINQ_DEGREE		29   //temps en ms pour tourner de 5°
#define UN_CM			130  //temps en ms pour parcourir 1 cm
#define DELTA1_6		20   //intervalle [-20,20] dans lequel le robot reste droit à ±2.5°
#define DELTA2_5		75   //intervalle [-75,75] dans lequel le robot reste au centre à ±0.5cm
#define OKAY			100  //si le robot est stabilisé
#define NOT_OKAY		200	 // si le robot n'est pas stabilisé

	int var0 = 0;
	int var1 = 0;
	int var2 = 0;
	int var5 = 0;
	int var6 = 0;
	int var7 = 0;
	int compteur = 0;
	int stab = 0;

	void guidage(int a)
	{
		switch(a){
		case AVANCE:
			left_motor_set_speed(600);
			right_motor_set_speed(600);
			break;

		case GAUCHE:
			left_motor_set_speed(-600);
			right_motor_set_speed(600);
			break;

		case DROITE:
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
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

		//stabilisateur();

		guidage(AVANCE);

		//ouverture à gauche et mur devant
		if ((var0 > MUR) & (var7 > MUR) & (var5 < VIDE)){
			reglage_angle_gauche(18*CINQ_DEGREE);

			reglage_distance(8*UN_CM);
			var5 = get_prox(LAT_GAUCHE);
			if(var5 > 90){
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
		}
		//ouverture à droite et mur devant
		else if((var0 > MUR) & (var7 > MUR) & (var2 < VIDE)){
			reglage_angle_droite(18*CINQ_DEGREE);

			reglage_distance(8*UN_CM);
			var2 = get_prox(LAT_DROITE);
			if(var2 > 90){
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
		}
		// cul de sac
		else if((var0 > MUR) & (var7 > MUR) & (var5 > MUR) & (var2 > MUR)){
			reglage_angle_gauche(36*CINQ_DEGREE);
			compteur++;
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
		}
		//ouverture à gauche sans mur en face
		else if((var5 < 80) & (var0 < 80) & (var7 < 80)){
			if(compteur == 0){
				reglage_distance(3*UN_CM);
				reglage_angle_gauche(18*CINQ_DEGREE);
				reglage_distance(8*UN_CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
			else{
				compteur--;
				reglage_distance(14*UN_CM);
			}
		}
		//ouverture à droite sans mur en face
		else if((var2 < 80) & (var0 < 80) & (var7 < 80)){
			if(compteur == 0){
				reglage_distance(3*UN_CM);
				reglage_angle_droite(18*CINQ_DEGREE);
				reglage_distance(8*UN_CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
			else{
				compteur--;
				reglage_distance(14*UN_CM);
			}
		}
		else{
			guidage(AVANCE);
		}
	}

	//rotation de robot à gauche
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

	//rotation du robot à droite
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

	//avvance du robot de 2-3 cm
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

		var0 = get_prox(AVANT_DROITE);
		var1 = get_prox(DIAG_DROITE);
		var2 = get_prox(LAT_DROITE);
		var5 = get_prox(LAT_GAUCHE);
		var6 = get_prox(DIAG_GAUCHE);
		var7 = get_prox(AVANT_GAUCHE);

		delta1_6 = var1-var6;
		delta2_5 = var2-var5;



		if(delta1_6 > DELTA1_6){
			while(delta1_6 > DELTA1_6){
				guidage(GAUCHE);
				//reglage_angle_gauche(0.5*CINQ_DEGREE);
				var1 = get_prox(DIAG_DROITE);
				var6 = get_prox(DIAG_GAUCHE);
				delta1_6 = var1-var6;
			}
			//reglage_angle_gauche(0.5*CINQ_DEGREE);
			/*var1 = get_prox(DIAG_DROITE);
			var6 = get_prox(DIAG_GAUCHE);
			delta1_6 = var1-var6;
			while(delta1_6 > 10){
				guidage(GAUCHE);
				var1 = get_prox(DIAG_DROITE);
				var6 = get_prox(DIAG_GAUCHE);
				delta1_6 = var1-var6;

			}*/
		}
		else if (delta1_6 < -DELTA1_6){
			while(delta1_6 < -DELTA1_6){
				guidage(DROITE);
				//reglage_angle_droite(0.5*CINQ_DEGREE);
				var1 = get_prox(DIAG_DROITE);
				var6 = get_prox(DIAG_GAUCHE);
				delta1_6 = var1-var6;

			}
			//reglage_angle_droite(0.5*CINQ_DEGREE);
			/*var1 = get_prox(DIAG_DROITE);
			var6 = get_prox(DIAG_GAUCHE);
			delta1_6 = var1-var6;
			while(delta1_6 < -10){
				guidage(DROITE);
				var1 = get_prox(DIAG_DROITE);
				var6 = get_prox(DIAG_GAUCHE);
				delta1_6 = var1-var6;
			}*/
		}
		else if (delta2_5 < -DELTA2_5){
			reglage_angle_droite(10*CINQ_DEGREE);

			reglage_distance(0.75*WAIT);

			reglage_angle_gauche(10*CINQ_DEGREE);
		}
		else if (delta2_5 > DELTA2_5){

			reglage_angle_gauche(10*CINQ_DEGREE);

			reglage_distance(0.75*WAIT);

			reglage_angle_droite(10*CINQ_DEGREE);
		}
		else{
			stab = OKAY;

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



