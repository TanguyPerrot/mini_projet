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

#define AVANCE 			100  //le robot avance
#define GAUCHE 			200  //le robot tourne � gauche
#define DROITE 			300  //le robot tourne � droite
#define STOP 			400  //le robot s'arr�te
#define MOTOR			600
#define AVANT_DROITE	0    //proximity sensor avant droite
#define DIAG_DROITE		1    //proximity sensor diagonale droite
#define LAT_DROITE		2    //proximity sensor lat�ral droite
#define LAT_GAUCHE		5    //proximity sensor lat�ral gauche
#define DIAG_GAUCHE		6    //proximity sensor diagonal gauche
#define AVANT_GAUCHE	7    //proximity sensor avant gauche
#define MUR				150  //distance � laquelle il detecte un mur
#define MUR_CDS			120	 //distance � laquelle il d�tecte un mur pour le cul de sac
#define MUR_OMBRE		100
#define VIDE			80   //distance � laquelle il detecte du vide
#define MUR_STAB		90
#define WAIT			200  //temps en milliseconde
#define CINQ_DEGREE		29   //temps en ms pour tourner de 5�
#define UN_CM			130  //temps en ms pour parcourir 1 cm
#define DELTA1_6		10   //intervalle [-10,10] dans lequel le robot reste droit � �1.5�
#define DELTA1_6_GRAND  30	 //intervalle [-30,30] dans lequel le robot reste droit � �3.5�
#define DELTA2_5		75   //intervalle [-75,75] dans lequel le robot reste au centre � �0.5cm
#define OKAY			1  	 //si le robot est stabilis�
#define NOT_OKAY		0	 // si le robot n'est pas stabilis�

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

		//ouverture � gauche et mur devant
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
		//ouverture � droite et mur devant
		else if((var0 > MUR) & (var7 > MUR) & (var2 < VIDE)){
			reglage_angle_droite(18.5*CINQ_DEGREE);

			reglage_distance(10*UN_CM);
			var2 = get_prox(LAT_DROITE);
			if(var2 > MUR_STAB){
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
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
		//ouverture � gauche sans mur en face
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
				stabilisateur();
			}
		}
		//ouverture � droite sans mur en face
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
		//sorti du labyrinth
		else if((var0 < VIDE) & (var7 < VIDE) & (var1 < VIDE) & (var2 < VIDE) & (var5 < VIDE) & (var6 < VIDE)){
			finish();
		}
		else{
			guidage(AVANCE);
		}
	}

	//rotation de robot � gauche
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

	//rotation du robot � droite
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

		var1 = get_prox(DIAG_DROITE);
		var2 = get_prox(LAT_DROITE);
		var5 = get_prox(LAT_GAUCHE);
		var6 = get_prox(DIAG_GAUCHE);

		delta1_6 = var1-var6;
		delta2_5 = var2-var5;


		//si le robot est trop tourn� vers la droite
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
			//reglage_angle_droite(0.5*CINQ_DEGREE);
		}

		//si le robot est trop tourn� vers la gauche
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
			//reglage_angle_droite(CINQ_DEGREE);
		}

		//si le robot n'est pas centr� entre les deux murs, proche du mur gauche
		else if (delta2_5 < -DELTA2_5){
			reglage_angle_droite(10*CINQ_DEGREE);

			reglage_distance(0.75*WAIT);

			reglage_angle_gauche(10*CINQ_DEGREE);
		}

		//si le robot n'est pas centr� entre les deux murs, proche du mur droit
		else if (delta2_5 > DELTA2_5){

			reglage_angle_gauche(10*CINQ_DEGREE);

			reglage_distance(0.75*WAIT);

			reglage_angle_droite(10*CINQ_DEGREE);
		}
		else{
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



