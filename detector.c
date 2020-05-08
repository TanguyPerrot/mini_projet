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
#include <stdint.h>

	static uint16_t tab_prox[8] = {0,0,0,0,0,0,0,0};
	static int delta1_6 = 0;
	static int delta2_5 = 0;
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
	tab_prox[AVANT_DROITE] = get_prox(AVANT_DROITE);
	tab_prox[DIAG_DROITE] = get_prox(DIAG_DROITE);
	tab_prox[LAT_DROITE] = get_prox(LAT_DROITE);
	tab_prox[LAT_GAUCHE] = get_prox(LAT_GAUCHE);
	tab_prox[DIAG_GAUCHE] = get_prox(DIAG_GAUCHE);
	tab_prox[AVANT_GAUCHE] = get_prox(AVANT_GAUCHE);

	guidage(AVANCE);

	//Open on the left and an obstacle in the front
	if ((tab_prox[AVANT_DROITE] > MUR) & (tab_prox[AVANT_GAUCHE] > MUR) & (tab_prox[LAT_GAUCHE] < VIDE)){
		reglage_angle_gauche(QUART_TOUR_G);

		reglage_distance(DIST_MUR_10CM);
		tab_prox[LAT_GAUCHE] = get_prox(LAT_GAUCHE);
		if((tab_prox[LAT_GAUCHE] > MUR_STAB_LAT) & (tab_prox[DIAG_GAUCHE] > MUR_STAB_DIAG)){
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
		}
	}
	//Open on the right and an obstacle in the front
	else if((tab_prox[AVANT_DROITE] > MUR) & (tab_prox[AVANT_GAUCHE] > MUR) & (tab_prox[LAT_DROITE] < VIDE)){
		reglage_angle_droite(QUART_TOUR_D);

		reglage_distance(DIST_MUR_10CM);
		tab_prox[LAT_DROITE] = get_prox(LAT_DROITE);
		if((tab_prox[LAT_DROITE] > MUR_STAB_LAT) & (tab_prox[DIAG_DROITE] > MUR_STAB_DIAG)){
			stab = NOT_OKAY;
			while(stab != OKAY){
				stabilisateur();
			}
			if(compteur > 0){
				reglage_angle_droite(CINQ_DEG);
			}
		}
	}

	//Open on the left without an obstacle in the front
	else if((tab_prox[LAT_GAUCHE] < VIDE) & (tab_prox[AVANT_GAUCHE] < VIDE) & (tab_prox[LAT_DROITE] > (MUR_OMBRE))){
		reglage_distance(DIST_OUVERTURE_4CM);
		if(tab_prox[AVANT_GAUCHE] < VIDE){


			if(compteur == 0){
				reglage_angle_gauche(QUART_TOUR_G);
				reglage_distance(DIST_OUVERTURE_8CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
			else{
				compteur--;
				reglage_distance(DIST_CDS_14CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
		}
	}
	//Open on the right without an obstacle in the front
	else if((tab_prox[LAT_DROITE] < VIDE) & (tab_prox[AVANT_DROITE] < VIDE) & (tab_prox[LAT_GAUCHE] > (MUR_OMBRE))){
		reglage_distance(DIST_OUVERTURE_4CM);
		if(tab_prox[AVANT_DROITE] < VIDE){
			if(compteur == 0){
				reglage_angle_droite(QUART_TOUR_G);
				reglage_distance(DIST_OUVERTURE_8CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
			else{
				compteur--;
				reglage_distance(DIST_CDS_14CM);
				stab = NOT_OKAY;
				while(stab != OKAY){
					stabilisateur();
				}
			}
		}
	}
	// cul de sac
	else if((tab_prox[AVANT_DROITE] > MUR_CDS) & (tab_prox[AVANT_GAUCHE] > MUR_CDS) & (tab_prox[LAT_GAUCHE] > MUR_CDS) & (tab_prox[LAT_DROITE] > MUR_CDS)){
		reglage_angle_gauche(DEMI_TOUR);
		compteur++;
		stab = NOT_OKAY;
		while(stab != OKAY){
			stabilisateur();
		}
	}
	//Exit of the maze: proximity sensors detect nothing
	else if((tab_prox[AVANT_DROITE] < VIDE) & (tab_prox[AVANT_GAUCHE] < VIDE) & (tab_prox[DIAG_DROITE] < VIDE) & (tab_prox[LAT_DROITE] < VIDE) & (tab_prox[LAT_GAUCHE] < VIDE) & (tab_prox[DIAG_GAUCHE] < VIDE)){
		finish();
	}
	else{
		guidage(AVANCE);
	}
	//proximity sensors frequency is about 100 Hz (10ms)
	chThdSleepMilliseconds(10);

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
	end = start + MS2ST(ROT_WAIT);

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
	end = start + MS2ST(ROT_WAIT);

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
	tab_prox[DIAG_DROITE] = get_prox(DIAG_DROITE);
	tab_prox[LAT_DROITE] = get_prox(LAT_DROITE);
	tab_prox[LAT_GAUCHE] = get_prox(LAT_GAUCHE);
	tab_prox[DIAG_GAUCHE] = get_prox(DIAG_GAUCHE);

	delta1_6 = tab_prox[DIAG_DROITE]-tab_prox[DIAG_GAUCHE];
	delta2_5 = tab_prox[LAT_DROITE]-tab_prox[LAT_GAUCHE];


	//Adjusment of the angle of the robot when it is turned more to the right
	if(delta1_6 > DELTA1_6){
		while(delta1_6 > DELTA1_6){
			if(delta1_6 > DELTA1_6_GRAND){
				guidage(GAUCHE);
			}
			else{
				reglage_angle_gauche(DEMI_DEG);
			}
			tab_prox[DIAG_DROITE] = get_prox(DIAG_DROITE);
			tab_prox[DIAG_GAUCHE] = get_prox(DIAG_GAUCHE);
			delta1_6 = tab_prox[DIAG_DROITE]-tab_prox[DIAG_GAUCHE];
		}
	}

	//Adjusment of the angle of the robot when it is turned more to the left
	else if (delta1_6 < -DELTA1_6){
		while(delta1_6 < -DELTA1_6){
			if(delta1_6 < -DELTA1_6_GRAND){
				guidage(DROITE);
			}
			else{
				reglage_angle_droite(DEMI_DEG);
			}
			tab_prox[DIAG_DROITE] = get_prox(DIAG_DROITE);
			tab_prox[DIAG_GAUCHE] = get_prox(DIAG_GAUCHE);
			delta1_6 = tab_prox[DIAG_DROITE]-tab_prox[DIAG_GAUCHE];
			}
	}

	//Adjusment of the robot when it is not centered on its path: closer to the left wall
	else if ((delta2_5 < -DELTA2_5)){
		reglage_angle_droite(ROT_STAB);

		reglage_distance(DIST_STAB);

		reglage_angle_gauche(ROT_STAB);
	}

	//Adjusment of the robot when it is not centered on its path: closer to the right wall
	else if ((delta2_5 > DELTA2_5)){

		reglage_angle_gauche(ROT_STAB);

		reglage_distance(DIST_STAB);

		reglage_angle_droite(ROT_STAB);
	}
	else {
		stab = OKAY;
	}
	//proximity sensors frequency is about 100 Hz (10ms)
	chThdSleepMilliseconds(10);
}


void finish()
{
	reglage_distance(DIST_END_5CM);
	while(1){
		guidage(DROITE);
		set_body_led(OKAY);
	}
}
void test_stab()
{
	tab_prox[AVANT_DROITE] = get_prox(AVANT_DROITE);
	tab_prox[DIAG_DROITE] = get_prox(DIAG_DROITE);
	tab_prox[LAT_DROITE] = get_prox(LAT_DROITE);
	tab_prox[LAT_GAUCHE] = get_prox(LAT_GAUCHE);
	tab_prox[DIAG_GAUCHE] = get_prox(DIAG_GAUCHE);
	tab_prox[AVANT_GAUCHE] = get_prox(AVANT_GAUCHE);
	chprintf((BaseSequentialStream *)&SD3, "%d %d %d %d %d %d\r\n", tab_prox[DIAG_DROITE], tab_prox[DIAG_GAUCHE], tab_prox[LAT_DROITE], tab_prox[LAT_GAUCHE], tab_prox[AVANT_GAUCHE], tab_prox[AVANT_DROITE]);
}



