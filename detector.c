#include "ch.h"
#include "hal.h"
#include <detector.h>
#include <main.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

//The detected values by the proximity sensors can go up to 4'000 so 16 bits were requiered
//we only use 6 proximty sensors out of 8
static uint16_t tab_prox[8] = {0,0,0,0,0,0,0,0};
static int delta_diag = 0;	//the difference between the values received by the diagonal proximity sensors on both sides
static int delta_lat = 0; 	//the difference between the values received by the lateral proximity sensors on both sides
static int counter = 0;
static bool stabilizer = 0;

void move_guidance(int a)
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
	if(counter == 0){
		set_led(LED1, OFF);
	}
	else{
		set_led(LED1, ON);
	}

	//We start by collecting the values detected by the proximity sensors
	tab_prox[FRONT_RIGHT] = get_prox(FRONT_RIGHT);
	tab_prox[DIAG_RIGHT] = get_prox(DIAG_RIGHT);
	tab_prox[LAT_RIGHT] = get_prox(LAT_RIGHT);
	tab_prox[LAT_LEFT]	= get_prox(LAT_LEFT);
	tab_prox[DIAG_LEFT] = get_prox(DIAG_LEFT);
	tab_prox[FRONT_LEFT] = get_prox(FRONT_LEFT);

	move_guidance(FORWARD);

	/*	Testing the different scenarios of the displacement of the robot inside the maze  */

	//Open on the left and an obstacle in the front
	if ((tab_prox[FRONT_RIGHT] > WALL) & (tab_prox[FRONT_LEFT] > WALL) & (tab_prox[LAT_LEFT] < VIDE)){
		reglage_left_angle(QUART_TURN_LEFT);

		reglage_distance(DIST_2B_BTW_2WALLS_OBST);
		tab_prox[LAT_LEFT] = get_prox(LAT_LEFT);

		if((tab_prox[LAT_LEFT] > WALL_STAB_LAT) & (tab_prox[DIAG_LEFT] > WALL_STAB_DIAG)){
			stabilizer= NOT_OKAY;

			while(stabilizer!= OKAY){
				stabilize_robot();
			}
		}
	}

	//Open on the right and an obstacle in the front
	else if((tab_prox[FRONT_RIGHT] > WALL) & (tab_prox[FRONT_LEFT] > WALL) & (tab_prox[LAT_RIGHT] < VIDE)){
		reglage_right_angle(QUART_TURN_RIGHT);

		reglage_distance(DIST_2B_BTW_2WALLS_OBST);
		tab_prox[LAT_RIGHT] = get_prox(LAT_RIGHT);

		if((tab_prox[LAT_RIGHT] > WALL_STAB_LAT) & (tab_prox[DIAG_RIGHT] > WALL_STAB_DIAG)){
			stabilizer= NOT_OKAY;

			while(stabilizer!= OKAY){
				stabilize_robot();
			}
			if(counter > 0){
				reglage_right_angle(FIVE_DEG);
			}
		}
	}

	//Open on the left without an obstacle in the front
	else if((tab_prox[LAT_LEFT] < VIDE) & (tab_prox[FRONT_LEFT] < VIDE) & (tab_prox[LAT_RIGHT] > (WALL_OMBRE))){
		reglage_distance(CENTER_BEFORE_OPNG);

		if(tab_prox[FRONT_LEFT] < VIDE){

			if(counter == 0){
				reglage_left_angle(QUART_TURN_LEFT);
				reglage_distance(DIST_2B_BTW_2WALLS_OPNG);
				stabilizer= NOT_OKAY;

				while(stabilizer!= OKAY){
					stabilize_robot();
				}
			}
			else{
				counter--;
				reglage_distance(DIST_PASS_OPNG);
				stabilizer= NOT_OKAY;

				while(stabilizer!= OKAY){
					stabilize_robot();
				}
			}
		}
	}

	//Open on the right without an obstacle in the front
	else if((tab_prox[LAT_RIGHT] < VIDE) & (tab_prox[FRONT_RIGHT] < VIDE) & (tab_prox[LAT_LEFT] > (WALL_OMBRE))){
		reglage_distance(CENTER_BEFORE_OPNG);

		if(tab_prox[FRONT_RIGHT] < VIDE){

			if(counter == 0){
				reglage_right_angle(QUART_TURN_LEFT);
				reglage_distance(DIST_2B_BTW_2WALLS_OPNG);
				stabilizer= NOT_OKAY;

				while(stabilizer!= OKAY){
					stabilize_robot();
				}
			}
			else{
				counter--;
				reglage_distance(DIST_PASS_OPNG);
				stabilizer= NOT_OKAY;

				while(stabilizer!= OKAY){
					stabilize_robot();
				}
			}
		}
	}

	//Cul-de-sac
	else if((tab_prox[FRONT_RIGHT] > WALL_CDS) & (tab_prox[FRONT_LEFT] > WALL_CDS) & (tab_prox[LAT_LEFT] > WALL_CDS) & (tab_prox[LAT_RIGHT] > WALL_CDS)){
		reglage_left_angle(HALF_TURN);
		counter++;
		stabilizer = NOT_OKAY;

		while(stabilizer!= OKAY){
			stabilize_robot();
		}
	}

	//Exit of the maze: proximity sensors detect nothing
	else if((tab_prox[FRONT_RIGHT] < VIDE) & (tab_prox[FRONT_LEFT] < VIDE) & (tab_prox[DIAG_RIGHT] < VIDE) & (tab_prox[DIAG_LEFT] < VIDE)){
		reglage_distance(CENTER_BEFORE_OPNG);

		if((tab_prox[LAT_LEFT] > WALL_STAB_LAT) | (tab_prox[LAT_RIGHT] > WALL_STAB_LAT)){
			end();
		}
	}
	else{
		move_guidance(FORWARD);
	}

	//proximity sensors frequency is about 100 Hz (10ms)
	chThdSleepMilliseconds(TEN_MILLISECONDS);
}

//Rotation of the robot to the left
void reglage_left_angle(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		move_guidance(LEFT);
	}
	start = chVTGetSystemTime();
	end = start + MS2ST(ROT_WAIT);

	while (chVTIsSystemTimeWithin(start, end)){
		move_guidance(STOP);
	}
}

//Rotation of the robot to the right
void reglage_right_angle(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		move_guidance(RIGHT);
	}

	start = chVTGetSystemTime();
	end = start + MS2ST(ROT_WAIT);

	while (chVTIsSystemTimeWithin(start, end)){
		move_guidance(STOP);
	}
}

//Robot moves forward of n times 2-3 cm
void reglage_distance(int n)
{
	systime_t start = chVTGetSystemTime();
	systime_t end = start + MS2ST(n);

	while (chVTIsSystemTimeWithin(start, end)){
		move_guidance(FORWARD);
	}
}

//The robot will stabilize at the center of its path and will be positioned to move straight forward
void stabilize_robot()
{
	tab_prox[DIAG_RIGHT] = get_prox(DIAG_RIGHT);
	tab_prox[LAT_RIGHT] = get_prox(LAT_RIGHT);
	tab_prox[LAT_LEFT] = get_prox(LAT_LEFT);
	tab_prox[DIAG_LEFT] = get_prox(DIAG_LEFT);

	delta_diag = tab_prox[DIAG_RIGHT] - tab_prox[DIAG_LEFT];
	delta_lat = tab_prox[LAT_RIGHT] - tab_prox[LAT_LEFT];


	//Adjusment of the angle of the robot when it is turned more to the right
	if(delta_diag > DELTA_DIAG){
		while(delta_diag > DELTA_DIAG){
			if(delta_diag > DELTA_DIAG_GRAND){
				move_guidance(LEFT);
			}
			else{
				reglage_left_angle(HALF_DEG);
			}
			tab_prox[DIAG_RIGHT] = get_prox(DIAG_RIGHT);
			tab_prox[DIAG_LEFT] = get_prox(DIAG_LEFT);
			delta_diag = tab_prox[DIAG_RIGHT] - tab_prox[DIAG_LEFT];
		}
	}

	//Adjusment of the angle of the robot when it is turned more to the left
	else if (delta_diag < -DELTA_DIAG){

		while(delta_diag < -DELTA_DIAG){

			if(delta_diag < -DELTA_DIAG_GRAND){
				move_guidance(RIGHT);
			}
			else{
				reglage_right_angle(HALF_DEG);
			}
			tab_prox[DIAG_RIGHT] = get_prox(DIAG_RIGHT);
			tab_prox[DIAG_LEFT] = get_prox(DIAG_LEFT);
			delta_diag = tab_prox[DIAG_RIGHT]-tab_prox[DIAG_LEFT];
		}
	}

	//Adjusment of the robot when it is not centered on its path: closer to the left wall
	else if ((delta_lat < -DELTA_LAT)){
		reglage_right_angle(ROT_STAB);

		reglage_distance(DIST_STAB);

		reglage_left_angle(ROT_STAB);
	}

	//Adjusment of the robot when it is not centered on its path: closer to the right wall
	else if ((delta_lat > DELTA_LAT)){

		reglage_left_angle(ROT_STAB);

		reglage_distance(DIST_STAB);

		reglage_right_angle(ROT_STAB);
	}
	else {
		stabilizer= OKAY;
	}
	//proximity sensors frequency is about 100 Hz (10ms)
	chThdSleepMilliseconds(TEN_MILLISECONDS);
}

//This function is activated once the robot reaches the end of the maze
void end()
{
	reglage_distance(DIST_AT_END);

	while(1){
		move_guidance(RIGHT);
		set_body_led(ON);
	}
}
