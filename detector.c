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

	int var0 = 0;
	int var1 = 0;
	int var2 = 0;
	int var5 = 0;
	int var6 = 0;
	int var7 = 0;

	void obstacle()
	{
		var0 = get_prox(0);
		var1 = get_prox(1);
		var2 = get_prox(2);
		var5 = get_prox(5);
		var6 = get_prox(6);
		var7 = get_prox(7);

		left_motor_set_speed(600);
		right_motor_set_speed(600);

		//si ouverture à gauche et mur devant
		if ((var0 > 150) & (var7 > 150) & (var5 < 80)){
			while(var1 > 150){
				left_motor_set_speed(-600);
				right_motor_set_speed(600);
				var1 = get_prox(1);
			}
			/*while(var5 < 80){
				left_motor_set_speed(600);
				right_motor_set_speed(600);
				var5 = get_prox(5);
			}*/

		}
		//si ouverture à droite et mur devant
		else if((var0 > 150) & (var7 > 150) & (var2 < 80)){
			while(var6 > 150){
				left_motor_set_speed(600);
				right_motor_set_speed(-600);
				var6 = get_prox(6);
			}
			/*while(var2 < 80){
				left_motor_set_speed(600);
				right_motor_set_speed(600);
				var2 = get_prox(2);
			}*/
		}
		/*//si ouverture à gauche et à droite et mur devant
		else if((var0 > 150) & (var7 > 150) & (var5 < 80) & (var2 < 80)){
			while(var1 > 150){
				left_motor_set_speed(-600);
				right_motor_set_speed(600);
				var1 = get_prox(1);
			}
			while(var5 < 80){
				left_motor_set_speed(600);
				right_motor_set_speed(600);
				var5 = get_prox(5);
			}
		}*/
		// cul de sac
		else if((var0 > 150) & (var7 > 150) & (var5 > 150) & (var2 > 150)){
				while((var0 > 150) & (var7 > 150) & (var1 > 150)){
					left_motor_set_speed(-600);
					right_motor_set_speed(600);
					var0 = get_prox(0);
					var7 = get_prox(7);
					var1 = get_prox(1);
			}
		}
/*		//ouverture à gauche
		else if((var5 < 80) & (var0 < 80) & (var7 < 80)){
				while(var1 > 150){
					left_motor_set_speed(-600);
					right_motor_set_speed(600);
					var1 = get_prox(1);
			}
		}
		//ouverture à droite
		else if((var2 < 80) & (var0 < 80) & (var7 < 80)){
				while(var6 > 150){
					left_motor_set_speed(600);
					right_motor_set_speed(-600);
					var6 = get_prox(6);
			}
		}*/
		else{
			left_motor_set_speed(600);
			right_motor_set_speed(600);
		}
	}


	void ouverture()
	{
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}

	void avancer ()
	{
		var0 = get_prox(0);
		var1 = get_prox(1);
		var2 = get_prox(2);
		var5 = get_prox(5);
		var6 = get_prox(6);
		var7 = get_prox(7);

		if((var0 > 150) & (var7 > 150)){
			//left_motor_set_speed(0);
			//right_motor_set_speed(0);
			obstacle();
		}
		/*else if((var2 < 80) & (var0 < 80) & (var7 < 80)){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			ouverture();
		}*/
	}

	void reglage_angle_gauche(int n)
	{

		systime_t start = chVTGetSystemTime();
		systime_t end = start + MS2ST(n);

		while (chVTIsSystemTimeWithin(start, end)){
			left_motor_set_speed(-600);
			right_motor_set_speed(600);

		}
		start = chVTGetSystemTime();
		end = start + MS2ST(200);

		while (chVTIsSystemTimeWithin(start, end)){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			}
		}
	void reglage_angle_droite(int n)
		{

			systime_t start = chVTGetSystemTime();
			systime_t end = start + MS2ST(n);

			while (chVTIsSystemTimeWithin(start, end)){
				left_motor_set_speed(600);
				right_motor_set_speed(-600);

			}
			start = chVTGetSystemTime();
			end = start + MS2ST(200);

			while (chVTIsSystemTimeWithin(start, end)){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				}
			}
	void reglage_distance()
			{

				systime_t start = chVTGetSystemTime();
				systime_t end = start + MS2ST(200);

				while (chVTIsSystemTimeWithin(start, end)){
					left_motor_set_speed(600);
					right_motor_set_speed(600);

				}
				start = chVTGetSystemTime();
				end = start + MS2ST(200);

				while (chVTIsSystemTimeWithin(start, end)){
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					}
				}

	void stabilisateur()
	{
		int delta1_6;
		int	delta2_5;

		var0 = get_prox(0);
		var1 = get_prox(1);
		var2 = get_prox(2);
		var5 = get_prox(5);
		var6 = get_prox(6);
		var7 = get_prox(7);

		delta1_6 = var1-var6;
		delta2_5 = var2-var5;

		if(delta1_6 > 20){
			while(delta1_6 > 20){
				reglage_angle_gauche(29);
				var1 = get_prox(1);
				var6 = get_prox(6);
				delta1_6 = var1-var6;
			}
		}
		else if (delta1_6 < -20){
			while(delta1_6 < -20){
				reglage_angle_droite(29);
				var1 = get_prox(1);
				var6 = get_prox(6);
				delta1_6 = var1-var6;

			}
		}
		else if (delta2_5 < -75){
			reglage_angle_droite(290);

			reglage_distance();

			reglage_angle_gauche(290);
		}
		else if (delta2_5 > 75){

				reglage_angle_gauche(290);

			reglage_distance();

				reglage_angle_droite(290);
		}
		else{
			left_motor_set_speed(600);
			right_motor_set_speed(600);
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



