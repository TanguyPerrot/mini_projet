/*
 * detector.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Tanguy Perrot
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

#define AVANCE			100  //le robot avance
#define GAUCHE 			200  //le robot tourne à gauche
#define DROITE 			300  //le robot tourne à droite
#define STOP 			400  //le robot s'arrête
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
#define DELTA1_6_GRAND  35	 //intervalle [-30,30] dans lequel le robot reste droit à ±3.5°
#define DELTA2_5		75   //intervalle [-75,75] dans lequel le robot reste au centre à ±0.5cm
#define OKAY			1  	 //si le robot est stabilisé
#define NOT_OKAY		0	 // si le robot n'est pas stabilisé

//Depending on the variable a, the robot will go forward, stop, turn to the right or the left
void guidage(int a);

//Diffrent senarios of when the robot meets an obstacle and how it moves then
void obstacle(void);

//The robot turns to the left with an angle of n times 5 degrees
void reglage_angle_gauche(int n);

//The robot turns to the right with an angle of n times 5 degrees
void reglage_angle_droite(int n);

//The robot will move forward with a distance of n times 2 (up to 3) cm  n fois (2 à 3) centimètres
void reglage_distance(int n);

//The robot will stabilize at the center of its path and will be positionned to move straight forward
void stabilisateur(void);

//Manges how to know when the robot reaches the end of the maze
void finish(void);

void test_stab(void);




#endif /* DETECTOR_H_ */
