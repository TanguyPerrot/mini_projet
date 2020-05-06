/*
 * detector.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Tanguy Perrot
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

//en fonction de la variable "a", le robot va soit avancer(AVANCE), s'arr�ter(STOP), tourner � droite(DROITE), tourner � gauche(GAUCHE)
void guidage(int a);


void obstacle(void);

//le robot va tourner � gauche par accoups de n fois 5�
void reglage_angle_gauche(int n);

//le robot va tourner � droite par accoups de n fois 5�
void reglage_angle_droite(int n);

//le robot va avancer tout droit de n fois (2 � 3) centim�tres
void reglage_distance(int n);

//le robot va se stabiliser au centre du parcours et se mettre face � la route
void stabilisateur(void);

void finish(void);

void test_stab(void);




#endif /* DETECTOR_H_ */
