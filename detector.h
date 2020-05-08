#ifndef DETECTOR_H_
#define DETECTOR_H_

#define AVANCE				100  //le robot avance
#define GAUCHE 				200  //le robot tourne à gauche
#define DROITE 				300  //le robot tourne à droite
#define STOP 				400  //le robot s'arrête
#define MOTOR				600
#define AVANT_DROITE		0    //proximity sensor avant droite
#define DIAG_DROITE			1    //proximity sensor diagonale droite
#define LAT_DROITE			2    //proximity sensor latéral droite
#define LAT_GAUCHE			5    //proximity sensor latéral gauche
#define DIAG_GAUCHE			6    //proximity sensor diagonal gauche
#define AVANT_GAUCHE		7    //proximity sensor avant gauche
#define MUR					150  //distance à laquelle il detecte un mur
#define MUR_CDS				120	 //distance à laquelle il détecte un mur pour le cul de sac(CDS)
#define MUR_OMBRE			100
#define VIDE				80   //distance à laquelle il detecte du vide
#define MUR_STAB_LAT		90	 //distance à laquelle il detecte un mur pour se stabiliser (latéral)
#define MUR_STAB_DIAG		95	 //distance à laquelle il detecte un mur pour se stabiliser (diagonal)
#define DIST_STAB			150  //temps en ms, se déplacer par accoup
#define ROT_WAIT			200  //temps en milliseconde, faire une pause, tourner en accoups
#define DEMI_DEG			2.9  //temps en ms pour tourner de 0.5°, pour se stabiliser
#define CINQ_DEG			29   //temps en ms pour tourner de 5°
#define ROT_STAB			290  //temps en ms pour tourner de 50° pour se stabiliser
#define QUART_TOUR_G		536.5//temps en ms pour tourner d'un quart de tour à gauche
#define QUART_TOUR_D		551	 //temps en ms pour tourner d'un quart de tour à droite
#define DEMI_TOUR			1015 //temps en ms pour tourner d'un demi tour
#define DIST_OUVERTURE_4CM	520  //temps en ms pour parcourir la distance pour être au centre de l'ouverture (4 cm)
#define DIST_OUVERTURE_8CM	1040 //temps en ms pour parcourir la distance pour être à nouveau entre deux murs (8 cm)
#define DIST_MUR_10CM		1300 //temps en ms pour parcourir la distance pour être de nouveau ente deux murs (10 cm)
#define DIST_CDS_14CM		1820 //temps en ms pour passer outre une ouverture àprès un cul de sac(CDS) (14 cm)
#define DIST_END_5CM		650  //temps en ms pour parcourir la disatnce finale (5cm)

#define DELTA1_6			10   //intervalle [-10,10] dans lequel le robot reste droit à ±1.5°
#define DELTA1_6_GRAND  	35	 //intervalle [-35,35] dans lequel le robot reste droit à ±3.5°
#define DELTA2_5			75   //intervalle [-75,75] dans lequel le robot reste au centre à ±0.5cm
#define OKAY				1  	 //si le robot est stabilisé
#define NOT_OKAY			0	 // si le robot n'est pas stabilisé

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
