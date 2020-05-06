#ifndef DETECTOR_H_
#define DETECTOR_H_

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
