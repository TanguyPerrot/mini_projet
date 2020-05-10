#ifndef DETECTOR_H_
#define DETECTOR_H_

#define OFF 						0
#define ON 							1
#define TEN_MILLISECONDS			10

#define FORWARD						100  //the robot goes forward
#define LEFT 						200  //the robot turns to the left
#define RIGHT 						300  //the robot turns to the right
#define STOP 						400  //the robot stops
#define MOTOR						600	 //value at which the robot starts to move

#define FRONT_RIGHT					0    //proximity sensor on the front right sided
#define DIAG_RIGHT					1    //proximity sensor diagonal on the right
#define LAT_RIGHT					2    //proximity sensor lateral on the right
#define LAT_LEFT					5    //proximity sensor lateral on the left
#define DIAG_LEFT					6    //proximity sensor diagonal on the left
#define FRONT_LEFT					7    //proximity sensor on the front left sided

#define WALL						150  //distance at which the robot detects a wall
#define WALL_CDS					120	 //distance at which the robot detects a wall in the Cul-de-sac(CDS)
#define WALL_OMBRE					100  //distance at which the robot detects a wall on the right/left : open on the left/right without an obstacle in the front
#define VIDE						80   //distance at which the robot detects nothing: there is only void around it
#define WALL_STAB_LAT				90	 //distance at which the robot detects a wall to stabilize (lateral)
#define WALL_STAB_DIAG				95	 //distance at which the robot detects a wall to stabilize (diagonal)

#define DIST_STAB					150  //time in ms, displacement w/ à-coups
#define ROT_WAIT					200  //time in ms, stop for a pause, turn w/ à-coups
#define HALF_DEG					2.9  //time in ms to turn about 0.5°, in order to stabilize
#define FIVE_DEG					29   //time in ms to turn about 5°
#define ROT_STAB					290  //time in ms to turn about 50° in order to stabilize
#define QUART_TURN_LEFT				536.5//time in ms to turn a quater turn to the left
#define QUART_TURN_RIGHT			551	 //time in ms to turn a quater turn to the right
#define HALF_TURN					1015 //time in ms to turn a half turn

#define CENTER_BEFORE_OPNG			520  //time in ms to travel a distance (estimated 4cm) to be centered at the opening
#define DIST_2B_BTW_2WALLS_OPNG		1040 //time in ms to travel a distance (estimated 8cm) to be again well placed between two walls while detecting an opening
#define DIST_2B_BTW_2WALLS_OBST		1300 //time in ms to travel a distance (estimated 10cm) to be again well placed between two walls while detecting an obstacle
#define DIST_PASS_OPNG				1820 //time in ms to pass the entrance opening after going in the Cul-de-sac, it travels 14cm
#define DIST_AT_END		   			650  //time in ms to travel a distance of around 5cm when reaching the end of the maze

#define DELTA_DIAG					10   //band of [-10,10] in which the robot stays directed straight forward w/ ±1.5°
#define DELTA_DIAG_GRAND  			35	 //band of [-35,35] in which the robot stays directed straight forward w/ ±3.5°
#define DELTA_LAT					75   //band of [-75,75] in which the robot stays in the center w/ ±0.5cm
#define OKAY						1  	 //Robot is stabilized on its path
#define NOT_OKAY					0	 //Robot is not stabilized on its path



//Depending on the variable a, the robot will go forward, stop, turn to the right or the left
void move_guidance(int a);

//Different scenarios of when the robot meets an obstacle and how it moves then
void obstacle(void);

//The robot turns to the left with an angle of n times 5 degrees
void reglage_left_angle(int n);

//The robot turns to the right with an angle of n times 5 degrees
void reglage_right_angle(int n);

//The robot will move forward with a distance of n times 2 (up to 3) cm
void reglage_distance(int n);

//The robot will stabilize at the center of its path and will be positioned to move straight forward
void stabilize_robot(void);

//Manages how to know when the robot reaches the end of the maze
void end(void);

#endif /* DETECTOR_H_ */
