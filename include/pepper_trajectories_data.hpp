#ifndef PEPPER_TRAJECTORIES_DATA_HPP
#define PEPPER_TRAJECTORIES_DATA_HPP


// 0:  Generate random poses in a squared area									18- H33 R33 L33     OK
#define RANDOM_POSES 0

// 1:  Generate a simple circular trajectory in the YZ plane for the arms and the head				6 - H00 R30 L30     OK ----
#define HEADARM_POSES_CIRCLES 1

// 2:  Wave with the right hand while keeping head looking forward (all orientation constrained)		7 - H02 R23 L00     OK ----
#define RIGHTARM_WAVING 2
// 3:  Wave with the right hand while fixing only head orientation velocity wz to zero				6 - H01 R23 L00     OK
#define RIGHTARM_WAVING_RELAXED 3
// 4:  3 but pepper looks to point in front of it								6 - H01 R23 L00     OK
#define RIGHTARM_WAVING_RELAXED_LOOK 4

// 5:  Pepper carries an object in the right hand while facing and moving forward				4 - H01 R30 L00     OK --
#define RIGHTARM_TRANSPORT_OBJECT_POS 5
// 6:  Like 5 but without constrining Z coordinate of right hand						3 - H01 R20 L00     OK
#define RIGHTARM_TRANSPORT_OBJECT_POS_XY 6

// 7:  Transport dish with hand faced up (like a waiter)							6 - H01 R00 L23     OK --
#define LEFTARM_TRANSPORT_OBJECT_POS_XY_OR 7
// 8:  Transport dish with hand faced up (like a waiter) fixing only w_x and w_y orientation			5 - H01 R00 L22     OK
#define LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY 8

// 9:  Shake right hand, constraint all pos and or of right hand, and head orientation				9 - H03 R33 L00        --
#define RIGHTHAND_SHAKE 9
// 10:  Like 9 but only Z pos taken into account								5 - H03 R13 L00
#define RIGHTHAND_SHAKE_POS_Z 10
// 11:  Like 9 but also constraining head y									10- H13 R33 L00
#define RIGHTHAND_SHAKE_HEAD_Y 11
// 12:  Like 9 but also constraining head Y and unconstraining Right arm pos X					9 - H13 R23 L00
#define RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ 12
// 13:  Like 9 but unconstraining Right arm pos X								8 - H03 R23 L00
#define RIGHTHAND_SHAKE_RIGHTARM_YZ 13
// 14:  head (Y, w_z), rightarm (y, z, w_x, w_y, w_z)								7 - H11 R23 L00
#define RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR 14
// 15:  head (Y, w_z), rightarm (y, z, w_x, w_y, w_z)								8 - H12 R23 L00
#define RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR 15
// 24:  head (w_x), rightarm (y, z, w_x, w_y, w_z)								6 - H01 R23 L00     OK
#define RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ 24

// 16:  head (Or), rightarm (Pos, Or), leftarm (Pos, Or)							15- H03 R33 L33
#define OBJECT_BOTH_HANDS 16
// 17:  rightarm (Pos, Or), leftarm (Pos, Or)									12- H00 R33 L33     OK
#define OBJECT_BOTH_HANDS_NO_HEAD 17
// 18:  rightarm (Pos), leftarm (Pos), with grasping coordination						12- H00 R33 L33     OK
#define OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD 18

// 19:  head (x, y, z)												3 - H30 R00 L00        --
#define HEAD_MOVE 19
// 20:  head (x, y, z, w_x', w_y', w_z')									6 - H33 R00 L00
#define HEAD_MOVE_STARE 20
// 21:  head (x, y, z, w_y', w_z')										5 - H32 R00 L00
#define HEAD_MOVE_STARE_FREEWX 21
// 22:  head (x, y, z, w_y', w_z')										4 - H22 R00 L00
#define HEAD_CRAZY_ARCH_FREEWX 22
// 23:  head (x, y, z, w_x', w_y', w_z')									5 - H23 R00 L00
#define HEAD_CRAZY_ARCH 23

// 24 is above

// 25:  head (w_x, w_y, w_z)											3 - H03 R00 L00
#define HEAD_GAZE 25

#endif


// --------------------------------------------------------------------------------------
// Combinations coded
// 
//   3 - H30 R00 L00
//   3 - H03 R00 L00
//   3 - H01 R20 L00
// 
//   4 - H22 R00 L00
//   4 - H01 R30 L00
// 
//   5 - H32 R00 L00
//   5 - H23 R00 L00
//   5 - H03 R13 L00
//   5 - H01 R00 L22
// 
//   6 - H33 R00 L00
//   6 - H01 R23 L00
//   6 - H01 R00 L23
//   6 - H00 R30 L30
// 
//   7 - H11 R23 L00
//   7 - H02 R23 L00
//
//   9 - H12 R23 L00
//   8 - H03 R23 L00
//
//   9 - H13 R23 L00
//   9 - H03 R33 L00
// 
//   10- H13 R33 L00
// 
//   12- H00 R33 L33
//   12- H00 R33 L33 - Grasp Coordination
//
//   15- H03 R33 L33
//
//   18- H33 R33 L33