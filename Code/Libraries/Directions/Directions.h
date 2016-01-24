#ifndef Directions_H
#define Directions_H

#include <Arduino.h> //for byte

//Directions to turn, wall follow, etc
enum Direction : byte
{
	UP,
	RIGHT,
	DOWN,
	LEFT
};

#endif