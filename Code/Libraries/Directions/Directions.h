#ifndef Directions_H
#define Directions_H

#include <Arduino.h> //for byte

//Directions to turn, wall follow, etc
enum Direction : byte
{
	UP = 0,
	RIGHT = 1,
	DOWN = 2,
	LEFT = 3
};

//Given an old robot direction, finds what the new direction would be after a rotation
Direction MapRotationToNewDirection(Direction rotation, Direction old_direction);
inline Direction MapRotationToNewDirection(Direction rotation, Direction old_direction)
{
	if(rotation == RIGHT)
	{
		return static_cast<Direction>((old_direction + 1) % 4);
	}
	else
	{
		if(old_direction == UP)
		{
			return LEFT;
		}
		else
		{
			return static_cast<Direction>((old_direction - 1));
		}
	}
}
#endif