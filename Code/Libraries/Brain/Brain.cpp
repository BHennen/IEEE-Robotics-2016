#include "Brain.h"

/**
* Constructor.
*/
Brain::Brain()
{

}

//Destructor
Brain::~Brain()
{

}

/**
* Use sensors to follow a wall to the [direction] of the robot indefinitely. Return true when any of the stop conditions set
* by flags are met.
* stop conditions :
*		gap -- Check if gap was found in the direction d. Return true when both sensors have detected the gap.
*		pixy -- Check if pixy has detected ~10? good blocks. Return true when it has detected enough blocks.
*		front -- Check front IR sensor return true when it it close to wall in front.
*
* To call this: FollowWall(left, gap | pixy); <-- This follow left wall and stop when a gap is detected or pixy sees a victim
*/
bool Brain::FollowWall(Direction d, StopConditions flags)
{
	//check for gap 
	if(flags & GAP)
	{
		
	}

	//check pixy 
	if(flags & PIXY)
	{
		
	}

	//check front
	if(flags & FRONT)
	{
		
	}

	//Follow Wall
	return false;
}

//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
bool Brain::GoAtoB(Position A, Position B)
{
	return false;
}

//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
bool Brain::GoToVictim()
{
	return false;
}