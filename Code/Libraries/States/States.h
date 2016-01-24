#ifndef States_H
#define States_H

#include <iterator>
#include <bitset>
#include <Directions.h>

class RobotState
{
public:
	//Constructors ////////////////////
	//Constructor given all paramaters
	RobotState(Direction dir, byte x, byte y);

	//Default (assume we're at start location)
	RobotState();

	//Copy constructor
	RobotState(const RobotState& other);

	//Overload assignment operator
	RobotState& operator= (const RobotState& rhs);

	//Setters //////////////////
	void SetDirection(Direction dir);

	void SetX(byte x);

	void SetY(byte y);

	//Getters //////////////////////
	Direction GetDirection();

	byte GetX();

	byte GetY();

private:
	static const size_t STATE_SIZE = 8;
	std::bitset<STATE_SIZE> bits_;
};

#endif