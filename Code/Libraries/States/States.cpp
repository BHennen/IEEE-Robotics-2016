#include <States.h>

//Constructors /////////////////////
//Constructor with initial state given a direction and x and y position
RobotState::RobotState(Direction dir, byte x, byte y)
{
	SetDirection(dir);
	SetX(x);
	SetY(y);
}

//Default (assume we're at start location)
RobotState::RobotState() : RobotState(RIGHT, static_cast<byte>(0), static_cast<byte>(0))
{

}

//Copy constructor
RobotState::RobotState(const RobotState& other)
{
	bits_ = other.bits_;
}

//Overload assignment operator
RobotState& RobotState::operator= (const RobotState& rhs)
{
	bits_ = rhs.bits_;
	return *this;
}

//Setters //////////////////

//Set leftmost bits of bitset representing our direction
//Direction:	76543210	00 = Up		01 = Right
//				XX------	10 = Down	10 = Left
void RobotState::SetDirection(Direction dir)
{
	switch(dir)
	{
	case UP: //00
		bits_[7] = 0;
		bits_[6] = 0;
		break;
	case RIGHT: //01
		bits_[7] = 0;
		bits_[6] = 1;
		break;
	case DOWN: //10
		bits_[7] = 1;
		bits_[6] = 0;
		break;
	case LEFT: //11
		bits_[7] = 1;
		bits_[6] = 1;
		break;
	}
}

//Set middle 3 bits of bitset representing our x position
//X:	76543210
//		--XXX---
void RobotState::SetX(byte x)
{
	//Check if third bit of x is set
	if(x & static_cast<byte>(4))
	{
		bits_[5] = 1;
	}
	else
	{
		bits_[5] = 0;
	}

	//Check if second bit of x is set
	if(x & static_cast<byte>(2))
	{
		bits_[4] = 1;
	}
	else
	{
		bits_[4] = 0;
	}

	//Check if first bit of x is set
	if(x & static_cast<byte>(1))
	{
		bits_[3] = 1;
	}
	else
	{
		bits_[3] = 0;
	}
}

//Set last 3 bits of bitset representing our y position
//Y:	76543210
//		-----XXX
void RobotState::SetY(byte y)
{
	//Check if third bit of y is set
	if(y & static_cast<byte>(4))
	{
		bits_[2] = 1;
	}
	else
	{
		bits_[2] = 0;
	}

	//Check if second bit of y is set
	if(y & static_cast<byte>(2))
	{
		bits_[1] = 1;
	}
	else
	{
		bits_[1] = 0;
	}

	//Check if first bit of y is set
	if(y & static_cast<byte>(1))
	{
		bits_[0] = 1;
	}
	else
	{
		bits_[0] = 0;
	}
}

//Getters //////////////////////

//Get leftmost bits of bitset representing our direction
//Direction:	76543210	00 = Up		01 = Right
//				XX------	10 = Down	11 = Left
Direction RobotState::GetDirection()
{
	if(bits_.test(7))
	{
		//Left or Down
		if(bits_.test(6))
		{
			return LEFT; //11
		}
		else
		{
			return DOWN; //10
		}
	}
	else
	{
		//Right or Up
		if(bits_.test(6))
		{
			return RIGHT; //01
		}
		else
		{
			return UP; //00
		}
	}
}

//Get middle 3 bits of bitset representing our x position
//X:	76543210
//		--XXX---
byte RobotState::GetX()
{
	//(1 or 0) * 4 + (1 or 0) * 2 + (1 or 0) * 1
	return bits_.test(5) * static_cast<byte>(4) + bits_.test(4) * static_cast<byte>(2) + bits_.test(3) * static_cast<byte>(1);
}

//Set last 3 bits of bitset representing our y position
//Y:	76543210
//		-----XXX
byte RobotState::GetY()
{
	//(1 or 0) * 4 + (1 or 0) * 2 + (1 or 0) * 1
	return bits_.test(2) * static_cast<byte>(4) + bits_.test(1) * static_cast<byte>(2) + bits_.test(0) * static_cast<byte>(1);
}