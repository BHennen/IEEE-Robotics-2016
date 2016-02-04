#include <States.h>

//Constructors /////////////////////
//Constructor with initial state given a direction and x and y position
RobotState::RobotState(Direction dir, byte x, byte y, bool on_victim/* = false*/)
{
	SetDirection(dir);
	SetX(x);
	SetY(y);
	SetOnVictim(on_victim);
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

//Set leftmost bits of Bitset representing our direction
//Direction:	876543210	00 = Up		01 = Right
//				-XX------	10 = Down	10 = Left
void RobotState::SetDirection(Direction dir)
{
	switch(dir)
	{
	case UP: //00
		bits_.Set(7, 0);
		bits_.Set(6, 0);
		break;
	case RIGHT: //01
		bits_.Set(7, 0);
		bits_.Set(6, 1);
		break;
	case DOWN: //10
		bits_.Set(7, 1);
		bits_.Set(6, 0);
		break;
	case LEFT: //11
		bits_.Set(7, 1);
		bits_.Set(6, 1);
		break;
	}
}

//Set middle 3 bits of Bitset representing our x position
//X:	876543210
//		---XXX---
void RobotState::SetX(byte x)
{
	//Check if third bit of x is set
	if(x & static_cast<byte>(4))
	{
		bits_.Set(5, 1);
	}
	else
	{
		bits_.Set(5, 0);
	}

	//Check if second bit of x is set
	if(x & static_cast<byte>(2))
	{
		bits_.Set(4, 1);
	}
	else
	{
		bits_.Set(4, 0);
	}

	//Check if first bit of x is set
	if(x & static_cast<byte>(1))
	{
		bits_.Set(3, 1);
	}
	else
	{
		bits_.Set(3, 0);
	}
}

//Set last 3 bits of Bitset representing our y position
//Y:	876543210
//		------XXX
void RobotState::SetY(byte y)
{
	//Check if third bit of y is set
	if(y & static_cast<byte>(4))
	{
		bits_.Set(2, 1);
	}
	else
	{
		bits_.Set(2, 0);
	}

	//Check if second bit of y is set
	if(y & static_cast<byte>(2))
	{
		bits_.Set(1, 1);
	}
	else
	{
		bits_.Set(1, 0);
	}

	//Check if first bit of y is set
	if(y & static_cast<byte>(1))
	{
		bits_.Set(0, 1);
	}
	else
	{
		bits_.Set(0, 0);
	}
}

//Set whenever we're on top of a victim
//on:	876543210
//		X--------
void RobotState::SetOnVictim(bool on_victim) 
{
	bits_.Set(8, on_victim);
}

//Getters //////////////////////

//Get leftmost bits of Bitset representing our direction
//Direction:	876543210	00 = Up		01 = Right
//				-XX------	10 = Down	11 = Left
Direction RobotState::GetDirection() const
{
	if(bits_.Test(7))
	{
		//Left or Down
		if(bits_.Test(6))
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
		if(bits_.Test(6))
		{
			return RIGHT; //01
		}
		else
		{
			return UP; //00
		}
	}
}

//Get middle 3 bits of Bitset representing our x position
//X:	876543210
//		---XXX---
byte RobotState::GetX() const
{
	//(1 or 0) * 4 + (1 or 0) * 2 + (1 or 0) * 1
	return bits_.Test(5) * static_cast<byte>(4) + bits_.Test(4) * static_cast<byte>(2) + bits_.Test(3) * static_cast<byte>(1);
}

//Set last 3 bits of Bitset representing our y position
//Y:	876543210
//		------XXX
byte RobotState::GetY() const
{
	//(1 or 0) * 4 + (1 or 0) * 2 + (1 or 0) * 1
	return bits_.Test(2) * static_cast<byte>(4) + bits_.Test(1) * static_cast<byte>(2) + bits_.Test(0) * static_cast<byte>(1);
}

//Return true when we're on top of a victim.
//on:	876543210
//		X--------
bool RobotState::IsOnVictim() const
{
	return bits_.Test(8);
}

//BoardState//////////////////////

//Constructor
BoardState::BoardState()
{
	//Create 8x8 grid of empty Bitsets
	for(int row = 0; row < 8; row++)
	{
		for(int col = 0; col < 8; col++)
		{
			arena_[row][col] = Bitset<byte>();
		}
	}
}

//Constructor for initial states
BoardState::BoardState(byte init_states[8][8])
{
	//fill in arena left to right, top to bottom from the list
	//It should be like a standard coordinate system, with start at [0][0]
	for(int row = 0; row < 8; row++)
	{
		for(int col = 0; col < 8; col++)
		{
			arena_[7 - row][col].Set(init_states[row][col]);
		}
	}
};

//TODO: In test code, x and y for these functions had to be swapped. Make sure this isn't the case.
//Test if a grid unit has a wall in specified direction
//		76543210
//		URDL----
bool BoardState::HasWall(byte x, byte y, Direction dir) const
{
	byte val;
	if(dir == UP)
	{
		val = 7;
	}
	else if(dir = RIGHT)
	{
		val = 6;
	}
	else if(dir = DOWN)
	{
		val = 5;
	}
	else if(dir = LEFT)
	{
		val = 4;
	}
	return arena_[x][y].Test(val);
}

//Check if grid unit is yellow location drop zone
//		76543210
//		----Y---
bool BoardState::IsYellow(byte x, byte y) const
{
	return arena_[x][y].Test(3);
}

//Check if grid unit is red location drop zone
//		76543210
//		-----R--
bool BoardState::IsRed(byte x, byte y) const
{
	return arena_[x][y].Test(2);
}

//Check if grid unit is a victim location
//		76543210
//		------V-
bool BoardState::HasVictim(byte x, byte y) const
{
	return arena_[x][y].Test(1);
}

//Check if grid unit is a passable location (meaning the robot can move through it)
//		76543210
//		-------P
bool BoardState::IsPassable(byte x, byte y) const
{
	return arena_[x][y].Test(0);
}

//Set the state of where the right victim is.
void BoardState::SetRightVictimLocation(Direction dir)
{
	if(dir == UP)
	{
		//Victim in the right upper position is there and no longer passable.
		arena_[5][7].Set(1, 1);
		arena_[5][7].Set(0, 0);
		//Remove lower victim and make passable
		arena_[7][5].Set(1, 0);
		arena_[7][5].Set(0, 1);
	}
	else if(dir == DOWN)
	{
		//Victim in the right lower position is there and no longer passable.
		arena_[7][5].Set(1, 1);
		arena_[7][5].Set(0, 0);
		//Remove upper victim and make passable
		arena_[5][7].Set(1, 0);
		arena_[5][7].Set(0, 1);
	}
}

//Set the state of where the left victim is.
void BoardState::SetLeftVictimLocation(Direction dir)
{
	if(dir == UP)
	{
		//Victim in the left upper position is there and no longer passable.
		arena_[0][5].Set(1, 1);
		arena_[0][5].Set(0, 0);
		//Remove lower victim and make passable
		arena_[0][3].Set(1, 0);
		arena_[0][3].Set(0, 1);
	}
	else if(dir == DOWN)
	{
		//Victim in the left lower position is there and no longer passable.
		arena_[0][3].Set(1, 1);
		arena_[0][3].Set(0, 0);
		//Remove upper victim and make passable
		arena_[0][5].Set(1, 0);
		arena_[0][5].Set(0, 1);
	}
}

//Set flag for grid position at [x][y] for the victim to false and make passable.
void BoardState::RemoveVictim(byte x, byte y)
{
	arena_[x][y].Set(1, 0);
	arena_[x][y].Set(0, 1);
}

RobotStateSet::RobotStateSet()
{

}

//Add copy of state to the set (if possible, returns true, otherwise returns false)
//Return true if successfully added
//Return false if set already contains this state
bool RobotStateSet::Add(const RobotState& state)
{
	if(this->Contains(state)) return false;

	set_.push_back(state);
	return true;
}

//Return true if the set already has this state.
bool RobotStateSet::Contains(const RobotState& state)
{
	for(std::vector<RobotState>::const_iterator it = set_.begin(); it != set_.end(); ++it)
	{
		if(*it == state) return true;
	}
	return false;
}