#ifndef States_H
#define States_H

#include <iterator>
#include <bitset>
#include <vector>
#include <Directions.h>

static const size_t ROBOT_STATE_SIZE = 9;
static const size_t BOARD_STATE_SIZE = 8;

//Class representing the state of a robot (direction and position)
class RobotState
{
public:
	//Constructors ////////////////////
	//Constructor given all paramaters
	RobotState(Direction dir, byte x, byte y, bool on_victim = false);

	//Default (assume we're at start location)
	RobotState();

	//Copy constructor
	RobotState(const RobotState& other);

	//Overload assignment operator
	RobotState& operator= (const RobotState& rhs);

	inline bool operator== (const RobotState& rhs) const
	{
		return bits_ == rhs.bits_;
	};

	//Setters //////////////////
	void SetDirection(Direction dir);

	void SetX(byte x);

	void SetY(byte y);

	//Set whenever we're on top of a victim
	void SetOnVictim(bool on_victim);

	//Getters //////////////////////
	Direction GetDirection() const;

	byte GetX() const;

	byte GetY() const;

	//Return true when we're on top of a victim.
	bool IsOnVictim() const;

private:
	std::bitset<ROBOT_STATE_SIZE> bits_;
};

//Class that represents the sate of the board.
class BoardState
{
public:
	BoardState();

	BoardState(std::bitset<BOARD_STATE_SIZE> init_states[64]);

	inline bool operator== (const BoardState& rhs) const
	{
		return arena_ == rhs.arena_;
	};

	//Test if a grid unit has a wall in specified direction
	bool HasWall(byte x, byte y, Direction dir) const;

	//Check if grid unit is yellow location drop zone
	bool IsYellow(byte x, byte y) const;

	//Check if grid unit is red location drop zone
	bool IsRed(byte x, byte y) const;

	//Check if grid unit is a victim location
	bool HasVictim(byte x, byte y) const;

	//Check if grid unit is a passable location (meaning the robot can move through it)
	bool IsPassable(byte x, byte y) const;

	//Set the state of where the right victim is.
	void SetRightVictimLocation(bool victim_is_up);

	//Set the state of where the left victim is.
	void SetLeftVictimLocation(bool victim_is_up);

	//Set flag for grid position at [x][y] for the victim to false
	void RemoveVictim(byte x, byte y);

private:
	std::bitset<BOARD_STATE_SIZE> arena_[8][8];
};

class RobotStateSet
{
public:
	RobotStateSet();

	//Add copy of state to the set (if possible, returns true, otherwise returns false)
	//Return true if successfully added
	//Return false if set already contains this state
	bool Add(const RobotState& state);

	//Return true if the set already has this state.
	bool Contains(const RobotState& state);
private:
	std::vector<RobotState> set_;
};
#endif