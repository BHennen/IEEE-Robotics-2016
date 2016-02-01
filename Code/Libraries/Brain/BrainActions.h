#ifndef BrainActions_H
#define BrainActions_H

class Brain; //Forward declare Brain class
#include <BrainEnums.h>
#include <States.h>

//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
class Action
{
public:
	//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
	Action(Brain* brain, const RobotState &state);

	//Execute the parenthesis operator.
	ActionResult Run();
	
	//Store resulting state this action would produce (when it is first constructed it is given this value from brain)
	RobotState new_state;

protected:
	//Let child classes overload this operator
	virtual ActionResult operator()() const = 0;

	Brain *brain_;
};

//Executes the follow wall function with given conditions and direction.
//Has success flags and error flags to return the correct action result.
//If still going, returns ACT_GOING.
//If error flag, returns which one.
//If success flag, returns ACT_SUCCESS.
class FollowWallAction : public Action
{
public:
	//Follow Wall //////////////////////////
	FollowWallAction(Brain* brain, const RobotState &state, Direction dir, StopConditions success_flags, StopConditions error_flags);

private:
	ActionResult operator()() const;
	Direction dir_;
	StopConditions success_flags_; //Which stop conditions signal that it was a success?
	StopConditions error_flags_; //Which stop conditions signal that it was an error?
};

//Executes the follow wall function with given direction.
//If still going, returns ACT_GOING.
//If done, returns ACT_SUCCESS.
class TravelPastWallAction : public Action
{
public:
	//Travel Past Wall //////////////////////////
	TravelPastWallAction(Brain* brain, const RobotState &state, Direction dir);


private:
	ActionResult operator()() const;
	Direction dir_;
};

//Executes the go to victim function.
//If still going, returns ACT_GOING.
//If done, returns ACT_SUCCESS.
class GoToVictimAction : public Action
{
public:
	//Go To Victim //////////////////////////
	GoToVictimAction(Brain* brain, const RobotState &state);


private:
	ActionResult operator()() const;
};

//Executes the Rotate90 function with given direction.
//If still going, returns ACT_GOING.
//If done, returns ACT_SUCCESS.
class Rotate90Action : public Action
{
public:
	//Rotate 90 //////////////////////////
	Rotate90Action(Brain* brain, const RobotState &state, Direction dir);

private:
	ActionResult operator()() const;
	Direction dir_;
};

#endif