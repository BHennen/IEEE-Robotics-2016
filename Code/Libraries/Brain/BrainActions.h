#ifndef BrainActions_H
#define BrainActions_H

#include <Brain.h>
#include <BrainEnums.h>

//
//Create a type that is a vector of functions that have a shared return type and arguments, call it action_list.
//Will be used to store functions that will be executed in order.
//typedef std::vector<std::function<return_type (args)>> action_list;
//
//Example use:
//
//Create functions that have arguments bounded to them, ready to be called.
//auto f1 = std::bind(function_name1, arg1, arg2, ...);
//auto f2 = std::bind(function_name2, arg1, arg2, ...);
//
//Create list:
//action_list my_list;
//Add functions to list:
//my_list.push_back(f1);
//my_list.push_back(f2);
//
//Loop through list and execute the function. (Shouldnt use for loop in arduino, however)
//for(auto& f : my_list)
//{
//	f();
//}
//std::list AStarSearch();

//IDEA:
//During search, store actions as a byte:
//		00000000
//		12345678
//
//Where bytes 123 represent the action (for a total of 8 actions)
//And bytes 45678 represent the parameters (specialized for each action)
//
//During runtime, these would be stored as a list but wouldnt need to be accessed by the actual search
//function, so this would save memory space.
//
//After the search has completed and a solution found, the list is converted to a list of ActionFunctors
//This would save processor power because it can run each function directly without having to convert the byte
// into a function
//It would take up a little bit of memory (assuming the list of actions is ~20 or less)

//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
class Action
{
public:
	Action(Brain* brain);
	virtual ActionResult operator()() = 0;
	ActionResult Run(); //Execute the parenthesis operator.

protected:
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
	FollowWallAction(Brain* brain, Direction dir, StopConditions success_flags, StopConditions error_flags);
	virtual ActionResult operator()();
private:
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
	TravelPastWallAction(Brain* brain, Direction dir);
	virtual ActionResult operator()();
private:
	Direction dir_;
};

//Executes the go to victim function.
//If still going, returns ACT_GOING.
//If done, returns ACT_SUCCESS.
class GoToVictimAction : public Action
{
public:
	GoToVictimAction(Brain* brain);
	virtual ActionResult operator()();
private:
};

//Executes the Rotate90 function with given direction.
//If still going, returns ACT_GOING.
//If done, returns ACT_SUCCESS.
class Rotate90Action : public Action
{
public:
	Rotate90Action(Brain* brain, Direction dir);
	virtual ActionResult operator()();
private:
	Direction dir_;
};
#endif