#ifndef ActionList_H
#define ActionList_H

class Action; //forward declare Action
#include <iterator>
#include <vector>
//ActionList holds a list of actions to be executed.
//Can add actions and go through the available actions in the list.
//Creating a copy of the list will not make multiple copies of the individual functors,
//  since they will not need to be changed or replaced ever; instead it makes a pointer to
//  the original one created.
class ActionList
{
public:
	//Constructor
	ActionList();

	//Add a pointer to an ActionType [eg: AddAction(new Action(arg1, arg2,...))
	//	or, if using bitset as actions: AddAction(new std::bitset<8>("10110010"))]
	void AddAction(Action *action);

	//Overload assignment operator
	ActionList& operator= (const ActionList& rhs);

	//Return true if there are still more actions to go through
	bool IsEmpty();

	//Moves iterator to next action (or actions_.end() if no more actions)
	void MoveToNextAction();

	//Return pointer to the current Action
	Action* GetCurrentAction();

private:
	std::vector<Action*> actions_; //Array of ActionTypes (could be an action stored as a byte or functor)
	typename std::vector<Action*>::iterator curr_action_; //What action the list is on
	bool no_actions_added_; //Flag to set the action iterator
};

#endif