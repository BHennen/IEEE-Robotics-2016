#ifndef ActionList_H
#define ActionList_H

//class Action; //forward declare Action
#include <BrainActions.h>
#include <Arduino.h>
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

	//Destructor
	~ActionList();

	//Add a pointer to an ActionType [eg: AddAction(new Action(arg1, arg2,...))
	//	or, if using bitset as actions: AddAction(new std::bitset<8>("10110010"))]
	void AddAction(Action *action);

	//Overload assignment operator
	ActionList& operator= (const ActionList& rhs);

	//Return true if there are no more actions to go through
	bool IsEmpty();

	//Moves iterator to next action (or actions_.end() if no more actions)
	void MoveToNextAction();

	//Return pointer to the current Action
	Action* GetCurrentAction();

private:
	Action* actions_[25]; //Array of Action Pointers (25 actions should be plenty)
	uint8_t curr_action_ = 0; //What action the list is on
	uint8_t num_actions_ = 0; //How many actions stored.
};

#endif