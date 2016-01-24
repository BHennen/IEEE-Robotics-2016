#ifndef ActionList_H
#define ActionList_H

#include <iterator>
#include <vector>

//ActionList holds a list of actions to be executed.
//Can add actions and go through the available actions in the list.
//Creating a copy of the list will not make multiple copies of the individual functors,
//  since they will not need to be changed or replaced ever; instead it makes a pointer to
//  the original one created.
template <typename ActionType>
class ActionList
{
public:
	//Constructor
	ActionList();

	//Copy constructor
	ActionList(const ActionList &other);

	//Overload assignment operator
	ActionList& operator= (const ActionList& rhs);

	//Add a pointer to an action functor (eg: AddAction(new Action(arg1, arg2,...)))
	void AddAction(ActionType *action);

	//Return true if there are still more actions to go through
	bool HasActions();

	//Moves iterator to next action (or actions_.end() if no more actions)
	void MoveToNextAction();

	//Return pointer to the current Action
	ActionType* GetCurrentAction();

private:
	std::vector<ActionType*> actions_; //Array of ActionTypes (could be an action stored as a byte or functor)
	typename std::vector<ActionType*>::iterator curr_action_; //What action the list is on
	bool no_actions_added_; //Flag to set the action iterator
};

#endif