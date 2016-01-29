#include <ActionList.h>

//Constructor
ActionList::ActionList() : no_actions_added_(true)
{};

//Add a pointer to an ActionType [eg: AddAction(new Action(arg1, arg2,...))
//	or, if using bitset as actions: AddAction(new std::bitset<8>("10110010"))]
void ActionList::AddAction(Action *action)
{
	actions_.push_back(action);
	if(no_actions_added_)
	{
		no_actions_added_ = false;
		curr_action_ = actions_.begin();
	}
}

//Overload assignment operator
ActionList& ActionList::operator= (const ActionList& rhs)
{
	if(this == &rhs)
		return *this;
	no_actions_added_ = rhs.no_actions_added_;
	curr_action_ = rhs.curr_action_;
	actions_ = rhs.actions_;
	return *this;
}

//Return true if there are still more actions to go through
bool ActionList::IsEmpty()
{
	return curr_action_ == actions_.end();
}

//Moves iterator to next action (or actions_.end() if no more actions)
void ActionList::MoveToNextAction()
{
	++curr_action_;
}

//Return pointer to the current ActionType
Action* ActionList::GetCurrentAction()
{
	return *curr_action_;
}