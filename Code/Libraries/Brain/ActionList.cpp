#include <ActionList.h>

//Constructor
ActionList::ActionList()
{};

ActionList::~ActionList()
{
	//Delete all pointers to functors in this list
	for(uint8_t action = 0; action < num_actions_; action++)
	{
		delete actions_[action];
	}
}

//Add a pointer to an ActionType [eg: AddAction(new Action(arg1, arg2,...))
//	or, if using bitset as actions: AddAction(new std::bitset<8>("10110010"))]
void ActionList::AddAction(Action *action)
{
	actions_[num_actions_] = action;
	num_actions_++;
}

//Overload assignment operator
ActionList& ActionList::operator= (const ActionList& rhs)
{
	if(this == &rhs)
		return *this;
	curr_action_ = rhs.curr_action_;
	num_actions_ = rhs.num_actions_;
	for(uint8_t action = 0; action < num_actions_; action++)
	{
		actions_[action] = new Action(*rhs.actions_[action]);
	}
	return *this;
}

//Return true if there are still more actions to go through
bool ActionList::IsEmpty()
{
	return curr_action_ == num_actions_;
}

//Moves iterator to next action (or actions_.end() if no more actions)
void ActionList::MoveToNextAction()
{
	++curr_action_;
}

//Return pointer to the current ActionType
Action* ActionList::GetCurrentAction()
{
	return actions_[curr_action_];
}