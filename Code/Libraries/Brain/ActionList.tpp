//Constructor
template <typename ActionType>
ActionList<ActionType>::ActionList(): no_actions_added_(false)
{
}

//Copy constructor
template <typename ActionType>
ActionList<ActionType>::ActionList(const ActionList &other)
{
	no_actions_added_ = other.no_actions_added_;
	curr_action_ = other.curr_action_;
	actions_ = other.actions_;
}

//Overload assignment operator
template <typename ActionType>
ActionList<ActionType>& ActionList<ActionType>::operator= (const ActionList<ActionType>& rhs)
{
	if(this == &rhs)
		return *this;
	no_actions_added_ = rhs.no_actions_added_;
	curr_action_ = rhs.curr_action_;
	actions_ = rhs.actions_;
	return *this;
}

//Add a pointer to an ActionType [eg: AddAction(new Action(arg1, arg2,...))
//	or, if using bitset as actions: AddAction(new std::bitset<8>("10110010"))]
template <typename ActionType>
void ActionList<ActionType>::AddAction(ActionType *action)
{
	actions_.push_back(action);
	if(no_actions_added_)
	{
		no_actions_added_ = false;
		curr_action_ = actions_.begin();
	}
}

//Return true if there are still more actions to go through
template <typename ActionType>
bool ActionList<ActionType>::HasActions()
{
	return curr_action_ != actions_.end();
}

//Moves iterator to next action (or actions_.end() if no more actions)
template <typename ActionType>
void ActionList<ActionType>::MoveToNextAction()
{
	++curr_action_;
}

//Return pointer to the current ActionType
template <typename ActionType>
ActionType* ActionList<ActionType>::GetCurrentAction()
{
	return *curr_action_;
}