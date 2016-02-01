#include <BrainActions.h>
#include <Brain.h>

//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
Action::Action(Brain* brain, const RobotState &state)
{
	brain_ = brain;
	new_state = state;
}

ActionResult Action::Run() //Execute the parenthesis operator.
{
	ActionResult result = (*this)();
	if(result == ACT_SUCCESS)
	{
		brain_->robot_state_ = new_state;
	}
	return result;
}


//Follow Wall //////////////////////////
FollowWallAction::FollowWallAction(Brain* brain, const RobotState &state, Direction dir, StopConditions success_flags,
								   StopConditions error_flags)
	: Action(brain, state), dir_(dir), success_flags_(success_flags), error_flags_(error_flags)
{

}

ActionResult FollowWallAction::operator()() const
{
	StopConditions ret_flag = brain_->FollowWall(dir_, success_flags_ | error_flags_);
	if(ret_flag == StopConditions::NONE)
	{
		return ACT_GOING;
	}
	else if(any_flags<StopConditions, byte>(ret_flag & error_flags_))
	{
		return ActionResult(ret_flag);
	}
	else if(any_flags<StopConditions, byte>(ret_flag & success_flags_))
	{
		return ACT_SUCCESS;
	}
}


//Travel Past Wall //////////////////////////
TravelPastWallAction::TravelPastWallAction(Brain* brain, const RobotState &state, Direction dir)
	: Action(brain, state), dir_(dir)
{

}

ActionResult TravelPastWallAction::operator()() const
{
	if(brain_->TravelPastWall(dir_))
	{
		return ACT_SUCCESS;
	}
	else
	{
		return ACT_GOING;
	}
}


//Go To Victim //////////////////////////
GoToVictimAction::GoToVictimAction(Brain* brain, const RobotState &state) : Action(brain, state)
{

}

ActionResult GoToVictimAction::operator()() const
{
	if(brain_->GoToVictim())
	{
		return ACT_SUCCESS;
	}
	else
	{
		return ACT_GOING;
	}
}

//Rotate 90 //////////////////////////
Rotate90Action::Rotate90Action(Brain* brain, const RobotState &state, Direction dir) : Action(brain, state), dir_(dir)
{

}

ActionResult Rotate90Action::operator()() const
{
	if(brain_->Rotate90(dir_))
	{
		return ACT_SUCCESS;
	}
	else
	{
		return ACT_GOING;
	}
}