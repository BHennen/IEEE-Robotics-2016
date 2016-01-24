#include <BrainActionFunctors.h>

//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
ActionFunctor::ActionFunctor(Brain* brain)
{
	brain_ = brain;
};

ActionResult ActionFunctor::Run() //Execute the parenthesis operator.
{
	return (*this)();
}

FollowWallActionFunctor::FollowWallActionFunctor(Brain* brain, Direction dir, StopConditions success_flags, StopConditions error_flags)
	: ActionFunctor(brain), dir_(dir), success_flags_(success_flags), error_flags_(error_flags)
{};

ActionResult FollowWallActionFunctor::operator()()
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
};