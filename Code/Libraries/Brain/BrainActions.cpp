#include <BrainActions.h>
#include <Brain.h>

Action::Action(Brain* brain, RobotState state, Direction dir, StopConditions success_flags, StopConditions error_flags, uint8_t act_num)
{
	brain_ = brain;
	new_state = state;
	dir_ = dir;
	suc_flags_ = success_flags;
	err_flags_ = error_flags;
	act_num_ = act_num;
}

ActionResult Action::Run()
{
	switch(act_num_)
	{
	case 0: //Rotate
		if(brain_->Rotate90(dir_))
		{
			brain_->robot_state_ = new_state;
			return ACT_SUCCESS;
		}
		else
		{
			return ACT_GOING;
		}
		break;
	case 1: //Travel Past Wall
		if(brain_->TravelPastWall(dir_))
		{
			brain_->robot_state_ = new_state;
			return ACT_SUCCESS;
		}
		else
		{
			return ACT_GOING;
		}
		break;		
	case 2: //Follow Wall
		{
			StopConditions ret_flag = brain_->FollowWall(dir_, suc_flags_ | err_flags_);
			if(ret_flag == StopConditions::NONE)
			{
				return ACT_GOING;
			}
			else if(any_flags<StopConditions, byte>(ret_flag & err_flags_))
			{
				return ActionResult(ret_flag);
			}
			else if(any_flags<StopConditions, byte>(ret_flag & suc_flags_))
			{
				brain_->robot_state_ = new_state;
				return ACT_SUCCESS;
			}
		}
		break;
	case 3: //Go To Victim
		if(brain_->GoToVictim())
		{
			brain_->robot_state_ = new_state;
			return ACT_SUCCESS;
		}
		else
		{
			return ACT_GOING;
		}
		break;
	}
}

//
////Follow Wall //////////////////////////
//FollowWallAction::FollowWallAction(Brain& brain, RobotState state, Direction dir, StopConditions success_flags,
//								   StopConditions error_flags)
//	: Action(brain, state), dir_(dir), success_flags_(success_flags), error_flags_(error_flags)
//{
//}
//
//ActionResult FollowWallAction::Run()
//{
//	StopConditions ret_flag = brain_->FollowWall(dir_, success_flags_ | error_flags_);
//	if(ret_flag == StopConditions::NONE)
//	{
//		return ACT_GOING;
//	}
//	else if(any_flags<StopConditions, byte>(ret_flag & error_flags_))
//	{
//		return ActionResult(ret_flag);
//	}
//	else if(any_flags<StopConditions, byte>(ret_flag & success_flags_))
//	{
//		brain_->robot_state_ = new_state;
//		return ACT_SUCCESS;
//	}
//}
//
//
////Travel Past Wall //////////////////////////
//TravelPastWallAction::TravelPastWallAction(Brain& brain, RobotState state, Direction dir)
//	: Action(brain, state), dir_(dir)
//{
//
//}
//
//ActionResult TravelPastWallAction::Run()
//{
//	Serial.println("travel past wall");
//	return ACT_GOING;
//	if(brain_->TravelPastWall(dir_))
//	{
//		brain_->robot_state_ = new_state;
//		return ACT_SUCCESS;
//	}
//	else
//	{
//		return ACT_GOING;
//	}
//}
//
//
////Go To Victim //////////////////////////
//GoToVictimAction::GoToVictimAction(Brain& brain, RobotState state) : Action(brain, state)
//{
//
//}
//
//ActionResult GoToVictimAction::Run()
//{
//	Serial.println("go to victim");
//	return ACT_GOING;
//	if(brain_->GoToVictim())
//	{
//		brain_->robot_state_ = new_state;
//		return ACT_SUCCESS;
//	}
//	else
//	{
//		return ACT_GOING;
//	}
//}
//
////Rotate 90 //////////////////////////
//Rotate90Action::Rotate90Action(Brain& brain, RobotState state, Direction dir) : Action(brain, state), dir_(dir)
//{
//
//}
//
//ActionResult Rotate90Action::Run()
//{
//	Serial.println("rotate90");
//	return ACT_GOING;
//	if(brain_->Rotate90(dir_))
//	{
//		brain_->robot_state_ = new_state;
//		return ACT_SUCCESS;
//	}
//	else
//	{
//		return ACT_GOING;
//	}
//}