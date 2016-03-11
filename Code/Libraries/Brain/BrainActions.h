#ifndef BrainActions_H
#define BrainActions_H

class Brain; //Forward declare Brain class
#include <BrainEnums.h>
#include <States.h>

//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
class Action
{
public:
	//Virtual class that is a functor (which stores a function [with arguments!] and can be called later).
	Action(Brain* brain, RobotState state, Direction dir, StopConditions success_flags, StopConditions error_flags, uint8_t act_num);

	//Run the selected action
	ActionResult Run();

	//Print 
	inline void Print()
	{
		switch(act_num_)
		{
		case 0:
			Serial.println("Prog 0: Rotate");
			break;
		case 1:
			Serial.println("Prog 1: TPW");
			break;
		case 2:
			Serial.println("Prog 2: FollowWall");
			break;
		case 3:
			Serial.println("Prog 3: GoVictim");
			break;
		}
	};

private:
	StopConditions suc_flags_;
	StopConditions err_flags_;
	Direction dir_;
	RobotState new_state;
	uint8_t act_num_;
	Brain* brain_;
};
//
////Executes the follow wall function with given conditions and direction.
////Has success flags and error flags to return the correct action result.
////If still going, returns ACT_GOING.
////If error flag, returns which one.
////If success flag, returns ACT_SUCCESS.
//class FollowWallAction : public Action
//{
//public:
//	//Follow Wall //////////////////////////
//	FollowWallAction(Brain& brain, RobotState state, Direction dir, StopConditions success_flags, StopConditions error_flags);
//
//	//Runs the FollowWall function
//	ActionResult Run();
//
//	inline void Print()
//	{
//		Serial.println("Follow Wall");
//	};
//
//private:
//	Direction dir_;
//	StopConditions success_flags_; //Which stop conditions signal that it was a success?
//	StopConditions error_flags_; //Which stop conditions signal that it was an error?
//};
//
////Executes the follow wall function with given direction.
////If still going, returns ACT_GOING.
////If done, returns ACT_SUCCESS.
//class TravelPastWallAction : public Action
//{
//public:
//	//Travel Past Wall //////////////////////////
//	TravelPastWallAction(Brain& brain, RobotState state, Direction dir);
//
//	//Runs the TravelPastWall function
//	ActionResult Run();
//
//	inline void Print()
//	{
//		Serial.println("TPW");
//	};
//
//private:
//	Direction dir_;
//};
//
////Executes the go to victim function.
////If still going, returns ACT_GOING.
////If done, returns ACT_SUCCESS.
//class GoToVictimAction : public Action
//{
//public:
//	//Go To Victim //////////////////////////
//	GoToVictimAction(Brain& brain, RobotState state);
//
//	//Runs the GoToVictim function
//	ActionResult Run();
//
//	inline void Print()
//	{
//		Serial.println("GTV");
//	};
//
//private:
//};
//
////Executes the Rotate90 function with given direction.
////If still going, returns ACT_GOING.
////If done, returns ACT_SUCCESS.
//class Rotate90Action : public Action
//{
//public:
//	//Rotate 90 //////////////////////////
//	Rotate90Action(Brain& brain, RobotState state, Direction dir);
//
//	//Runs the Rotate90 function
//	ActionResult Run();
//
//	inline void Print()
//	{
//		Serial.println("R90");
//	};
//
//private:
//	Direction dir_;
//};

#endif