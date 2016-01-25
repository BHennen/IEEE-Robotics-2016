#ifndef AStarSearch_H
#define AStarSearch_H

#include <ActionList.h>
#include <States.h>
#include <iterator>
#include <bitset>
#include <queue>
#include <vector>
#include <BrainEnums.h>

static const size_t BYTE_ACTION_SIZE = 16;
typedef std::vector<std::bitset<BYTE_ACTION_SIZE>> byte_action_list;
typedef std::bitset<BYTE_ACTION_SIZE> byte_action;

static const byte ROTATE_COST = 1; //default cost of rotating
static const byte TRAVEL_PAST_WALL_COST = 1; //default cost of travelling past a wall
static const byte FOLLOW_WALL_COST = 1; //default cost of following a wall
static const byte GO_TO_VICTIM_COST = 1; //default cost of going to a victim (once we're on them)

class SearchAlgorithm
{
public:
	//Use search algorithm called A* to create a list of actions that go to an end location from the current
	//robot state and board state.
	//Returns byte_action_list == std::vector<std::bitset<N>> (a vector of bitsets, where each bitset represents an
	//action, and can be converted into an ActionList of functors).
	//If search was unsuccessful, return an empty list.
	static byte_action_list AStarGoAToB(byte end_x, byte end_y, RobotState init_robot, BoardState init_board);
private:
	struct Successor
	{
		RobotState state;
		byte_action action;
		byte cost;
	};

	class SearchNode
	{
	public:
		SearchNode();
		SearchNode(RobotState state, byte_action_list actions, word g_cost, byte h_cost);
		bool operator >(const SearchNode &b) const;

		RobotState state;
		byte_action_list actions;
		word g_cost; //cumulative cost of actions required to get to this node.
		byte h_cost; //current hueristic cost
		word f_cost; //h_cost + g_cost
	};

	SearchAlgorithm();
	static byte Hueristic(byte end_x, byte end_y, RobotState robot, BoardState board);
	static byte_action GenerateRotateByteAction(Direction dir);
	static byte_action GenerateTravelPastWallByteAction(Direction dir);
	static byte_action GenerateFollowWallByteAction(Direction dir, StopConditions success_flags, StopConditions error_flags);
	static byte_action GenerateGoToVictimByteAction();
	static bool IsGoalState(RobotState current_state, byte end_x, byte end_y, BoardState init_board);
	static std::vector<Successor> GetSuccessors(RobotState curr_state, BoardState board_state);
};


#endif
