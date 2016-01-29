#ifndef AStarSearch_H
#define AStarSearch_H

#include <States.h>
#include <iterator>
#include <Bitset.h>
#include <queue>
#include <vector>
#include <BrainEnums.h>

typedef Bitset<word> byte_action;
typedef std::vector<byte_action> byte_action_list;

static const byte ROTATE_COST = 1; //default cost of rotating
//exponential cost of travelling past a wall, with regards to distance travelled (2 means it takes no risks travelling past a wall)
static const byte TRAVEL_PAST_WALL_COST = 2;
static const byte FOLLOW_WALL_COST = 1; //default cost of following a wall
static const byte GO_TO_VICTIM_COST = 0; //default cost of going to a victim (once we're on them)

class SearchAlgorithm
{
public:
	//Use search algorithm called A* to create a list of actions that go to an end location from the current
	//robot state and board state.
	//Returns byte_action_list == std::vector<Bitset<N>> (a vector of Bitsets, where each Bitset represents an
	//action, and can be converted into an ActionList of functors).
	//If search was unsuccessful, return an empty list.
	static byte_action_list AStarGoAToB(byte end_x, byte end_y, RobotState init_robot, BoardState init_board);

	template <typename Data_Type>
	static void PrintByteActionString(Bitset<Data_Type> bits);
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
		SearchNode(RobotState state, byte_action_list actions, byte g_cost, byte h_cost);
		bool operator >(const SearchNode &b) const;

		RobotState state;
		byte_action_list actions;
		byte g_cost; //cumulative cost of actions required to get to this node.
		byte f_cost; //h_cost + g_cost
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
