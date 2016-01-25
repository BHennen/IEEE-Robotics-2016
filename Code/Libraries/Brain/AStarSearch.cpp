#include <AStarSearch.h>

//SearchNode //////////////////////
SearchAlgorithm::SearchNode::SearchNode()
{

}

SearchAlgorithm::SearchNode::SearchNode(RobotState state, byte_action_list actions, word g_cost, byte h_cost)
{
	this->state = state;
	this->actions = actions;
	this->g_cost = g_cost;
	this->h_cost = h_cost;
	f_cost = g_cost + h_cost;
}

bool SearchAlgorithm::SearchNode::operator >(const SearchNode &b) const
{
	return f_cost > b.f_cost;
}

//SearchAlgorithm //////////////////////

//Generate byte version of rotate action
//		...76543210		X = 0 -> left; 1 -> right	P = program(000)
//		-------XPPP
byte_action SearchAlgorithm::GenerateRotateByteAction(Direction dir)
{
	if(dir == LEFT)
	{
		return byte_action(0000);
	}
	else
	{
		return byte_action(1000);
	}
}

//Generate byte version of travelpastwall action
//		..876543210		X = 0 -> left; 1 -> right	P = program(001)
//		-------XPPP
byte_action SearchAlgorithm::GenerateTravelPastWallByteAction(Direction dir)
{
	if(dir == LEFT)
	{
		return byte_action(0001);
	}
	else
	{
		return byte_action(1001);
	}
}

//Generate byte version of FollowWall action
//		..9876543210		X = 0 -> left; 1 -> right	P = program(010)
//		ZZZYYY00XPPP		Y = success_flags			Z = error_flags
byte_action SearchAlgorithm::GenerateFollowWallByteAction(Direction dir, StopConditions success_flags, StopConditions error_flags)
{
	int suc_flags = static_cast<int>(success_flags) << 6;
	int err_flags = static_cast<int>(success_flags) << 9;
	if(dir == LEFT)
	{
		return byte_action(err_flags + suc_flags + 0010);
	}
	else
	{
		return byte_action(err_flags + suc_flags + 1010);
	}
}

//Generate byte version of GoToVictim action
//		..9876543210		P = program(011)
//		---------PPP		
byte_action SearchAlgorithm::GenerateGoToVictimByteAction()
{
	return byte_action(011);
}

//Check if the given state is the goal state (ignores rotation. shouldnt matter?)
//Takes into account the ending location and what is in that location
bool SearchAlgorithm::IsGoalState(RobotState current_state, byte end_x, byte end_y, BoardState board_state)
{
	//Set goal to be at end position and current direction.
	RobotState goal_state(current_state.GetDirection(), end_x, end_y);

	//If the end location has a victim, make sure goal state knows that robot has to be on the victim.
	if(board_state.HasVictim(end_x, end_y))
		goal_state.SetOnVictim(true);

	return current_state == goal_state;
}

//Given the robot state and the board state, find the successors to that state (which include the resulting state,
//		the action required to get to that state, and the cost of the action) and return those as a vector.
std::vector<SearchAlgorithm::Successor> SearchAlgorithm::GetSuccessors(RobotState curr_state, BoardState board_state)
{
	std::vector<Successor> successors;

	//Get current robot values
	Direction robot_dir = curr_state.GetDirection();
	byte robot_x = curr_state.GetX();
	byte robot_y = curr_state.GetY();

	//Check if our current position has a victim and we're not on top of it
	if(board_state.HasVictim(robot_x, robot_y) && !curr_state.IsOnVictim())
	{
		//Generate GoToVictim successor (do not bother with others; if we occupy the same space as a victim
		//We only need one step to the goal state.
		Successor go_to_victim =
		{
			RobotState(robot_dir, robot_x, robot_y, true), //Same location but now it is on top of the victim
			GenerateGoToVictimByteAction(),
			GO_TO_VICTIM_COST
		};
		successors.push_back(go_to_victim);
	}
	else
	{
		Direction right_of_robot = MapRotationToNewDirection(RIGHT, robot_dir);
		Direction left_of_robot = MapRotationToNewDirection(LEFT, robot_dir);

		//Generate rotation successors /////////////////////
		Successor rotate_right =
		{
			RobotState(right_of_robot, robot_x, robot_y),
			GenerateRotateByteAction(RIGHT),
			ROTATE_COST 
		};
		successors.push_back(rotate_right);
		Successor rotate_left =
		{
			RobotState(left_of_robot, robot_x, robot_y),
			GenerateRotateByteAction(LEFT),
			ROTATE_COST
		};
		successors.push_back(rotate_right);

		//Follow Wall check and generator lambda function
		//This lambda looks at a changing variable and checks in a direction for a stop condition. Whenever it finds one, it
		//creates a successor and then returns true (shouldn't be called again for this state).
		//Stop conditions: Front, Gap, Pixy, Pixy | Front, impassable
		//change_var: the variable thats changing (looping over)
		//x_changing: true if x is changing, false if y is changing
		//dir: Direction to check
		//Return true if stop condition found (travelPastWall not suitable or successor created).
		auto CheckAndMakeFollowWallAction = [robot_dir, robot_x, robot_y, right_of_robot, left_of_robot, board_state]
			(byte change_var, bool x_changing, Direction dir, std::vector<Successor> &successors) -> bool
		{
			Direction wall_dir = (dir == LEFT) ? left_of_robot : right_of_robot;
			Direction behind = MapRotationToNewDirection(dir, wall_dir); //opposite of where we're facing
			char x_var;
			char y_var;
			char dist_travelled = 0;
			if(x_changing)
			{
				x_var = change_var;
				y_var = robot_y;
				dist_travelled = abs(x_var - robot_x);
			}
			else
			{
				x_var = robot_x;
				y_var = change_var;
				dist_travelled = abs(y_var - robot_y);
			}
			
			//Check if location is passable. If not, we cannot perform this action UNLESS there is a victim
			//		in this location.
			if(!board_state.IsPassable(x_var, y_var))
			{
				//If victim in this location and impassable, we've followed the wall until victim; generate successor.
				//Action is to follow the wall until PIXY condition
				if(board_state.HasVictim(x_var, y_var))
				{
					Successor follow_wall_victim =
					{
						RobotState(robot_dir, x_var, y_var),
						GenerateFollowWallByteAction(dir, StopConditions::PIXY, StopConditions::NONE),
						static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
					};
					successors.push_back(follow_wall_victim);
				}
				return true;
			}

			//Check if current location has a victim (but is passable terrain) This indicates a victim may or may not
			//be here (those pesky victims). If so, generate successor with Pixy success flag and Front error flag
			//(pixy indicates there was a victim here while front indicates no victim)
			if(board_state.HasVictim(x_var, y_var))
			{
				Successor follow_wall_victim =
				{
					RobotState(robot_dir, x_var, y_var),
					GenerateFollowWallByteAction(dir, StopConditions::PIXY, StopConditions::FRONT),
					static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
				};
				successors.push_back(follow_wall_victim);
				return true;
			}

			//Check if current location has a wall in front (same direction we're facing). If so, generate successor.
			if(board_state.HasWall(x_var, y_var, dir))
			{
				Successor follow_wall_victim =
				{
					RobotState(robot_dir, x_var, y_var),
					GenerateFollowWallByteAction(dir, StopConditions::FRONT, StopConditions::NONE),
					static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
				};
				successors.push_back(follow_wall_victim);
				return true;
			}

			//Check if current location doesnt have a wall in direction we're looking for. (gap stop condition)
			if(!board_state.HasWall(x_var, y_var, wall_dir)) //check parallel to direction of motion
			{
				//Don't generate successor if we don't move.
				if(x_changing)
				{
					if(x_var == robot_x) return true;
				}
				else
				{
					if(y_var == robot_y) return true;
				}
				//good stop position. generate successor
				Successor follow_wall_victim =
				{
					RobotState(robot_dir, x_var, y_var),
					GenerateFollowWallByteAction(dir, StopConditions::GAP, StopConditions::NONE),
					static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
				};
				successors.push_back(follow_wall_victim);
				return true;
			}
			return false;
		};

		//Travel Past Wall check and generator lambda function
		//This lambda looks at a changing variable and checks in a direction for a stop condition. Whenever it finds one, it
		//creates a successor and then returns true (shouldn't be called again for this state).
		//Stop conditions: Wall found (either perpendicular or parallel), impassable
		//change_var: the variable thats changing (looping over)
		//x_changing: true if x is changing, false if y is changing
		//dir: Direction to check
		//Return true if stop condition found (travelPastWall not suitable or successor created).
		auto CheckAndMakeTravelPastWallAction = [robot_dir, robot_x, robot_y, right_of_robot, left_of_robot, board_state]
			(byte change_var, bool x_changing, Direction dir, std::vector<Successor> &successors) -> bool
		{
			Direction wall_dir = (dir == LEFT) ? left_of_robot : right_of_robot;
			Direction behind = MapRotationToNewDirection(dir, wall_dir); //opposite of where we're facing
			char x_var;
			char y_var;
			char dist_travelled = 0;
			if(x_changing)
			{
				x_var = change_var;
				y_var = robot_y;
				dist_travelled = abs(x_var - robot_x);
			}
			else
			{
				x_var = robot_x;
				y_var = change_var;
				dist_travelled = abs(y_var - robot_y);
			}

			//Check if current location is passable. If not, we cannot perform this action
			if(!board_state.IsPassable(x_var, y_var))
			{
				return true;
			}

			//Check if current location has a wall nearby. If so, we stop here.
			bool wall_found;
			if(board_state.HasWall(x_var, y_var, wall_dir)) //check parallel to direction of motion
			{
				wall_found = true;
			}
			else 
			{
				//check to see if there is a wall perpendicular to path of motion (in this case it would be one tile
				//over in the same direction we're looking for a wall, and the wall is behind the robot.)
				//Also Make sure not to check off the map
				switch(wall_dir)
				{
				case UP: //check tiles to the north of us
					if(y_var < 7)
						wall_found = board_state.HasWall(x_var, y_var + 1, behind);
					break;
				case RIGHT: //check tiles to the east of us
					if(x_var < 7)
						wall_found = board_state.HasWall(x_var + 1, y_var, behind);
					break;
				case DOWN: //check tiles to the south of us
					if(y_var > 0)
						wall_found = board_state.HasWall(x_var, y_var - 1, behind);
					break;
				case LEFT: //check tiles to the west of us
					if(x_var > 0)
						wall_found = board_state.HasWall(x_var - 1, y_var, behind);
					break;
				}
			}

			//generate successor function if wall found (stop condition reached)
			if(wall_found)
			{
				//Don't generate successor if we don't move.
				if(x_changing)
				{
					if(x_var == robot_x) return true;
				}
				else
				{
					if(y_var == robot_y) return true;
				}
				//good stop position. generate successor
				Successor travel_past_wall =
				{
					RobotState(robot_dir, x_var, y_var),
					GenerateTravelPastWallByteAction(dir),
					static_cast<byte>(TRAVEL_PAST_WALL_COST + dist_travelled)
				};
				successors.push_back(travel_past_wall);
				return true;
			}
			return false;
		};

		//Perform the actual checks and generation using the lambda function for the desired direction and function.
		bool check_left_TPW = true;
		bool check_right_TPW = true;
		bool check_left_FW = true;
		bool check_right_FW = true;
		switch(robot_dir)
		{
		case UP: //travel upwards
			for(char y = robot_y; y < 8; ++y)
			{
				if(check_left_TPW && CheckAndMakeTravelPastWallAction(y, false, LEFT, successors))
				{
					check_left_TPW = false; //Found stopping point on left side. Stop checking left
				}
				if(check_right_TPW && CheckAndMakeTravelPastWallAction(y, false, RIGHT, successors))
				{
					check_right_TPW = false; //Found stopping point on right side. Stop checking right
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(y, false, LEFT, successors))
				{
					check_left_FW = false; //Found stopping point on left side
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(y, false, RIGHT, successors))
				{
					check_right_FW = false; //Found stopping point on right side
				}
				if(!(check_left_TPW || check_right_TPW || check_left_FW || check_right_FW)) break; //done checking both sides, stop loop
			}
			break;
		case RIGHT:  //travel rightwards, checking left side
			for(char x = robot_x; x < 8; ++x)
			{
				if(check_left_TPW && CheckAndMakeTravelPastWallAction(x, true, LEFT, successors))
				{
					check_left_TPW = false;//Found stopping point on left side. Stop checking left
				}
				if(check_right_TPW && CheckAndMakeTravelPastWallAction(x, true, RIGHT, successors))
				{
					check_right_TPW = false;//Found stopping point on right side. Stop checking right
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(x, true, LEFT, successors))
				{
					check_left_FW = false; //Found stopping point on left side
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(x, true, RIGHT, successors))
				{
					check_right_FW = false; //Found stopping point on right side
				}
				if(!(check_left_TPW || check_right_TPW || check_left_FW || check_right_FW)) break; //done checking both sides, stop loop
			}
			break;
		case DOWN: //travel downwards
			for(char y = robot_y; y >= 0; --y)
			{
				if(check_left_TPW && CheckAndMakeTravelPastWallAction(y, false, LEFT, successors))
				{
					check_left_TPW = false;//Found stopping point on left side. Stop checking left
				}
				if(check_right_TPW && CheckAndMakeTravelPastWallAction(y, false, RIGHT, successors))
				{
					check_right_TPW = false;//Found stopping point on right side. Stop checking right
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(y, false, LEFT, successors))
				{
					check_left_FW = false; //Found stopping point on left side
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(y, false, RIGHT, successors))
				{
					check_right_FW = false; //Found stopping point on right side
				}
				if(!(check_left_TPW || check_right_TPW || check_left_FW || check_right_FW)) break; //done checking both sides, stop loop
			}
			break;
		case LEFT: // travel leftwards
			for(char x = robot_x; x >= 0; --x)
			{
				if(check_left_TPW && CheckAndMakeTravelPastWallAction(x, true, LEFT, successors))
				{
					check_left_TPW = false;//Found stopping point on left side. Stop checking left
				}
				if(check_right_TPW && CheckAndMakeTravelPastWallAction(x, true, RIGHT, successors))
				{
					check_right_TPW = false;//Found stopping point on right side. Stop checking right
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(x, true, LEFT, successors))
				{
					check_left_FW = false; //Found stopping point on left side
				}
				if(check_left_FW && CheckAndMakeFollowWallAction(x, true, RIGHT, successors))
				{
					check_right_FW = false; //Found stopping point on right side
				}
				if(!(check_left_TPW || check_right_TPW || check_left_FW || check_right_FW)) break; //done checking both sides, stop loop
			}
			break;
		}
	}
	return successors;
}

//Hueristic that attempts to estimate the remaining cost of travelling to desired position.
//Usually it provides a solution to the relaxed problem.
//A good hueristic can cut down drastically the number of nodes expanded (and by extension memory size and processing time)
byte SearchAlgorithm::Hueristic(byte end_x, byte end_y, RobotState robot, BoardState board)
{
	//For now, just use manhattan distance as the hueristic
	return abs(end_x - robot.GetX()) + abs(end_y + robot.GetY());
}

//A* Search algorithm. Execute a search to an end position, given the initial states of the robot and board.
//Returns a vector of byte_actions (to save space during search). Can be transformed into ActionFunctors.
byte_action_list SearchAlgorithm::AStarGoAToB(byte end_x, byte end_y, RobotState init_robot, BoardState board_state)
{
	//Create a set of RobotStates. Used to keep track of states we've already explored.
	RobotStateSet closed_set;

	//create priority queue of nodes, with highest priority being lower cost nodes.
	//Represents the fringe or frontier of what we haven't explored yet. We want to explore the cheapest cost node
	//		on the fringe first.
	std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> fringe;

	//initial node
	SearchNode startNode(init_robot, byte_action_list(), 0, 0);
	fringe.push(startNode);

	//Execute search
	while(!fringe.empty())
	{
		SearchNode curr_node = fringe.top(); //Get least cost node on fringe
		fringe.pop(); //Remove top
		if(IsGoalState(curr_node.state, end_x, end_y, board_state)) //If this node is a goal state, we're done! Return the actions required to get there.
			return curr_node.actions;
		if(closed_set.Add(curr_node.state))
		{
			//State was added successfully; It wasn't previously in the set
			//Now loop through all the successor paths from the current state
			for(Successor successor : GetSuccessors(curr_node.state, board_state))
			{
				//Copy list from current node into successor_actions, add successor action to that list
				byte_action_list successor_actions(curr_node.actions);
				successor_actions.push_back(successor.action);

				//Cost of traveling to this node and the cost of performing the successor's action
				word successor_g_cost = curr_node.g_cost + successor.cost;

				//Cost of the heuristic (how close we're estimated to being finished)
				byte successor_h_cost = Hueristic(end_x, end_y, successor.state, board_state);

				//Create new node from this successor
				SearchNode successor_node(successor.state, successor_actions, successor_g_cost, successor_h_cost);

				//Put this successor in priority queue
				fringe.push(successor_node);
			}
		}
	}

	//We looped through all nodes on the fringe without finding a solution; return empty list
	return byte_action_list();
};