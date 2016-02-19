#include <AStarSearch.h>

void SearchAlgorithm::PrintByteActionString(Bitset<word> bits)
{
	word prog = bits.Test(2) * 4 + bits.Test(1) * 2 + bits.Test(0) * 1;
	if(prog == 0)
	{
		if(bits.Test(3))
		{
			Serial.println(F("ROTATE RIGHT"));
		}
		else
		{
			Serial.println(F("ROTATE LEFT"));
		}
	}
	else if(prog == 1)
	{
		if(bits.Test(3))
		{
			Serial.println(F("TPW RIGHT"));
		}
		else
		{
			Serial.println(F("TPW LEFT"));
		}
	}
	else if(prog == 2)
	{
		byte len = 0;
		char buf[60];		
		len += sprintf(buf + len, "FOLLOW ");
		word err = bits.Test(11) * 4 + bits.Test(10) * 2 + bits.Test(9) * 1;
		word suc = bits.Test(8) * 4 + bits.Test(7) * 2 + bits.Test(6) * 1;
		if(bits.Test(3))
		{			
			len += sprintf(buf + len, "RIGHT");
		}
		else
		{
			len += sprintf(buf + len, "LEFT");
		}
				
		len += sprintf(buf + len, " success: ");
		if(suc == static_cast<byte>(StopConditions::NONE))
		{
			len += sprintf(buf + len, "NONE ");
		}
		else if(suc == static_cast<byte>(StopConditions::GAP))
		{			
			len += sprintf(buf + len, "GAP ");
		}
		else if(suc == static_cast<byte>(StopConditions::FRONT))
		{			
			len += sprintf(buf + len, "FRONT ");
		}
		else if(suc == static_cast<byte>(StopConditions::PIXY))
		{			
			len += sprintf(buf + len, "PIXY ");
		}
				
		len += sprintf(buf + len, "fail: ");
		if(!any_flags<word, byte>(err))
		{			
			len += sprintf(buf + len, "NONE ");
		}
		if((err & static_cast<byte>(StopConditions::GAP)) > 0)
		{			
			len += sprintf(buf + len, "GAP ");
		}
		if((err & static_cast<byte>(StopConditions::FRONT)) > 0)
		{
			len += sprintf(buf + len, "FRONT ");
		}
		if((err & static_cast<byte>(StopConditions::PIXY)) > 0)
		{
			len += sprintf(buf + len, "PIXY ");
		}

		Serial.println(buf);
	}
	else if(prog == 3)
	{
		Serial.println(F("GO TO VICTIM"));
	}
}

//SearchNode //////////////////////
SearchAlgorithm::SearchNode::SearchNode()
{

}

SearchAlgorithm::SearchNode::SearchNode(RobotState state, byte_action_list actions, byte g_cost, byte h_cost)
{
	this->state = state;
	this->actions = actions;
	this->g_cost = g_cost;
	f_cost = g_cost + h_cost;
}

bool SearchAlgorithm::SearchNode::operator >(const SearchNode &b) const
{
	return f_cost > b.f_cost;
}

//Generate byte version of rotate action
//		...76543210		X = 0 -> left; 1 -> right	P = program(000)
//		-------XPPP
byte_action SearchAlgorithm::GenerateRotateByteAction(Direction dir)
{
	if(dir == LEFT)
	{
		return byte_action(B0000);
	}
	else
	{
		return byte_action(B1000);
	}
}

//Generate byte version of travelpastwall action
//		..876543210		X = 0 -> left; 1 -> right	P = program(001)
//		-------XPPP
byte_action SearchAlgorithm::GenerateTravelPastWallByteAction(Direction dir)
{
	if(dir == LEFT)
	{
		return byte_action(B0001);
	}
	else
	{
		return byte_action(B1001);
	}
}

//Generate byte version of FollowWall action
//		..9876543210		X = 0 -> left; 1 -> right	P = program(010)
//		ZZZYYY00XPPP		Y = success_flags			Z = error_flags
byte_action SearchAlgorithm::GenerateFollowWallByteAction(Direction dir, StopConditions success_flags, StopConditions error_flags)
{
	word suc_flags = static_cast<word>(success_flags) << 6;
	word err_flags = static_cast<word>(error_flags) << 9;
	if(dir == LEFT)
	{
		return byte_action(err_flags + suc_flags + B0010);
	}
	else
	{
		return byte_action(err_flags + suc_flags + B1010);
	}
}

//Generate byte version of GoToVictim action
//		..9876543210		P = program(011)
//		---------PPP		
byte_action SearchAlgorithm::GenerateGoToVictimByteAction()
{
	return byte_action(B011);
}

//Check if the given state is the goal state (ignores rotation. shouldnt matter?)
//Takes into account the ending location and what is in that location
bool SearchAlgorithm::IsGoalState(RobotState current_state, byte end_x, byte end_y, BoardState board_state, int desired_direction)
{
	//Set goal to be at end position and desired direction (or if -1, our current direction).
	Direction dir = (desired_direction == -1) ? current_state.GetDirection() : static_cast<Direction>(desired_direction);
	RobotState goal_state(dir, end_x, end_y);

	//If the end location has a victim, make sure goal state knows that robot has to be on the victim.
	if(board_state.HasVictim(end_x, end_y))
		goal_state.SetOnVictim(true);

	return current_state == goal_state;
}

bool SearchAlgorithm::GenerateRotateSuccessor(RobotState &curr_state, BoardState &board_state, Direction dir, Successor &successor)
{
	//Get current robot values
	Direction robot_dir = curr_state.GetDirection();
	byte robot_x = curr_state.GetX();
	byte robot_y = curr_state.GetY();
	Direction new_dir = (dir == LEFT) ? MapRotationToNewDirection(LEFT, robot_dir) : MapRotationToNewDirection(RIGHT, robot_dir);
	Successor rotate =
	{
		RobotState(new_dir, robot_x, robot_y),
		GenerateRotateByteAction(dir),
		ROTATE_COST
	};
	successor = rotate;
	return true;
}

bool SearchAlgorithm::GenerateGoToVictimSuccessor(RobotState &curr_state, BoardState &board_state, Successor &successor)
{
	//Get current robot values
	Direction robot_dir = curr_state.GetDirection();
	byte robot_x = curr_state.GetX();
	byte robot_y = curr_state.GetY();
	if(board_state.HasVictim(robot_x, robot_y) && !curr_state.IsOnVictim())
	{
		//Generate GoToVictim successor
		Successor go_to_victim =
		{
			RobotState(robot_dir, robot_x, robot_y, true), //Same location but now it is on top of the victim
			GenerateGoToVictimByteAction(),
			GO_TO_VICTIM_COST
		};
		successor = go_to_victim;
		return true;
	}
	return false;
}

bool SearchAlgorithm::GenerateTravelPastWallSuccessor(RobotState &curr_state, BoardState &board_state, Direction dir, Successor &successor)
{
	//Get current robot values
	Direction robot_dir = curr_state.GetDirection();
	byte robot_x = curr_state.GetX();
	byte robot_y = curr_state.GetY();
	Direction wall_dir = (dir == LEFT) ? MapRotationToNewDirection(LEFT, robot_dir) : MapRotationToNewDirection(RIGHT, robot_dir);
	Direction behind = MapRotationToNewDirection(dir, wall_dir); //opposite of where we're facing

	auto CheckAndMakeTravelPastWallAction = [&robot_dir, &robot_x, &robot_y, &wall_dir, &board_state, &dir, &successor, &behind]
		(byte change_var, bool x_changing) -> bool
	{
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

		//check if we've moved past a wall (therefore this is an illegal move, return)
		if(dist_travelled > 0 and board_state.HasWall(x_var, y_var, behind))
		{
			return true;
		}

		//Check if current location has a wall nearby. If so, we stop here.
		bool wall_found = false;
		if(board_state.HasWall(x_var, y_var, wall_dir)) //check parallel to direction of motion
		{
			wall_found = true;
		}
		else
		{
			//check to see if there is a wall perpendicular to path of motion (in this case it would be one tile
			//over in the same direction we're looking for a wall, and the wall is behind the robot.)

			//Don't check if wall behind us if we didn't move, but continue to check so return false.
			if(dist_travelled == 0) return false;

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
			if(dist_travelled == 0) return true;

			//good stop position. generate successor
			Successor travel_past_wall =
			{
				RobotState(robot_dir, x_var, y_var),
				GenerateTravelPastWallByteAction(dir),
				//Traveling past wall has exponential cost for distanct travelled
				static_cast<byte>(pow(dist_travelled, TRAVEL_PAST_WALL_COST))
			};
			successor = travel_past_wall;
			return true;
		}
		return false;
	};


	Successor original = successor; //Save original successor before we try to find a new one
	switch(robot_dir)
	{
	case UP: //travel upwards
		for(char y = robot_y; y < 8; ++y)
		{
			if(CheckAndMakeTravelPastWallAction(y, false))
			{
				break;
			}
		}
		break;
	case RIGHT:  //travel rightwards, checking left side
		for(char x = robot_x; x < 8; ++x)
		{
			if(CheckAndMakeTravelPastWallAction(x, true))
			{
				break;
			}
		}
		break;
	case DOWN: //travel downwards
		for(char y = robot_y; y >= 0; --y)
		{
			if(CheckAndMakeTravelPastWallAction(y, false))
			{
				break;
			}
		}
		break;
	case LEFT: // travel leftwards
		for(char x = robot_x; x >= 0; --x)
		{
			if(CheckAndMakeTravelPastWallAction(x, true))
			{
				break;
			}
		}
		break;
	}

	//Check if current successor is the same as it was as when we entered the function
	if(successor == original)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//Will return true if the function found a successor, and update the referenced successor. 
//Returns false if no successor found, and does not update the referenced successor.
bool SearchAlgorithm::GenerateFollowWallSuccessor(RobotState &curr_state, BoardState &board_state, Direction dir,
												  Successor &successor)
{
	//Get current robot values
	Direction robot_dir = curr_state.GetDirection();
	byte robot_x = curr_state.GetX();
	byte robot_y = curr_state.GetY();
	Direction wall_dir = (dir == LEFT) ? MapRotationToNewDirection(LEFT, robot_dir) : MapRotationToNewDirection(RIGHT, robot_dir);

	auto CheckAndMakeFollowWallAction = [&robot_dir, &robot_x, &robot_y, &wall_dir, &board_state, &dir, &successor]
		(byte change_var, bool x_changing) -> bool
	{
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
				successor = follow_wall_victim;
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
			successor = follow_wall_victim;
			return true;
		}

		//Check if current location has a wall in front (same direction we're facing). If so, generate successor.
		if(board_state.HasWall(x_var, y_var, robot_dir))
		{
			Successor follow_wall_victim =
			{
				RobotState(robot_dir, x_var, y_var),
				GenerateFollowWallByteAction(dir, StopConditions::FRONT, StopConditions::NONE),
				static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
			};
			successor = follow_wall_victim;
			return true;
		}

		//Check if current location doesnt have a wall in direction we're looking for. (gap stop condition)
		if(!board_state.HasWall(x_var, y_var, wall_dir)) //check parallel to direction of motion
		{
			//Don't generate successor if we don't move.
			if(dist_travelled == 0) return true;

			//good stop position. generate successor
			Successor follow_wall_victim =
			{
				RobotState(robot_dir, x_var, y_var),
				GenerateFollowWallByteAction(dir, StopConditions::GAP, StopConditions::NONE),
				static_cast<byte>(FOLLOW_WALL_COST + dist_travelled)
			};
			successor = follow_wall_victim;
			return true;
		}
		return false;
	};

	Successor original = successor; //Save original successor before we try to find a new one
	switch(robot_dir)
	{
	case UP: //travel upwards
		for(char y = robot_y; y < 8; ++y)
		{
			if(CheckAndMakeFollowWallAction(y, false))
			{
				break;
			}
		}
		break;
	case RIGHT:  //travel rightwards, checking left side
		for(char x = robot_x; x < 8; ++x)
		{
			if(CheckAndMakeFollowWallAction(x, true))
			{
				break;
			}
		}
		break;
	case DOWN: //travel downwards
		for(char y = robot_y; y >= 0; --y)
		{
			if(CheckAndMakeFollowWallAction(y, false))
			{
				break;
			}
		}
		break;
	case LEFT: // travel leftwards
		for(char x = robot_x; x >= 0; --x)
		{
			if(CheckAndMakeFollowWallAction(x, true))
			{
				break;
			}
		}
		break;
	}

	//Check if current successor is the same as it was as when we entered the function
	if(successor == original)
	{
		return false;
	}
	else
	{
		return true;
	}
};

//Given the robot state and the board state, find the successors to that state (which include the resulting state,
//		the action required to get to that state, and the cost of the action) and return those as a vector.
std::vector<Successor> SearchAlgorithm::GetSuccessors(RobotState curr_state, BoardState board_state)
{
	std::vector<Successor> successors;
	Successor temp_succ;

	//If we're on a victim, we can't progress(for now)
	if(curr_state.IsOnVictim())
	{
		return successors;
	}
	//Check if our current position has a victim and we're not on top of it
	else if(GenerateGoToVictimSuccessor(curr_state, board_state, temp_succ))
	{
		//Generate GoToVictim successor succeeded (do not bother with others; if we occupy the same space as a victim
		//We only need one step to the goal state.
		successors.push_back(temp_succ);
	}
	else //Check normal successors
	{
		//Generate rotation successors /////////////////////
		if(GenerateRotateSuccessor(curr_state, board_state, RIGHT, temp_succ))
		{
			successors.push_back(temp_succ);
		}
		if(GenerateRotateSuccessor(curr_state, board_state, LEFT, temp_succ))
		{
			successors.push_back(temp_succ);
		}

		//Generate Follow Wall successors
		if(GenerateFollowWallSuccessor(curr_state, board_state, RIGHT, temp_succ))
		{
			successors.push_back(temp_succ);
		}
		if(GenerateFollowWallSuccessor(curr_state, board_state, LEFT, temp_succ))
		{
			successors.push_back(temp_succ);
		}

		//Generate Travel Past Wall successors
		if(GenerateTravelPastWallSuccessor(curr_state, board_state, RIGHT, temp_succ))
		{
			successors.push_back(temp_succ);
		}
		if(GenerateTravelPastWallSuccessor(curr_state, board_state, LEFT, temp_succ))
		{
			successors.push_back(temp_succ);
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
	return abs(end_x - robot.GetX()) + abs(end_y - robot.GetY());
}

//A* Search algorithm. Execute a search to an end position, given the initial states of the robot and board.
//Returns a vector of byte_actions (to save space during search). Can be transformed into ActionFunctors.
byte_action_list SearchAlgorithm::AStarGoAToB(byte end_x, byte end_y, RobotState init_robot, BoardState board_state, int desired_direction/* = -1*/)
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
		if(IsGoalState(curr_node.state, end_x, end_y, board_state, desired_direction)) //If this node is a goal state, we're done! Return the actions required to get there.
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