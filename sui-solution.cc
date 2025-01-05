/**
 * @file sui-solution.cc
 * @author Jindřich Vodák (xvodak06)
 * @author Anastasiia Lebedenko (xlebed11)
 */

#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

#define MEMORY_LIMIT 50000000

typedef std::shared_ptr<SearchState> SharedPtr;

bool memoryLimitExceeded(size_t limit)
{
	return (getCurrentRSS() > limit - MEMORY_LIMIT);
}

struct StateCost {
    SearchState state;
    double g_score;  // Cost from start node to this node
    double f_score;  // Estimated cost from start to goal through this node

    // Comparator that compares the f_score for priority queue (min-heap)
    bool operator>(const StateCost& other) const {
        return this->f_score > other.f_score;
    }
};

// returns the path
std::vector<SearchAction> ReturnPath(SharedPtr currentState, SearchAction action, const std::map<SharedPtr, std::pair<SearchAction, SharedPtr>>& map)
{
    std::vector<SearchAction> path;
    SharedPtr tempState = currentState;
    
    path.push_back(action);
    
    while (tempState != nullptr)
	{
        auto it = map.find(tempState);
        if (it == map.end())
		{
            break;
        }
        
        tempState = it->second.second;
        if (tempState != nullptr)
		{
            path.push_back(it->second.first);
        }
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	// initial declarations
	std::queue<SharedPtr> open;
	std::set<SharedPtr> explored;
	std::map<SharedPtr, std::pair<SearchAction, SharedPtr>> parent_map;

	// pushing initial state into queue
	SharedPtr init = std::make_shared<SearchState>(init_state);
	open.push(init);

	// checking if initial state is final state
	if (init_state.isFinal())
	{
		return {};
	}

	// while queue is not empty
	while (!open.empty())
	{
		// getting the first element in queue
		SharedPtr currentState = open.front();
		open.pop();

		// preventing duplicates
		if (explored.find(currentState) != explored.end())
		{
			continue;
		}

		// marking current state as explored
		explored.insert(currentState);

		std::vector<SearchAction> currentStateActions = currentState->actions();
		
		for (SearchAction action : currentStateActions)
		{
			// checking memory limit
			if (memoryLimitExceeded(mem_limit_))
			{
				fprintf(stderr, "Memory limit exceeded. Aborting.\n");
				return {};
			}

			SharedPtr nextState = std::make_shared<SearchState>(action.execute(*currentState));

			// checking for final state
			if (nextState->isFinal())
			{
				return ReturnPath(currentState, action, parent_map);
			}

			// preventing duplicates and marking state
            if (explored.find(nextState) == explored.end())
			{
                open.push(nextState);
				std::pair<SearchAction, SharedPtr> newPair(action, currentState);
                parent_map.insert_or_assign(nextState, newPair);
            }
		}
	}

	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// initial declarations
	std::stack<std::pair<SharedPtr, int>> open;
	std::set<SharedPtr> explored;
	std::map<SharedPtr, std::pair<SearchAction, SharedPtr>> parent_map;
	int depth = 0;

	// pushing initial state into stack
	SharedPtr init = std::make_shared<SearchState>(init_state);
  	open.push(std::make_pair(init, depth));

	// checking if initial state is final state
	if (init_state.isFinal())
	{
		return {};
	}

	// while stack is not empty
	while (!open.empty())
	{
		// getting the first element in stack
		std::pair<SharedPtr, int> currentPair = open.top();
		SharedPtr currentState = currentPair.first;

		// getting depth from the current element
		depth = currentPair.second;

		// removing the element from the stack
		open.pop();
		explored.erase(currentState);

		// checking the depth limit
		if (depth >= depth_limit_)
		{
			// skip the iteration
			continue;
		}

		std::vector<SearchAction> currentStateActions = currentState->actions();

		for (SearchAction action : currentStateActions)
		{
			// checking memory limit
			if (memoryLimitExceeded(mem_limit_))
			{
				fprintf(stderr, "Memory limit exceeded. Aborting.\n");
				return {};
			}

			SharedPtr nextState = std::make_shared<SearchState>(action.execute(*currentState));

			// checking for final state
			if (nextState->isFinal())
			{
				return ReturnPath(currentState, action, parent_map);
			}

			// preventing duplicates and marking state
            if (explored.find(nextState) == explored.end())
			{
                open.push(std::make_pair(nextState, depth + 1));
				explored.insert(nextState);
				std::pair<SearchAction, SharedPtr> newPair(action, currentState);
                parent_map.insert_or_assign(nextState, newPair);
            }
		}
	}

	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
        std::priority_queue<StateCost, std::vector<StateCost>, std::greater<StateCost>> open_set;

		std::set<SharedPtr> explored;
		std::map<SharedPtr, std::pair<SearchAction, SharedPtr>> parent_map;

		if (init_state.isFinal())
		{
			return {};
		}
			
		SharedPtr init = std::make_shared<SearchState>(init_state);
		std::vector<SearchAction> currentStateActions = init_state.actions();
		for (SearchAction action : currentStateActions)
		{
			SearchState nextState = action.execute(init_state);

			StateCost initial = 
			{
				nextState,
				0,
				compute_heuristic(nextState, StudentHeuristic())
			};
			open_set.push(initial);
		}


		StateCost top = open_set.top();
		if (top.state.isFinal())
		{
			return {};
		}
			
		std::vector<SearchAction> currentStateActions1 = top.state.actions();
		SharedPtr stateToInsert = std::make_shared<SearchState>(init_state);
		// std::pair<SearchAction, SharedPtr> newPair(action, currentState);
		// parent_map.insert_or_assign(stateToInsert, newPair);

        return {};  // Return empty if no solution found
}