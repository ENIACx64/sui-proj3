/**
 * @file sui-solution.cc
 * @author Jindřich Vodák (xvodak06)
 * 
 */

#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

#define MEMORY_LIMIT 50000

typedef std::shared_ptr<SearchState> SharedPtr;

bool memoryLimitExceeded(size_t limit)
{
	return (getCurrentRSS() > limit - MEMORY_LIMIT);
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
				// reconstructing the path
                std::vector<SearchAction> path;
                SharedPtr tempState = currentState;
                SearchAction tempAction = action;

                // backtracking from final state to initial state
                while (tempState != nullptr)
				{
                    path.push_back(tempAction);
                    auto it = parent_map.find(tempState);

                    if (it != parent_map.end())
					{
                        tempAction = it->second.first;
                        tempState = it->second.second;
                    }
					else
					{
                        break;
                    }
                }

                // reversing the path to get the correct order
                std::reverse(path.begin(), path.end());
                return path;
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
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}