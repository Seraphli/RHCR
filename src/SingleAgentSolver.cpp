#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location)
{
    // Compute heuristic when loc are not in G.heuristics
    int loc = goal_location[goal_id].first;
    G.ensure_heuristic(loc);
    double h = G.heuristics.at(goal_location[goal_id].first)[curr];
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        h += G.heuristics.at(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
        goal_id++;
    }
    return h;
}
