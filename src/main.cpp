#include "../include/planning_problem.h"
#include "../include/lpa_star.h"
#include <iostream>

int main() {
    PlanningProblem p;
    p.initialState = 1;
    p.goalState = 4;

    p.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {3.0, 0.0}}
    };

    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);

    std::cout << "Success: " << r.success << std::endl;
    std::cout << "Path: ";
    for (uint64_t s : r.statePath) std::cout << s << " ";
    std::cout << std::endl;
    std::cout << "Cost: " << r.totalCost << std::endl;

    return 0;
}