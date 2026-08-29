#include "../include/planning_problem.h"
#include "../include/graph.h"
#include "../include/safety.h"
#include <iostream>

int main() {
    State a; a.id = 1; a.embedding = {0.0, 0.0};
    State bad; bad.id = 2; bad.embedding = {3.0, 4.0};

    std::vector<State> allStates = {a, bad};
    std::vector<uint64_t> badIds = {2};

    double d = distanceToNearestBadState(a, badIds, allStates);
    std::cout << "Distance: " << d << std::endl; // should print 5

    return 0;
}