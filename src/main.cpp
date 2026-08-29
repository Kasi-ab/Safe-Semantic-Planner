#include "../include/planning_problem.h"
#include "../include/graph.h"
#include "../include/safety.h"
#include "../include/heuristic.h"
#include <iostream>

int main() {
    State s; s.id = 1; s.embedding = {0.0, 0.0};
    State goal; goal.id = 2; goal.embedding = {3.0, 4.0};

    double h = heuristic(s, goal);
    std::cout << "Heuristic: " << h << std::endl; // should print 5

    return 0;
}