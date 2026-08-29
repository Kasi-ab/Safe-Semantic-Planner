#ifndef LPA_STAR_H
#define LPA_STAR_H

#include <unordered_map>
#include <vector>
#include <utility>
#include "planner.h"
#include "graph.h"

class LPAStarPlanner : public Planner {
public:
    PlanningResult plan(const PlanningProblem& problem) override;

private:
    struct QueueEntry {
        double k1, k2;
        uint64_t stateId;
    };

    const PlanningProblem* problemPtr;
    Graph* graphPtr;
    std::unordered_map<uint64_t, State> stateLookup;
    std::unordered_map<uint64_t, std::vector<Transition>> incoming;
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;
    std::vector<QueueEntry> queue;

    void initialize();
    std::pair<double,double> calculateKey(uint64_t stateId);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    bool isBadState(uint64_t stateId);
    double effectiveCost(const Transition& t);
    void pushQueue(uint64_t stateId);
    void removeFromQueue(uint64_t stateId);
};

#endif