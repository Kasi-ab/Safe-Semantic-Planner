# Safe Semantic Planner

A C++ implementation of LPA* (Lifelong Planning A*) for finding safe paths through a finite Cartesian state space, avoiding bad states while balancing cost against distance from danger. Supports efficient incremental replanning when the environment changes.

## Requirements

- C++17 or later
- g++ (MinGW-w64 on Windows, or any standard g++/clang on Linux/macOS)

## Project Structure

```
include/    Header files (class declarations)
  state.h            State class
  transition.h       Transition class
  planning_problem.h PlanningProblem class
  planning_result.h  PlanningResult class
  graph.h            Graph module
  safety.h           Safety module
  heuristic.h        Heuristic module
  planner.h          Abstract Planner interface
  lpa_star.h         LPAStarPlanner class

src/        Source files (implementations)
  graph.cpp
  safety.cpp
  heuristic.cpp
  lpa_star.cpp

tests/      Test suite covering all 6 assignment test cases
  run_tests.cpp

build/      Compiled output (not tracked in git)
```

## Architecture

How the modules depend on each other, from the base data types up through the planner core:

```
+------------------------------------+
|         Core data types            |
| State, Transition, Problem, Result |
+----------------+-------------------+
                  |
      +-----------+------------+
      |           |            |
+-----v----+ +----v-----+ +----v------+
|  Graph   | |  Safety  | | Heuristic |
| module   | | module   | | module    |
+-----+----+ +----+-----+ +----+------+
      |           |            |
      +-----------+------------+
                  |
         +--------v-----------+
         |   LPA* planner     |
         | g / rhs cost search|
         +--------+-----------+
                  |
         +--------v-----------+
         |    Replanning      |
         | Incremental updates|
         +--------+-----------+
                  |
         +--------v----------+
         | Test suite (6)    |
         +-------------------+
```

Core data types sit at the base with no dependencies. The graph, safety, and heuristic modules each build on those types independently, and don't depend on each other. The LPA* planner draws on all three to run the search. Replanning sits on top of the planner, reusing its internal `g`/`rhs` state instead of rebuilding it. The test suite exercises the planner and replanning layer together across all six assignment scenarios.

## Working

How a call to `plan()` actually runs, start to finish:

1. **Build lookups.** The planner takes a `PlanningProblem` and builds a state-by-id map and a reverse (incoming) transition map, so it can answer "who leads into state X" quickly during search.
2. **Initialize.** Every state's cost estimate (`g`) and lookahead estimate (`rhs`) start at infinity, except the start state, whose `rhs` is set to 0. The start state goes into a priority queue.
3. **Search loop.** The planner repeatedly pulls the lowest-priority state from the queue. If its cost just improved, it propagates that improvement to every state it leads to. If its cost just got worse (only relevant during replanning), it resets and lets neighbors recompute. This continues until the goal's cost is settled.
4. **Safety weighting.** Every edge's cost used during search isn't just its raw `cost` field, it's `cost + 1/distance_to_nearest_bad_state`, so paths that pass close to danger become expensive even when technically legal, and truly bad states are excluded outright.
5. **Path reconstruction.** Once the goal is settled, the planner walks backward from goal to start, at each step choosing whichever predecessor gave the cheapest cost, skipping any bad states.
6. **Result.** Total cost and minimum safety distance across the path are computed and returned in a `PlanningResult`, along with the full state and transition path.

For a change in the environment (a transition disabled or added, the goal changed, bad states updated), the planner does **not** repeat steps 1-2. It only re-triggers step 3's search loop starting from whichever states the change directly affects, letting the update ripple outward only as far as it needs to. That's the entire mechanism behind fast replanning.

## Building

From the project root:

```
g++ -Iinclude tests/run_tests.cpp src/graph.cpp src/safety.cpp src/heuristic.cpp src/lpa_star.cpp -o build/tests.exe
```

## Running

```
./build/tests.exe
```

This runs all six test cases in sequence and prints, for each: success status, resulting path, total cost, safety score, states explored, and planning time. Dynamic test cases (4, 5, 6) additionally print replanning time after applying a change.

## What each test case demonstrates

| Test | Scenario | What it proves |
|---|---|---|
| 1 | Basic reachability | Finds the unique valid path |
| 2 | Bad state avoidance | Never routes through a bad state, even if that path exists |
| 3 | Safety margin | Picks a costlier path when it stays farther from danger |
| 4 | Transition removed | Replans around a disabled edge |
| 5 | Goal changed | Replans for a new goal without a full rebuild |
| 6 | Transition added | Discovers a new shortcut once available |

## Core Design

**Algorithm**: LPA*, chosen for efficient replanning, see the Working section above for the full mechanism.

**Safety**: Each state's distance to the nearest bad state is computed and folded into edge costs as a penalty (`1 / distance`), so the search naturally prefers paths that stay farther from danger, without treating safety as a hard cutoff beyond the bad-state exclusion itself.

**Heuristic**: Euclidean distance between a state's embedding and the goal's embedding, admissible as long as no edge costs less than the straight-line distance between its endpoints.

## Extending

- `LPAStarPlanner::onTransitionUnavailable()`, `onTransitionAdded()`, `onBadStatesChanged()`, `onGoalChanged()` are the four entry points for reacting to a changing environment. Call these instead of `plan()` again to reuse prior search work.
- `LPAStarPlanner::statesExpanded` tracks how many states the most recent `plan()` or replanning call actually explored.
