# Safe Semantic Planner

A C++ implementation of LPA* (Lifelong Planning A*) for finding safe paths through a finite Cartesian state space, avoiding bad states while balancing cost against distance from danger. Supports efficient incremental replanning when the environment changes.

## Requirements

- C++17 or later
- g++ (MinGW-w64 on Windows, or any standard g++/clang on Linux/macOS)

## Project Structure

```
include/    Header files (class declarations)
src/        Source files (implementations)
tests/      Test suite covering all 6 assignment test cases
build/      Compiled output (not tracked in git)
```

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

**Algorithm**: LPA*, chosen for efficient replanning: `g` and `rhs` cost values are tracked from the start state outward, so most replanning operations only touch the states actually affected by a change, not the whole graph.

**Safety**: Each state's distance to the nearest bad state is computed and folded into edge costs as a penalty (`1 / distance`), so the search naturally prefers paths that stay farther from danger, without treating safety as a hard cutoff beyond the bad-state exclusion itself.

**Heuristic**: Euclidean distance between a state's embedding and the goal's embedding, admissible as long as no edge costs less than the straight-line distance between its endpoints.

## Extending

- `LPAStarPlanner::onTransitionUnavailable()`, `onTransitionAdded()`, `onBadStatesChanged()`, `onGoalChanged()` are the four entry points for reacting to a changing environment. Call these instead of `plan()` again to reuse prior search work.
- `LPAStarPlanner::statesExpanded` tracks how many states the most recent `plan()` or replanning call actually explored.