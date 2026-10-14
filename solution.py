"""Student entry point. See README.md for the API and scoring rules."""

from collections import deque
from time import perf_counter

from sokoban import sokoban_goal_state


def solve(initial_state, timebound=120):
    """Return a goal state found by breadth-first search, or ``False``.

    Every action in the supplied Sokoban framework costs one step, so the
    first goal removed from the queue has a shortest valid parent chain.
    """
    if timebound <= 0:
        return False

    deadline = perf_counter() + timebound
    if sokoban_goal_state(initial_state):
        return initial_state

    frontier = deque([initial_state])
    visited = {initial_state.hashable_state()}

    while frontier and perf_counter() < deadline:
        state = frontier.popleft()

        for successor in state.successors():
            if perf_counter() >= deadline:
                return False

            state_key = successor.hashable_state()
            if state_key in visited:
                continue
            visited.add(state_key)

            if sokoban_goal_state(successor):
                return successor
            frontier.append(successor)

    return False
