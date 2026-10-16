"""A* solver for the supplied Sokoban framework."""

from collections import deque
from heapq import heappop, heappush
from itertools import count
from time import perf_counter

from sokoban import sokoban_goal_state


INF = 10 ** 9
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))


def _push_distances(state):
    """Return the minimum number of pushes from every cell to any storage."""
    distances = {}
    for storage in state.storage:
        for cell, distance in _push_distances_to_storage(state, storage).items():
            distances[cell] = min(distances.get(cell, INF), distance)
    return distances


def _push_distances_to_storage(state, storage):
    """Return reverse-push distances to one storage cell."""
    distances = {}
    queue = deque()

    distances[storage] = 0
    queue.append(storage)

    while queue:
        current = queue.popleft()
        x, y = current
        for dx, dy in DIRS:
            previous = (x - dx, y - dy)
            robot = (x - 2 * dx, y - 2 * dy)

            def free(cell):
                cx, cy = cell
                return (0 <= cx < state.width and 0 <= cy < state.height
                        and cell not in state.obstacles)

            if free(previous) and free(robot) and previous not in distances:
                distances[previous] = distances[current] + 1
                queue.append(previous)

    return distances


def build_heuristic(state):
    """Build B's push-distance matching heuristic for a fixed board."""
    storages = tuple(state.storage)
    distance_maps = [
        _push_distances_to_storage(state, storage) for storage in storages
    ]
    cache = {}

    def heuristic(current):
        boxes = current.boxes
        if boxes in cache:
            return cache[boxes]

        # dp[mask] is the cheapest assignment for the boxes seen so far.
        dp = {0: 0}
        for box in boxes:
            next_dp = {}
            for mask, cost in dp.items():
                for index, distances in enumerate(distance_maps):
                    if mask & (1 << index):
                        continue
                    distance = distances.get(box, INF)
                    if distance == INF:
                        continue
                    new_mask = mask | (1 << index)
                    next_dp[new_mask] = min(
                        next_dp.get(new_mask, INF), cost + distance
                    )
            dp = next_dp

        result = min(dp.values(), default=INF)
        cache[boxes] = result
        return result

    return heuristic


def _dead_squares(state, push_distances):
    return frozenset(
        (x, y)
        for x in range(state.width)
        for y in range(state.height)
        if (x, y) not in state.obstacles
        and (x, y) not in push_distances
    )


def _search_key(state):
    """Robots are interchangeable for reachability and box placement."""
    return (tuple(sorted(state.robots)), state.boxes)


def solve(initial_state, timebound=120):
    """Return a goal state found by A*, or ``False`` if the budget expires."""
    if timebound <= 0:
        return False

    deadline = perf_counter() + timebound
    if sokoban_goal_state(initial_state):
        return initial_state

    push_distances = _push_distances(initial_state)
    dead_squares = _dead_squares(initial_state, push_distances)
    heuristic = build_heuristic(initial_state)
    initial_h = heuristic(initial_state)
    if initial_h >= INF:
        return False

    # The counter makes heap entries deterministic when f and h tie.
    tie_breaker = count()
    frontier = [(initial_state.gval + initial_h, initial_h,
                 next(tie_breaker), initial_state)]
    best_g = {_search_key(initial_state): initial_state.gval}

    while frontier and perf_counter() < deadline:
        _, _, _, state = heappop(frontier)
        state_key = _search_key(state)
        if state.gval != best_g.get(state_key):
            continue
        if sokoban_goal_state(state):
            return state

        for successor in state.successors():
            if perf_counter() >= deadline:
                return False

            moved_boxes = successor.boxes - state.boxes
            if any(box in dead_squares and box not in successor.storage
                   for box in moved_boxes):
                continue

            successor_key = _search_key(successor)
            if successor.gval >= best_g.get(successor_key, INF):
                continue

            successor_h = heuristic(successor)
            if successor_h >= INF:
                continue
            best_g[successor_key] = successor.gval
            heappush(
                frontier,
                (successor.gval + successor_h, successor_h,
                 next(tie_breaker), successor),
            )

    return False
