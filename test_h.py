from sokoban import PROBLEMS, sokoban_goal_state
from collections import deque
from heuristic import build_heuristic

def bfs_cost(s):
    seen, q = {s.hashable_state()}, deque([s])
    while q:
        c = q.popleft()
        if sokoban_goal_state(c): return c.gval
        for n in c.successors():
            k = n.hashable_state()
            if k not in seen:
                seen.add(k); q.append(n)

for i in [1, 3, 4, 6]:
    s = PROBLEMS[i]
    print(i, "h =", build_heuristic(s)(s), "optimal =", bfs_cost(s))