"""Optimized A* solver for the supplied Sokoban framework."""

from collections import deque
from heapq import heappop, heappush
from itertools import count
from time import perf_counter

from sokoban import sokoban_goal_state


INF = 10 ** 9      # "vô cực" cho heuristic / g
BIG = 10 ** 6      # chi phí cho cặp (box, storage) không thể tới được
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))


# --------------------------------------------------------------------------
# Tiền xử lý (chỉ chạy 1 lần cho mỗi bàn chơi)
# --------------------------------------------------------------------------
def _push_distances_to_storage(state, storage):
    """BFS ngược (pull) từ 1 storage: số lần đẩy tối thiểu từ mỗi ô tới storage."""
    width, height, obstacles = state.width, state.height, state.obstacles

    def free(cx, cy):
        return 0 <= cx < width and 0 <= cy < height and (cx, cy) not in obstacles

    distances = {storage: 0}
    queue = deque([storage])
    while queue:
        current = queue.popleft()
        x, y = current
        d = distances[current] + 1
        for dx, dy in DIRS:
            px, py = x - dx, y - dy          # ô box đứng trước khi bị đẩy
            if (px, py) in distances:
                continue
            if free(px, py) and free(x - 2 * dx, y - 2 * dy):
                distances[(px, py)] = d
                queue.append((px, py))
    return distances


def _precompute(state):
    """Trả về (bảng chi phí theo ô, tập ô chết)."""
    storages = tuple(state.storage)
    maps = [_push_distances_to_storage(state, s) for s in storages]

    # cost_rows[cell] = tuple khoảng cách tới từng storage (BIG nếu không tới được)
    cost_rows = {}
    reachable = set()
    for m in maps:
        reachable.update(m)
    for cell in reachable:
        cost_rows[cell] = tuple(m.get(cell, BIG) for m in maps)

    dead_squares = frozenset(
        (x, y)
        for x in range(state.width)
        for y in range(state.height)
        if (x, y) not in state.obstacles and (x, y) not in reachable
    )
    return cost_rows, len(storages), dead_squares


# --------------------------------------------------------------------------
# Heuristic: matching tối thiểu box <-> storage (Hungarian, O(n^2 * m))
# --------------------------------------------------------------------------
def _min_assignment(rows, m):
    """Chi phí gán tối thiểu, mỗi hàng (box) một cột (storage) khác nhau. n <= m."""
    n = len(rows)
    u = [0] * (n + 1)
    v = [0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            row = rows[i0 - 1]
            ui0 = u[i0]
            delta = INF
            j1 = 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = row[j - 1] - ui0 - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    return -v[0]


def build_heuristic(cost_rows, num_storages):
    """Heuristic có cache theo tập box. Trả INF nếu có box ở ô chết."""
    cache = {}
    dead_row = (BIG,) * num_storages

    def heuristic(current):
        boxes = current.boxes
        cached = cache.get(boxes)
        if cached is not None:
            return cached

        rows = [cost_rows.get(b, dead_row) for b in boxes]
        if any(min(r) >= BIG for r in rows):
            result = INF
        elif len(rows) == 1:
            result = min(rows[0])
        else:
            total = _min_assignment(rows, num_storages)
            result = INF if total >= BIG else total

        cache[boxes] = result
        return result

    return heuristic


# --------------------------------------------------------------------------
# Cắt tỉa deadlock 2x2 (rẻ, chỉ kiểm tra quanh box vừa đẩy)
# --------------------------------------------------------------------------
def _make_block_check(state):
    width, height, obstacles, storage = (
        state.width, state.height, state.obstacles, state.storage)

    def freeze_2x2(box, boxes):
        x, y = box
        for ox in (-1, 0):
            for oy in (-1, 0):
                cells = ((x + ox, y + oy), (x + ox + 1, y + oy),
                         (x + ox, y + oy + 1), (x + ox + 1, y + oy + 1))
                all_blocked = True
                all_on_goal = True
                for c in cells:
                    cx, cy = c
                    if (not (0 <= cx < width and 0 <= cy < height)
                            or c in obstacles):
                        continue                      # tường
                    if c in boxes:
                        if c not in storage:
                            all_on_goal = False
                        continue
                    all_blocked = False
                    break
                if all_blocked and not all_on_goal:
                    return True
        return False

    return freeze_2x2


def _search_key(state):
    """Các robot hoán đổi được cho nhau."""
    return (tuple(sorted(state.robots)), state.boxes)


# --------------------------------------------------------------------------
# A*
# --------------------------------------------------------------------------
def solve(initial_state, timebound=120):
    """Trả về goal state tìm được bằng A*, hoặc False nếu hết thời gian / vô nghiệm."""
    if timebound <= 0:
        return False

    deadline = perf_counter() + timebound
    if sokoban_goal_state(initial_state):
        return initial_state

    cost_rows, num_storages, dead_squares = _precompute(initial_state)
    heuristic = build_heuristic(cost_rows, num_storages)
    freeze_2x2 = _make_block_check(initial_state)
    storage = initial_state.storage

    initial_h = heuristic(initial_state)
    if initial_h >= INF:
        return False

    tie = count()
    frontier = [(initial_state.gval + initial_h, initial_h,
                 next(tie), initial_state)]
    best_g = {_search_key(initial_state): initial_state.gval}

    iterations = 0
    while frontier:
        iterations += 1
        # perf_counter() khá tốn -> chỉ kiểm tra mỗi 64 lần mở rộng
        if not iterations & 63 and perf_counter() >= deadline:
            return False

        _, _, _, state = heappop(frontier)
        if state.gval != best_g.get(_search_key(state)):
            continue                                  # entry cũ (stale)
        if sokoban_goal_state(state):
            return state

        parent_boxes = state.boxes
        for successor in state.successors():
            boxes = successor.boxes

            # box vừa bị đẩy (nếu có)
            moved = boxes - parent_boxes
            bad = False
            for box in moved:
                if (box in dead_squares and box not in storage) \
                        or freeze_2x2(box, boxes):
                    bad = True
                    break
            if bad:
                continue

            key = _search_key(successor)
            g = successor.gval
            if g >= best_g.get(key, INF):
                continue

            h = heuristic(successor)
            if h >= INF:
                continue

            best_g[key] = g
            heappush(frontier, (g + h, h, next(tie), successor))

    return False