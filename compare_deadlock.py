"""Kiểm tra deadlock.py: (1) map đề bài 0..7, (2) hàng nghìn map ngẫu nhiên.

Mỗi map chạy BFS 2 cách: "ô chết + canonical" (tuần 2) và "+ freeze" (tuần 3).
Phải cho CÙNG kết quả (cùng độ dài, hoặc cùng vô nghiệm). Số nút phải <= .

Chạy:  python compare_deadlock.py            (map 0..7, rồi 500 map ngẫu nhiên)
       python compare_deadlock.py 10000 5    (10000 map ngẫu nhiên, seed 5)
"""
import random
import sys
from collections import deque
from sokoban import PROBLEMS, SokobanState, sokoban_goal_state
from dead_squares import compute_dead_squares
from pruning import canonical_key
from deadlock import filter_successors2
from pruning import filter_successors

MAX_SEEN = 20000


def bfs(state, use_freeze, max_seen=MAX_SEEN):
    if sokoban_goal_state(state):
        return 0, 0, "ok"
    dead = compute_dead_squares(state)
    flt = filter_successors2 if use_freeze else filter_successors
    seen = {canonical_key(state)}
    queue = deque([state])
    expanded = 0
    while queue:
        cur = queue.popleft()
        expanded += 1
        for s in flt(cur, dead):
            k = canonical_key(s)
            if k in seen:
                continue
            if sokoban_goal_state(s):
                return s.gval, expanded, "ok"
            seen.add(k)
            queue.append(s)
            if len(seen) > max_seen:
                return None, expanded, "quá lớn"
    return None, expanded, "vô nghiệm"


def random_state(rng):
    W, H = rng.randint(4, 6), rng.randint(4, 6)
    cells = [(x, y) for x in range(W) for y in range(H)]
    rng.shuffle(cells)
    nwall = rng.randint(0, 4)
    walls = frozenset(cells[:nwall])
    free = cells[nwall:]
    nr, nb = rng.randint(1, 2), rng.randint(2, 4)
    if len(free) < nr + nb + 1:
        return None
    robots = tuple(free[:nr])
    boxes = frozenset(free[nr:nr + nb])
    ns = rng.randint(nb, min(len(free), nb + 2))
    storage = frozenset(rng.sample(free, ns))
    return SokobanState("START", 0, None, W, H, robots, boxes, storage, walls)


def describe(s):
    return "W=%d H=%d robots=%r boxes=%r storage=%r obstacles=%r" % (
        s.width, s.height, s.robots, sorted(s.boxes), sorted(s.storage), sorted(s.obstacles))


def fmt(r):
    return "%-8s" % r[2] if r[0] is None else "%3d bước, %6d nút" % (r[0], r[1])


def same(a, b):
    if a[2] == "quá lớn" or b[2] == "quá lớn":
        return True          # không kết luận được, bỏ qua
    return a[0] == b[0] and a[2] == b[2]


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    print("PHẦN 1: map đề bài")
    print("map | tuần 2 (ô chết+canonical) | tuần 3 (+freeze)     | kiểm tra")
    wrong = 0
    for i in range(8):
        a, b = bfs(PROBLEMS[i], False, 300000), bfs(PROBLEMS[i], True, 300000)
        ok = "OK" if same(a, b) and b[1] <= a[1] else "SAI"
        wrong += ok != "OK"
        print("%3d | %s | %s | %s" % (i, fmt(a), fmt(b), ok))
    print("PHẦN 2: %d map ngẫu nhiên (seed %d)" % (n, seed))
    rng = random.Random(seed)
    tested = solved = unsolv = 0
    nodes_a = nodes_b = 0
    for _ in range(n):
        s = random_state(rng)
        if s is None:
            continue
        a, b = bfs(s, False), bfs(s, True)
        if a[2] == "quá lớn" or b[2] == "quá lớn":
            continue
        tested += 1
        solved += a[2] == "ok"
        unsolv += a[2] == "vô nghiệm"
        nodes_a += a[1]
        nodes_b += b[1]
        if not same(a, b):
            wrong += 1
            print("SAI:", describe(s), "| tuần 2:", a[:1] + a[2:], "| freeze:", b[:1] + b[2:])
    print("Đã thử %d map (giải được %d, vô nghiệm %d)" % (tested, solved, unsolv))
    print("Tổng nút: tuần 2 = %d, tuần 3 = %d" % (nodes_a, nodes_b))
    print("Số map bị sai:", wrong)
