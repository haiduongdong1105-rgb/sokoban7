"""Đếm node của A* tham chiếu trên map khó.  python analyze_timeouts.py --maps 15-19 -b 60"""
import argparse, heapq, time
from collections import deque
from sokoban import PROBLEMS, sokoban_goal_state

D = ((0, -1), (1, 0), (0, 1), (-1, 0))


def push_dist(s):
    """Số lần đẩy tối thiểu tới đích (kéo ngược). Ô vắng mặt = ô chết."""
    free = {(x, y) for x in range(s.width) for y in range(s.height)} - s.obstacles
    d = {g: 0 for g in s.storage}
    q = deque(d)
    while q:
        c = q.popleft()
        for dx, dy in D:
            b, r = (c[0] + dx, c[1] + dy), (c[0] + 2 * dx, c[1] + 2 * dy)
            if b in free and r in free and b not in d:
                d[b] = d[c] + 1
                q.append(b)
    return d, len(free)


def analyze(i, budget):
    s0 = PROBLEMS[i]
    pd, nfree = push_dist(s0)
    h = lambda s: sum(pd[b] for b in s.boxes)
    tie, openq = 0, [(h(s0), 0, 0, s0)]
    seen, canon = {s0.hashable_state(): 0}, set()
    exp = gen = dup = walk = 0
    best, t0, solved = h(s0), time.perf_counter(), None
    while openq:
        _, g, _, s = heapq.heappop(openq)
        if seen[s.hashable_state()] < g:
            continue
        exp += 1
        if sokoban_goal_state(s):
            solved = s
            break
        if exp % 256 == 0 and time.perf_counter() - t0 > budget:
            break
        for n in s.successors():
            gen += 1
            walk += n.boxes == s.boxes
            if any(b not in pd for b in n.boxes):
                continue
            k = n.hashable_state()
            canon.add((tuple(sorted(n.robots)), n.boxes))
            if seen.get(k, 1e9) <= n.gval:
                dup += 1
                continue
            seen[k] = n.gval
            best = min(best, h(n))
            tie += 1
            heapq.heappush(openq, (n.gval + h(n), n.gval, tie, n))
    el = time.perf_counter() - t0
    print("MAP %d %dx%d robots=%d boxes=%d ô chết=%d" % (
        i, s0.width, s0.height, len(s0.robots), len(s0.boxes), nfree - len(pd)))
    print("  %s | mở rộng %d (%.0f node/s) | nhánh≈%.1f | trùng %.0f%%" % (
        "GIẢI %d bước" % solved.gval if solved else "HẾT %.0fs" % budget,
        exp, exp / el, gen / max(exp, 1), 100 * dup / max(gen, 1)))
    print("  đi bộ %.0f%% | raw/canonical = %.2fx | h: %d -> %d" % (
        100 * walk / max(gen, 1), len(seen) / max(len(canon), 1), h(s0), best))


ap = argparse.ArgumentParser()
ap.add_argument("--maps", default="15-19")
ap.add_argument("-b", type=float, default=30)
a = ap.parse_args()
for p in a.maps.split(","):
    for m in range(int(p.split("-")[0]), int(p.split("-")[-1]) + 1):
        analyze(m, a.b)
