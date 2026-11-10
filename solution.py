"""Sokoban solver (nhieu robot): anytime Weighted A*.

Ghep tu phan cua ca nhom:
  - A: vong tim kiem A* (heapq, tie-breaker, dict g tot nhat), nang cap thanh anytime
  - B: heuristic khoang cach day + ghep thung-dich (DP bitmask)
  - C: o chet (BFS nguoc), loc successor, canonical_key, deadlock dong bang (freeze)

Chien luoc: chay Weighted A* voi w = 3, 2, 1.5, 1. Moi lan tim duoc loi giai thi giu lai
va cat cac nhanh co g + h >= loi giai tot nhat. Het gio thi tra loi giai tot nhat dang co,
chua co thi tra False. Chi dung thu vien chuan.
"""

from collections import deque
from heapq import heappop, heappush
from itertools import count
from time import perf_counter

from sokoban import sokoban_goal_state

INF = 10 ** 9
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))      # (dx, dy): len, phai, xuong, trai
SAFETY = 0.85                                  # dung toi da 85% timebound
WEIGHTS = (3, 2, 1.5, 1)                       # w giam dan
FALLBACK_WEIGHTS = (6, 12, 40)                 # neu w=3 can qua nhieu bo nho ma chua co loi giai
MAX_STATES = 1500000                           # tran so trang thai moi lan chay (tranh tran RAM)
MAX_MASK_STORAGES = 12                         # nhieu dich hon thi dung can long hon, khong dung bitmask
STALL_FRAC = 0.35                               # từng này thời gian mà ko cải thiện thì dừng
SOFT_FRAC = 0.7                                # giải được thì tổng tgian tối đa


# ----------------------------------------------------------------------------
# Phan cua C: o chet
# ----------------------------------------------------------------------------
def _is_free(state, cell):
    x, y = cell
    return 0 <= x < state.width and 0 <= y < state.height and cell not in state.obstacles


def _push_distances_from(state, sources):
    """BFS nguoc tu cac o 'sources': so lan day it nhat de dua thung tu moi o toi do."""
    dist = {}
    queue = deque()
    for s in sources:
        dist[s] = 0
        queue.append(s)
    while queue:
        p = queue.popleft()
        px, py = p
        for dx, dy in DIRS:
            prev = (px - dx, py - dy)              # o thung dung truoc cu day
            robot = (px - 2 * dx, py - 2 * dy)     # o robot phai dung de day
            if prev not in dist and _is_free(state, prev) and _is_free(state, robot):
                dist[prev] = dist[p] + 1
                queue.append(prev)
    return dist


def _dead_squares(state, any_dist):
    """O di vao duoc nhung thung dat vao la khong bao gio toi duoc dich nao."""
    return frozenset((x, y)
                     for x in range(state.width) for y in range(state.height)
                     if _is_free(state, (x, y)) and (x, y) not in any_dist)


# ----------------------------------------------------------------------------
# Phan cua C: deadlock dong bang (freeze), gom ca khoi 2x2 va thung dung sat tuong
# ----------------------------------------------------------------------------
_AXES = (((-1, 0), (1, 0)), ((0, -1), (0, 1)))   # truc ngang, truc doc


def _is_wall(state, cell):
    x, y = cell
    return not (0 <= x < state.width and 0 <= y < state.height) or cell in state.obstacles


def _blocked(state, boxes, dead, cell, axis, visiting):
    x, y = cell
    both_dead = True
    for dx, dy in _AXES[axis]:
        n = (x + dx, y + dy)
        if _is_wall(state, n):
            return True
        if n in boxes and (n in visiting or _frozen(state, boxes, dead, n, visiting)):
            return True
        if n not in dead:
            both_dead = False
    return both_dead


def _frozen(state, boxes, dead, cell, visiting):
    visiting = visiting | {cell}
    return (_blocked(state, boxes, dead, cell, 0, visiting)
            and _blocked(state, boxes, dead, cell, 1, visiting))


def _freeze_deadlock(state, dead):
    """True neu co thung dong bang nam ngoai o dich (robot khong tinh la vat chan)."""
    boxes, storage = state.boxes, state.storage
    for b in boxes:
        if b not in storage and _frozen(state, boxes, dead, b, frozenset()):
            return True
    return False


# ----------------------------------------------------------------------------
# Phan cua B: heuristic ghep thung - dich
# ----------------------------------------------------------------------------
def _build_heuristic(state):
    storages = sorted(state.storage)
    m = len(storages)
    maps = [_push_distances_from(state, [s]) for s in storages]
    cache = {}

    def h(boxes):
        v = cache.get(boxes)
        if v is not None:
            return v
        if len(boxes) > m:
            cache[boxes] = INF
            return INF
        rows = [[mp.get(b, INF) for mp in maps] for b in boxes]
        if any(min(r) >= INF for r in rows):
            v = INF
        elif m > MAX_MASK_STORAGES:
            v = sum(min(r) for r in rows)          # can long hon nhung van la can duoi
        else:
            dp = {0: 0}
            for row in rows:
                nxt = {}
                for mask, c in dp.items():
                    for j in range(m):
                        if mask >> j & 1 or row[j] >= INF:
                            continue
                        nm, nc = mask | (1 << j), c + row[j]
                        if nc < nxt.get(nm, INF):
                            nxt[nm] = nc
                dp = nxt
                if not dp:
                    break
            v = min(dp.values()) if dp else INF
        cache[boxes] = v
        return v

    return h


# ----------------------------------------------------------------------------
# Phan cua A + C: tim kiem
# ----------------------------------------------------------------------------
def _key(state):
    """Khoa de nhan biet trang thai da tham: robot giong nhau nen sap xep (chi dung lam khoa)."""
    return (tuple(sorted(state.robots)), state.boxes)


class _Result:
    FOUND, EXHAUSTED, TIMEOUT, CAP = "found", "exhausted", "timeout", "cap"


def _weighted_astar(start, w, h, dead, deadline, best_goal, stall=None, cap=None):
    """Mot luot Weighted A*. Tra (trang thai ket thuc, loi giai tot nhat, tinh trang)."""
    best_g = best_goal.gval if best_goal is not None else INF
    h0 = h(start.boxes)
    if h0 >= INF or start.gval + h0 >= best_g:
        return _Result.EXHAUSTED, best_goal
    tie = count()
    frontier = [(start.gval + w * h0, h0, next(tie), start)]
    seen_g = {_key(start): start.gval}
    expanded = 0
    while frontier:
        expanded += 1
        if expanded & 63 == 0 and perf_counter() >= deadline:
            return _Result.TIMEOUT, best_goal
        _, hv, _, cur = heappop(frontier)
        if cur.gval != seen_g.get(_key(cur)):
            continue                                   # ban cu, da co duong tot hon
        if cur.gval + hv >= best_g:
            if w == 1:
                return _Result.EXHAUSTED, best_goal    # w=1: moi thu con lai deu >= best
            continue
        for s in cur.successors():
            g = s.gval
            moved = s.boxes - cur.boxes
            if moved:
                if any(b in dead for b in moved):
                    continue                           # day thung vao o chet
                if _freeze_deadlock(s, dead):
                    continue                           # thung dong bang ngoai dich
            k = _key(s)
            if g >= seen_g.get(k, INF):
                continue
            hs = h(s.boxes)
            if hs >= INF or g + hs >= best_g:
                continue
            seen_g[k] = g
            if sokoban_goal_state(s):
                best_goal, best_g = s, g
                if stall is not None:                  # cải thiện gia hạn thêm 1 đoạn stall
                    deadline = min(cap, perf_counter() + stall)
                if w > 1:
                    return _Result.FOUND, best_goal    # lay luon, luot sau cai thien
                continue
            heappush(frontier, (g + w * hs, hs, next(tie), s))
        if len(seen_g) > MAX_STATES:
            return _Result.CAP, best_goal
    return _Result.EXHAUSTED, best_goal


def solve(initial_state, timebound=120):
    """Tra ve trang thai dich (co chuoi parent hop le) hoac dung False."""
    best = None
    try:
        if timebound is None or timebound <= 0:
            return False
        start_time = perf_counter()
        deadline = start_time + timebound * SAFETY
        if sokoban_goal_state(initial_state):
            return initial_state

        dist_any = _push_distances_from(initial_state, list(initial_state.storage))
        dead = _dead_squares(initial_state, dist_any)
        h = _build_heuristic(initial_state)
        if h(initial_state.boxes) >= INF:
            return False                               # chung minh duoc la vo nghiem

        stall = timebound * STALL_FRAC
        soft_end = start_time + timebound * SOFT_FRAC

        w = WEIGHTS[0]
        while perf_counter() < deadline:
            if best is None:
                eff, cap = deadline, None              # chưa có lời giải: dùng hết 85% như cũ
                st = None
            else:
                cap = min(deadline, soft_end)
                eff = min(cap, perf_counter() + stall)
                st = stall
                if eff <= perf_counter():
                    break
            status, best = _weighted_astar(initial_state, w, h, dead, eff, best, st, cap)

            if status == _Result.TIMEOUT:
                break
            if status == _Result.EXHAUSTED:
                # Moi cach cat nhanh deu an toan => het nhanh nghia la khong con loi giai
                # tot hon (hoac vo nghiem neu chua tim duoc loi giai nao).
                return best if best is not None else False
            if status == _Result.FOUND:
                lower = [x for x in WEIGHTS if x < w]
                if not lower:
                    break
                w = lower[0]                           # co loi giai: thu w nho hon de cai thien
            else:                                      # CAP: het bo nho
                if best is not None:
                    break                              # da co loi giai: giu lai
                greedier = [x for x in FALLBACK_WEIGHTS if x > w]
                if not greedier:
                    break
                w = greedier[0]                        # chua co: thu tham lam hon
        return best if best is not None else False
    except Exception:
        return best if best is not None else False