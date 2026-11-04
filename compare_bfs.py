"""So sánh 3 cách chạy BFS: thường / + lọc ô chết / + lọc ô chết + canonical_key.

Mục đích: chứng minh hai cải tiến của C làm GIẢM số trạng thái phải xét
mà KHÔNG làm đổi độ dài lời giải (lời giải ngắn nhất vẫn như cũ).

Chạy:   python compare_bfs.py            (các map nhỏ 0..7)
        python compare_bfs.py 1 3 4      (chỉ các map đã chọn)
"""
import sys
from collections import deque
from sokoban import PROBLEMS, sokoban_goal_state
from dead_squares import compute_dead_squares
from pruning import creates_dead_box, canonical_key

MAX_SEEN = 300000


def bfs(state, use_dead=False, use_canon=False):
    """Trả về (độ dài lời giải hoặc None, số trạng thái đã mở rộng, ghi chú)."""
    if sokoban_goal_state(state):
        return 0, 0, "ok"
    dead = compute_dead_squares(state) if use_dead else frozenset()
    key = canonical_key if use_canon else (lambda s: s.hashable_state())
    seen = {key(state)}
    queue = deque([state])
    expanded = 0
    while queue:
        cur = queue.popleft()
        expanded += 1
        for s in cur.successors():
            if use_dead and creates_dead_box(cur, s, dead):
                continue
            k = key(s)
            if k in seen:
                continue
            if sokoban_goal_state(s):
                return s.gval, expanded, "ok"
            seen.add(k)
            queue.append(s)
            if len(seen) > MAX_SEEN:
                return None, expanded, "quá lớn"
    return None, expanded, "vô nghiệm"


def fmt(result):
    length, expanded, note = result
    if length is None:
        return "%-9s" % note
    return "%3d bước, %6d nút" % (length, expanded)


if __name__ == "__main__":
    ids = [int(a) for a in sys.argv[1:]] or list(range(8))
    print("map | BFS thường            | + ô chết             | + ô chết + canonical | kiểm tra")
    wrong = 0
    for i in ids:
        r0 = bfs(PROBLEMS[i])
        r1 = bfs(PROBLEMS[i], use_dead=True)
        r2 = bfs(PROBLEMS[i], use_dead=True, use_canon=True)
        lengths = {r[0] for r in (r0, r1, r2) if r[2] == "ok"}
        ok = "OK" if len(lengths) <= 1 else "SAI: độ dài khác nhau"
        if ok != "OK":
            wrong += 1
        print("%3d | %s | %s | %s | %s" % (i, fmt(r0), fmt(r1), fmt(r2), ok))
    print("Số map bị sai:", wrong)
