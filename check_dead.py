"""Kiểm tra ĐỘC LẬP rằng không ô nào bị đánh dấu chết nhầm (đây là hướng nguy hiểm).

Với mỗi ô chết, thử đặt MỘT thùng ở đó và MỘT robot ở mọi vị trí có thể, rồi dùng
successors() của framework để thử mọi cách chơi. Nếu có cách thắng thì ô đó KHÔNG chết (bạn sai).

Chạy:  python check_dead.py            (các map nhỏ 0..13)
       python check_dead.py 1 3 4      (chỉ các map đã chọn)
Chỉ nên chạy map nhỏ (5x5, 6x6); map 8x8 sẽ rất chậm.
"""
import sys
from collections import deque
from sokoban import PROBLEMS, SokobanState, sokoban_goal_state
from dead_squares import compute_dead_squares


def can_win(state):
    seen = {state.hashable_state()}
    q = deque([state])
    while q:
        cur = q.popleft()
        if sokoban_goal_state(cur):
            return True
        for s in cur.successors():
            k = s.hashable_state()
            if k not in seen:
                seen.add(k)
                q.append(s)
    return False


def check(state):
    dead = compute_dead_squares(state)
    free = [(x, y) for x in range(state.width) for y in range(state.height)
            if (x, y) not in state.obstacles]
    wrong = []
    for cell in sorted(dead):
        for robot in free:
            if robot == cell:
                continue
            test = SokobanState("START", 0, None, state.width, state.height, (robot,),
                                frozenset([cell]), state.storage, state.obstacles)
            if can_win(test):
                wrong.append((cell, robot))
                break
    return dead, wrong


if __name__ == "__main__":
    ids = [int(a) for a in sys.argv[1:]] or list(range(14))
    total_wrong = 0
    for i in ids:
        dead, wrong = check(PROBLEMS[i])
        status = "OK" if not wrong else "SAI"
        print("map %2d: %2d ô chết -> %s" % (i, len(dead), status))
        for cell, robot in wrong:
            print("    ô %s bị đánh dấu chết nhưng thùng đặt ở đó vẫn thắng được khi robot ở %s" % (cell, robot))
        total_wrong += len(wrong)
    print("Tổng số ô chết sai:", total_wrong)
