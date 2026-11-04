"""4 tình huống làm tay cho is_freeze_deadlock. Chạy: python test_freeze.py"""
from sokoban import SokobanState
from dead_squares import compute_dead_squares
from deadlock import is_freeze_deadlock


def make(w, h, robots, boxes, storage, walls=()):
    return SokobanState("START", 0, None, w, h, tuple(robots), frozenset(boxes),
                        frozenset(storage), frozenset(walls))


CASES = [
    ("A. khối 2x2 giữa bàn, chưa ô nào là đích", True,
     make(6, 5, [(5, 4)], [(1, 1), (2, 1), (1, 2), (2, 2)], [(5, 0), (4, 0), (3, 0), (0, 4)])),
    ("B. khối 2x2 nhưng cả 4 thùng đã ở đích", False,
     make(6, 5, [(5, 4)], [(1, 1), (2, 1), (1, 2), (2, 2)], [(1, 1), (2, 1), (1, 2), (2, 2)])),
    ("C. hai thùng cạnh nhau sát tường trên (hàng 0 có đích ở xa)", True,
     make(6, 4, [(0, 3)], [(1, 0), (2, 0)], [(5, 0), (4, 0), (0, 2)])),
    ("D. một thùng đứng một mình giữa bàn", False,
     make(6, 5, [(5, 4)], [(2, 2)], [(0, 0)])),
]
bad = 0
for name, expected, st in CASES:
    got = is_freeze_deadlock(st, compute_dead_squares(st))
    ok = got == expected
    bad += not ok
    print("%-62s mong đợi %-5s được %-5s %s" % (name, expected, got, "OK" if ok else "SAI"))
print("Số ca sai:", bad)
