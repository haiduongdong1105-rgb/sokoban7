"""In bản đồ kèm ô chết để kiểm tra bằng mắt.
Chạy:  python show_dead.py 1        (số 0..19 là chỉ số bản đồ)
Ký hiệu:  # tường   . ô đích   x ô chết   (trống) ô sống.  Hai bên có dấu | để thấy rõ khoảng trắng.
"""
import sys
from sokoban import PROBLEMS
from dead_squares import compute_dead_squares


def show(state):
    dead = compute_dead_squares(state)
    print("+" + "-" * state.width + "+")
    for y in range(state.height):
        row = ""
        for x in range(state.width):
            c = (x, y)
            if c in state.obstacles:
                row += "#"
            elif c in state.storage:
                row += "."
            elif c in dead:
                row += "x"
            else:
                row += " "
        print("|" + row + "|")
    print("+" + "-" * state.width + "+")
    print("số ô chết:", len(dead))


if __name__ == "__main__":
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    show(PROBLEMS[idx])
