"""
Hướng đi giống framework: 0=lên, 1=phải, 2=xuống, 3=trái.
"""
from collections import deque

DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))   # (dx, dy)


def is_free(state, cell):
    """True nếu `cell` nằm TRONG bàn chơi và KHÔNG phải tường."""
    x, y = cell
    return 0 <= x < state.width and 0 <= y < state.height and cell not in state.obstacles


def push_distances(state):
    """Trả về dict {ô: số lần ĐẨY tối thiểu để đưa thùng từ ô đó tới ô đích gần nhất}.

    Ô không bao giờ tới được đích thì KHÔNG có mặt trong dict.
    Làm bằng BFS NGƯỢC, bắt đầu từ MỌI ô đích cùng lúc.
    """
    dist = {}
    queue = deque()

    for s in state.storage:
        dist[s] = 0
        queue.append(s)

    while queue:
        p = queue.popleft()
        px, py = p
        for dx, dy in DIRS:
            # Giả sử thùng vừa bị đẩy theo hướng (dx, dy) để TỚI ô p.
            #   - lúc trước nó đứng ở ô `prev`
            #   - robot đứng ở ô `robot` để thực hiện cú đẩy đó
            prev = (px - dx, py - dy)
            robot = (px - 2 * dx, py - 2 * dy)
            if is_free(state, prev) and is_free(state, robot) and prev not in dist:
                dist[prev] = dist[p] + 1
                queue.append(prev)
    return dist


def compute_dead_squares(state):
    """Trả về frozenset các ô tự do (không phải tường) mà thùng đặt vào đó
    thì KHÔNG BAO GIỜ tới được đích nào."""
    dist = push_distances(state)
    return frozenset((x, y) for x in range(state.width) for y in range(state.height)
                     if is_free(state, (x, y)) and (x, y) not in dist)