from collections import deque

INF = 10 ** 9
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))


def build_heuristic(state):
    """Tính trước bảng khoảng cách, trả về hàm h(state)."""
    W, H, walls = state.width, state.height, state.obstacles
    storages = sorted(state.storage)
    m = len(storages)

    def free(c):
        return 0 <= c[0] < W and 0 <= c[1] < H and c not in walls

    # dist_to[j][cell] = số lần đẩy tối thiểu để đưa thùng từ cell tới storages[j]
    dist_to = []
    for s in storages:
        d = {s: 0}
        q = deque([s])
        while q:
            p = q.popleft()
            for dx, dy in DIRS:
                prev = (p[0] - dx, p[1] - dy)       # ô thùng đứng trước cú đẩy
                robot = (p[0] - 2 * dx, p[1] -  2 * dy)  # ô robot đứng để đẩy
                if free(prev) and free(robot) and prev not in d:
                    d[prev] = d[p] + 1
                    q.append(prev)
        dist_to.append(d)

    cache = {}

    def h(st):
        boxes = st.boxes                 # frozenset, dùng làm khóa cache luôn
        if boxes in cache:
            return cache[boxes]
        dp = {0: 0}                      # dp[mask] = chi phí khi các đích trong mask đã dùng
        for b in boxes:
            new = {}
            for mask, c in dp.items():
                for j in range(m):
                    if mask >> j & 1:
                        continue
                    d = dist_to[j].get(b, INF)
                    if d >= INF:
                        continue
                    nm = mask | (1 << j)
                    v = c + d
                    if v < new.get(nm, INF):
                        new[nm] = v
            dp = new
            if not dp:                   # có thùng không tới được đích nào
                cache[boxes] = INF
                return INF
        res = min(dp.values())
        cache[boxes] = res
        return res

    return h