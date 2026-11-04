"""Tuần 3 của C: deadlock "đóng băng" (freeze), gồm cả luật 2x2 và thùng dựa tường.

Một thùng bị ĐÓNG BĂNG nếu nó không thể nhúc nhích theo cả hai trục (ngang và dọc),
dù robot làm gì đi nữa. Nếu có thùng đóng băng mà KHÔNG nằm trên ô đích thì không
bao giờ thắng được nữa => trạng thái chết, bỏ.

Thùng bị CHẶN theo một trục nếu một trong hai bên (trái/phải, hoặc trên/dưới) là:
  - tường / ngoài bàn / vật cản, hoặc
  - một thùng khác mà bản thân nó cũng đóng băng, hoặc
  - cả hai bên đều là ô chết.
Robot KHÔNG tính là vật chặn (robot đi chỗ khác được).
"""
DIRS_PAIR = (((-1, 0), (1, 0)),     # trục ngang: trái, phải
             ((0, -1), (0, 1)))     # trục dọc: trên, dưới


def is_wall(state, cell):
    x, y = cell
    return not (0 <= x < state.width and 0 <= y < state.height) or cell in state.obstacles


def box_frozen(state, boxes, dead, cell, visiting):
    visiting = visiting | {cell}
    return (blocked(state, boxes, dead, cell, 0, visiting)
            and blocked(state, boxes, dead, cell, 1, visiting))


def blocked(state, boxes, dead, cell, axis, visiting):
    x, y = cell
    both_dead = True
    for dx, dy in DIRS_PAIR[axis]:
        n = (x + dx, y + dy)
        if is_wall(state, n):
            return True
        if n in boxes and (n in visiting or box_frozen(state, boxes, dead, n, visiting)):
            return True
        if n not in dead:
            both_dead = False
    return both_dead


def is_freeze_deadlock(state, dead):
    """True nếu trong state có thùng đóng băng nằm ngoài ô đích."""
    boxes = state.boxes
    for b in boxes:
        if b not in state.storage and box_frozen(state, boxes, dead, b, frozenset()):
            return True
    return False


def filter_successors2(state, dead):
    """Như pruning.filter_successors nhưng loại thêm trạng thái có thùng đóng băng."""
    from pruning import creates_dead_box
    out = []
    for s in state.successors():
        if creates_dead_box(state, s, dead):
            continue
        if s.boxes != state.boxes and is_freeze_deadlock(s, dead):
            continue
        out.append(s)
    return out
