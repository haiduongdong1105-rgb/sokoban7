"""NGƯỜI C - Tuần 2: lọc nước đi bằng ô chết, và khóa trạng thái đối xứng.

Chỉ dùng thư viện chuẩn. Hàm nào cần ô chết thì nhận `dead` (frozenset) làm tham số,
tính một lần ở đầu `solve` bằng compute_dead_squares(initial_state).
"""
from dead_squares import compute_dead_squares   # MERGE:REMOVE


def pushed_box(parent, child):
    """Nếu bước đi từ `parent` sang `child` ĐẨY một thùng, trả về ô MỚI của thùng đó.
    Nếu chỉ đi bộ (không đẩy gì) thì trả về None."""
    new_cells = child.boxes - parent.boxes        # các ô có thùng ở `child` mà `parent` chưa có
    if new_cells:
        return next(iter(new_cells))              # lấy phần tử duy nhất của tập
    return None


def creates_dead_box(parent, child, dead):
    """True nếu bước đi này đẩy một thùng vào ô chết (ván coi như hỏng)."""
    cell = pushed_box(parent, child)
    return cell is not None and cell in dead


def filter_successors(state, dead):
    """Giống state.successors() nhưng bỏ các trạng thái đẩy thùng vào ô chết."""
    return [s for s in state.successors() if not creates_dead_box(state, s, dead)]


def canonical_key(state):
    """Khóa nhận diện trạng thái, đã sắp xếp robot để gộp các trạng thái đối xứng.

    CHỈ dùng làm khóa trong tập đã thăm (visited). KHÔNG thay trạng thái thật bằng bản sắp xếp.
    """
    return (tuple(sorted(state.robots)), state.boxes)
