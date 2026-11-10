# Mô tả thiết kế solver Sokoban

**Đề tài:** Solver Sokoban nhiều robot bằng Anytime Weighted A\*

**Nhóm 4 thành viên**

| MSSV | Họ tên | Phần phụ trách |
| --- | --- | --- |
| 25023206 | Đồng Hải Dương | Tính ô chết, deadlock đóng băng (freeze) |
| 25023364 | Nguyễn Khắc Nhật Quang | Heuristic ghép thùng - đích |
| 25023159 | Nguyễn Tương Thị Quỳnh Anh | Core search (Weighted A\*, vòng anytime) |
| 25023242 | Trịnh Văn Hiệp | Kiểm thử, benchmark, chất lượng lời giải |


## 1. Thiết kế

### 1.1. Mô hình bài toán

Bàn chơi gồm nhiều robot, thùng, ô đích (storage) và tường. Mỗi hành động chọn một robot đi một ô theo một trong bốn hướng; nếu ô kế tiếp có thùng thì robot đẩy thùng đi một ô (chỉ khi ô sau thùng trống và nằm trong bàn). Không kéo thùng, không đẩy hai thùng cùng lúc, các robot không đi xuyên nhau. Mục tiêu là mọi thùng nằm trên ô đích (có thể có nhiều đích hơn thùng).

Điểm đáng chú ý của đề này là **mỗi lần di chuyển một robot tính 1 bước, kể cả đi bộ lẫn đẩy thùng**. Vì vậy chất lượng lời giải được đo bằng tổng số bước (`gval`), không chỉ số lần đẩy như trong Sokoban thông thường. Hàm `successors()` của framework sinh tối đa 4 × (số robot) trạng thái kế, mỗi trạng thái có chi phí chuyển 1.

Trạng thái tìm kiếm là cặp (vị trí các robot, tập thùng). Vì các robot giống hệt nhau về luật chơi, khóa nhận biết trạng thái đã thăm sắp xếp vị trí robot trước khi so sánh: `(tuple(sorted(robots)), boxes)`. Cách này gộp các trạng thái chỉ khác nhau do hoán vị robot, giảm đáng kể không gian tìm kiếm với các map có 3-4 robot.

### 1.2. Tổng quan kiến trúc

Solver được tổ chức thành bốn khối, ứng với phân công của nhóm:

1. **Ô chết** (`_push_distances_from`, `_dead_squares`): tính trước một lần ở đầu hàm `solve`.
2. **Deadlock đóng băng** (`_is_wall`, `_blocked`, `_frozen`, `_freeze_deadlock`): kiểm tra mỗi khi có thùng bị đẩy.
3. **Heuristic** (`_build_heuristic`): trả về hàm `h(boxes)` có bộ nhớ đệm.
4. **Tìm kiếm** (`_weighted_astar`, `solve`): Weighted A\* chạy lặp với trọng số `w` giảm dần và điều kiện dừng theo thời gian.

Luồng chạy của `solve(initial_state, timebound)`:

1. Nếu trạng thái đầu đã là đích thì trả về luôn.
2. Tính khoảng cách đẩy ngược từ mọi ô đích, suy ra tập ô chết.
3. Dựng heuristic. Nếu `h(trạng thái đầu)` là vô hạn thì chứng minh được vô nghiệm và trả `False`.
4. Chạy vòng anytime: lần lượt Weighted A\* với `w = 3, 2, 1.5, 1`, giữ lại lời giải tốt nhất.
5. Hết thời gian hoặc điều kiện dừng thì trả lời giải tốt nhất đang có, chưa có thì trả đúng `False`.

Hàm không sửa `initial_state`, và trả về chính đối tượng trạng thái đích do `successors()` sinh ra, nên chuỗi `parent`, `action`, `gval` luôn hợp lệ theo yêu cầu của `validation.py`. Toàn bộ thân hàm được bọc trong `try/except`: nếu có lỗi bất ngờ thì vẫn trả lời giải tốt nhất đã tìm được (hoặc `False`) thay vì làm crash tiến trình.

### 1.3. Ô chết (phụ trách: Đồng Hải Dương)

Ô chết là ô đi vào được nhưng nếu đặt một thùng vào đó thì thùng không bao giờ tới được ô đích nào, bất kể robot di chuyển thế nào.

**Cách tính.** Hàm `_push_distances_from(state, sources)` chạy BFS **ngược** từ các ô đích. Từ một ô thùng `p`, với mỗi hướng `(dx, dy)`, ô `prev = p - d` là ô thùng có thể đứng ngay trước cú đẩy, còn `robot = p - 2d` là ô robot phải đứng để thực hiện cú đẩy. Cả hai ô phải nằm trong bàn và không phải tường thì `prev` mới được thêm vào với khoảng cách `dist[p] + 1`. Kết quả là số lần đẩy tối thiểu để đưa một thùng từ mỗi ô tới đích, bỏ qua các thùng khác (nên là cận dưới).

Hàm `_dead_squares` lấy các ô tự do không xuất hiện trong bảng khoảng cách trên. Đây chính là các ô chết đơn giản (simple deadlock): các góc, các ô sát tường không dẫn được tới đích, v.v.

**Cách dùng.** Trong tìm kiếm, mỗi lần một thùng bị đẩy tới ô mới, nếu ô đó thuộc tập ô chết thì loại luôn trạng thái kế. Vì tập ô chết được tính bằng BFS ngược nên mọi ô bị loại đều thật sự không thể cứu được, tức là việc cắt tỉa này **an toàn** (không làm mất lời giải).

### 1.4. Deadlock đóng băng (phụ trách: Đồng Hải Dương)

Một thùng bị coi là *đóng băng* nếu nó không thể di chuyển theo cả hai trục, dù robot làm gì. Nếu có thùng đóng băng nằm ngoài ô đích thì trạng thái đó chắc chắn vô nghiệm.

**Điều kiện chặn theo một trục.** Hàm `_blocked(cell, axis)` xét hai ô lân cận theo trục đó (trái/phải hoặc trên/dưới). Thùng bị chặn theo trục nếu một trong các điều kiện sau đúng:

- một trong hai ô lân cận là tường hoặc nằm ngoài bàn;
- một trong hai ô lân cận là thùng khác mà bản thân thùng đó cũng đóng băng (kiểm tra đệ quy bằng `_frozen`), hoặc thùng đó đang nằm trong tập `visiting` (phụ thuộc vòng, ví dụ khối 2x2 các thùng, được xem là chặn lẫn nhau);
- cả hai ô lân cận đều là ô chết (thùng có đẩy được sang hai bên cũng chỉ rơi vào ô chết).

Thùng đóng băng khi bị chặn ở **cả hai** trục (`_frozen`). Tập `visiting` được truyền theo đệ quy để tránh lặp vô hạn khi các thùng chặn lẫn nhau.

**Phạm vi.** `_freeze_deadlock` duyệt các thùng không nằm trên ô đích, nếu có thùng đóng băng thì trả `True`. Robot không được tính là vật chắn vì robot có thể di chuyển đi chỗ khác. Kiểm tra này chỉ chạy khi có thùng bị đẩy (`moved = s.boxes - cur.boxes` khác rỗng), vì nước đi bộ không làm thay đổi cấu hình thùng. Cách xử lý này bao phủ các trường hợp quen thuộc: thùng kẹt sát tường không có đích dọc theo tường, khối 2x2, và các cụm thùng chặn nhau.

### 1.5. Heuristic ghép thùng - đích (phụ trách: Nguyễn Khắc Nhật Quang)

**Ý tưởng.** Với mỗi ô đích, solver dùng BFS ngược (cùng hàm `_push_distances_from`) để tính trước khoảng cách đẩy từ mọi ô tới đích đó. Lúc chạy, với một tập thùng, ta lập ma trận chi phí `rows[i][j]` = số lần đẩy tối thiểu để thùng `i` tới đích `j`, rồi tìm **phép ghép một-một có tổng chi phí nhỏ nhất** giữa thùng và đích.

**Cách tính.**

- Nếu số thùng nhiều hơn số đích thì trả vô hạn (không thể xong).
- Nếu có thùng mà mọi chi phí đều vô hạn (thùng ở ô không thể tới đích nào) thì trả vô hạn.
- Nếu số đích `m ≤ 12` (`MAX_MASK_STORAGES`) thì giải bài toán gán tối ưu bằng quy hoạch động theo bitmask: `dp[mask]` là chi phí nhỏ nhất khi các thùng đã xét được gán vào tập đích `mask`; với mỗi thùng, thử gán vào mọi đích chưa dùng. Độ phức tạp cỡ `O(số thùng × 2^m × m)` nhưng được cắt gọn vì chỉ giữ các mask thực sự xuất hiện.
- Nếu `m > 12` thì dùng cận lỏng hơn là tổng của min mỗi hàng (mỗi thùng chọn đích gần nhất, cho phép nhiều thùng cùng đích). Cận này thấp hơn phép ghép tối ưu nhưng vẫn là cận dưới hợp lệ, và tránh bùng nổ bộ nhớ.
- Kết quả được lưu trong `cache` theo khóa là tập thùng, vì nhiều trạng thái chỉ khác nhau vị trí robot (cùng tập thùng) nên dùng lại được rất nhiều.

**Tính chấp nhận được (admissible).** Mỗi lần đẩy một thùng cần ít nhất 1 bước (và mỗi bước chỉ đẩy được tối đa một thùng), còn heuristic chỉ đếm số lần đẩy tối thiểu mà bỏ qua quãng đường đi bộ và sự cản trở giữa các thùng. Do đó `h` không bao giờ đánh giá cao hơn chi phí thật tính theo bước, tức là **admissible**. Heuristic cũng bằng 0 khi mọi thùng đã ở đích.

**Điểm yếu cần nêu rõ.** Heuristic không tính quãng đường robot phải đi tới thùng, nên khoảng cách giữa `g + h` và chi phí thật khá lớn. Đây là lý do với w = 1 solver khó chứng minh được tính tối ưu trên các map lớn (xem mục 2).

### 1.6. Core search: Weighted A\* dạng anytime (phụ trách: Nguyễn Tương Thị Quỳnh Anh)

**Weighted A*.*\* Hàm `_weighted_astar` dùng hàng đợi ưu tiên `heapq` với khóa `f = g + w·h`. Phần tử hàng đợi là bộ `(f, h, bộ đếm, trạng thái)`: bộ đếm `itertools.count` đóng vai trò tie-breaker để không phải so sánh hai đối tượng trạng thái. Với `w > 1`, thuật toán thiên về các trạng thái có `h` nhỏ, nhanh chóng tìm được lời giải nhưng có thể chưa tối ưu; với `w = 1` là A\* chuẩn.

**Bảng g tốt nhất.** `seen_g` lưu `g` nhỏ nhất đã biết cho mỗi khóa trạng thái. Một trạng thái kế chỉ được xét nếu `g` mới nhỏ hơn giá trị đã lưu; các bản cũ còn sót trong hàng đợi bị bỏ qua khi lấy ra nếu `g` không còn khớp (lazy deletion).

**Cắt nhánh.** Trước khi đưa trạng thái kế vào hàng đợi, solver loại bỏ nếu:

- thùng bị đẩy vào ô chết;
- xuất hiện deadlock đóng băng;
- `h = vô hạn`;
- `g + h ≥ best_g`, với `best_g` là số bước của lời giải tốt nhất đã có. Vì `h` admissible nên nhánh này không thể cho lời giải tốt hơn.

Khi gặp trạng thái đích thì cập nhật `best_goal`. Nếu `w > 1`, lượt chạy kết thúc ngay (`FOUND`) để lượt sau dùng `w` nhỏ hơn cải thiện; nếu `w = 1` thì tiếp tục tìm vì vẫn có thể còn lời giải tốt hơn.

**Vòng anytime trong `solve`.** Lần lượt chạy `w = 3, 2, 1.5, 1`. Mỗi lượt sau được truyền `best_goal` của lượt trước nên cắt được rất nhiều nhánh bằng điều kiện `g + h ≥ best_g`. Các trạng thái kết thúc lượt:

| Trạng thái | Ý nghĩa | Hành động |
| --- | --- | --- |
| `FOUND` | Tìm được lời giải với `w > 1` | Chuyển sang `w` nhỏ hơn tiếp theo |
| `EXHAUSTED` | Hết mọi nhánh chưa bị cắt | Trả lời giải tốt nhất (hoặc `False` nếu chưa có: vô nghiệm) |
| `TIMEOUT` | Hết thời gian của lượt | Dừng, trả lời giải tốt nhất |
| `CAP` | Vượt `MAX_STATES = 1 500 000` trạng thái | Nếu đã có lời giải thì giữ lại; nếu chưa thì thử `w` tham lam hơn (6, 12, 40) |

Giới hạn `MAX_STATES` nhằm tránh tràn RAM. Nhánh `FALLBACK_WEIGHTS` đảm bảo các map mà `w = 3` quá tốn bộ nhớ vẫn có cơ hội tìm được lời giải đầu tiên bằng cách tìm kiếm tham lam hơn.

### 1.7. Quản lý thời gian và điều kiện dừng sớm

Ban đầu solver dùng tới `SAFETY = 85%` của `timebound` (102 giây khi `timebound = 120`). Quan sát cho thấy trên các map khó (9, 10, 15-19), lượt `w = 1` không bao giờ cạn nên solver cứ chạy tới hết giờ dù lời giải cuối cùng thường đã xuất hiện từ rất sớm. Để giảm rủi ro bị tính timeout trên máy chấm chậm hơn, nhóm thêm hai tham số tính theo tỉ lệ `timebound` (vì vậy vẫn đúng nếu bộ chấm dùng `timebound` khác 120):

- `STALL_FRAC = 0.12` (khoảng 14,4 giây): sau khi **đã có lời giải**, nếu chừng đó thời gian không tìm được lời giải tốt hơn thì dừng. Mỗi lần cải thiện sẽ gia hạn thêm một đoạn như vậy.
- `SOFT_FRAC = 0.5` (60 giây): sau khi đã có lời giải, tổng thời gian không vượt quá mức này.

Cả hai chỉ có tác dụng sau khi đã có lời giải. Nếu chưa có lời giải thì solver vẫn dùng đến 85% như cũ, nên cơ chế này không làm tăng khả năng trả về `False`. Việc kiểm tra thời gian được thực hiện cứ mỗi 64 lần mở rộng nút để giảm chi phí gọi `perf_counter`.

### 1.8. Phân công và phối hợp

| Thành viên | Đóng góp chính |
| --- | --- |
| Đồng Hải Dương | Thiết kế và cài đặt BFS ngược tính ô chết; kiểm tra deadlock đóng băng (khối 2x2, thùng sát tường, thùng chặn lẫn nhau) |
| Nguyễn Khắc Nhật Quang | Heuristic khoảng cách đẩy và ghép thùng - đích bằng DP bitmask, bộ nhớ đệm, cận lỏng khi nhiều đích |
| Nguyễn Tương Thị Quỳnh Anh | Vòng tìm kiếm Weighted A\* (heapq, tie-breaker, bảng g tốt nhất), vòng anytime, xử lý bộ nhớ và các trạng thái kết thúc |
| Trịnh Văn Hiệp | Bộ kiểm thử và benchmark theo từng map (mỗi map một tiến trình riêng), so sánh với kết quả gốc để phát hiện hồi quy, đánh giá chất lượng lời giải và điều chỉnh tham số dừng sớm |

Các phần được viết riêng rồi ghép lại thành một file `solution.py` duy nhất theo yêu cầu nộp bài.

## 2. Chất lượng lời giải

### 2.1. Cách tìm và cải thiện lời giải

Solver là thuật toán **anytime**: lời giải đầu tiên được tìm nhanh bằng `w` lớn, sau đó được cải thiện dần bằng cách giảm `w`, mỗi lượt tận dụng cận `best_g` của lượt trước để cắt nhánh. Ở bất kỳ thời điểm nào, nếu đã có lời giải thì kết quả trả về luôn là lời giải tốt nhất tìm được cho tới lúc đó.

### 2.2. Khi nào lời giải là tối ưu

Nhóm **chỉ tuyên bố tối ưu trong trường hợp cụ thể** sau: lượt `w = 1` kết thúc với trạng thái `EXHAUSTED`. Khi đó mọi nhánh chưa bị cắt đều có `g + h ≥ best_g`. Điều này đúng được vì ba điều kiện:

1. `h` admissible (mục 1.5), nên cắt theo `g + h ≥ best_g` không loại lời giải tốt hơn;
2. cắt ô chết là an toàn (mục 1.3), vì ô chết được suy ra bằng BFS ngược;
3. deadlock đóng băng là điều kiện đủ để trạng thái vô nghiệm (mục 1.4).

Các map như 11-14 và các map nhỏ kết thúc theo kiểu này, nên lời giải ở đó có cơ sở là tối ưu về số bước (theo mô hình đã nêu).

Với các map khó hơn (9, 10, 15-19), lượt `w = 1` không cạn được trong thời gian cho phép nên solver dừng do điều kiện thời gian. **Nhóm không tuyên bố tối ưu cho các map này**; lời giải chỉ là lời giải tốt nhất tìm được trong ngân sách thời gian và bộ nhớ. Tương tự, nếu lượt chạy bị dừng do `CAP` thì cũng không có chứng minh tối ưu.

Trường hợp trả về `False` có hai nguyên nhân khác nhau: (a) chứng minh được vô nghiệm (heuristic vô hạn ở trạng thái đầu, hoặc `EXHAUSTED` khi chưa có lời giải), hoặc (b) hết thời gian/bộ nhớ khi chưa tìm được lời giải nào. Trường hợp (b) không phải bằng chứng vô nghiệm.

### 2.3. Đánh đổi giữa thời gian và số bước

Cơ chế dừng sớm (mục 1.7) đổi một chút chất lượng lấy sự an toàn về thời gian. Kết quả đo ở mục 3 cho thấy tổng số bước tăng nhẹ (706 lên 710) trong khi thời gian chạy trên các map nặng giảm từ khoảng 102 giây xuống còn khoảng 15-25 giây.

## 3. Thực nghiệm

### 3.1. Môi trường và cách đo

- Chạy bằng `benchmark.py`: mỗi map chạy trong một tiến trình Python mới, `timebound = 120`.
- Mỗi lời giải được kiểm tra lại bằng `validation.py` (phát lại toàn bộ chuỗi `parent`, kiểm tra chi phí từng bước, tính hợp lệ của từng nước đi và trạng thái cuối).
- Có kiểm tra `solve()` không làm thay đổi `initial_state`.
- Cũng chạy `autograder.py` (public test) để kiểm tra giao diện.
- Môi trường: Python 3.14, Windows, chạy trong VS Code. Máy cụ thể: \[điền cấu hình CPU/RAM nếu muốn ghi\].

### 3.2. Kết quả trên 20 map thực hành

Cả 20 map đều được giải và qua kiểm tra hợp lệ (PASS 20/20). Bảng dưới so sánh bản gốc (chạy tới 85% `timebound`, tức 102 giây) với bản có dừng sớm (`STALL_FRAC = 0.12`, `SOFT_FRAC = 0.5`) cho các map khó:

| Map | Số bước (gốc) | Thời gian (gốc) | Số bước (dừng sớm) | Thời gian (dừng sớm) |
| --- | --- | --- | --- | --- |
| 0 | 16 | 0,3 s | 16 | 0,3 s |
| 1 | 4 | 0,0 s | 4 | 0,0 s |
| 2 | 21 | 0,0 s | 21 | 0,0 s |
| 3 | 10 | 0,0 s | 10 | 0,0 s |
| 4 | 8 | 0,0 s | 8 | 0,0 s |
| 5 | 30 | 0,3 s | 30 | 0,3 s |
| 6 | 16 | 0,0 s | 16 | 0,0 s |
| 7 | 41 | 0,0 s | 41 | 0,0 s |
| 8 | 14 | 0,2 s | 14 | 0,2 s |
| 9 | 24 | 102,4 s | 24 | 14,5 s |
| 10 | 30 | 102,9 s | 32 | 15,3 s |
| 11 | 26 | 68,3 s | 26 | 16,8 s |
| 12 | 24 | 34,1 s | 24 | 16,7 s |
| 13 | 27 | 12,6 s | 27 | 6,8 s |
| 14 | 25 | 8,9 s | 25 | 5,1 s |
| 15 | 69 | 103,7 s | 69 | 25,4 s |
| 16 | 54 | 103,5 s | 54 | 19,3 s |
| 17 | 100 | 104,0 s | 100 | 16,0 s |
| 18 | 94 | 104,2 s | 94 | 15,9 s |
| 19 | 73 | 104,3 s | 75 | 15,1 s |



**Tổng số bước trên 20 map:** 706 (bản gốc) và 710 (bản dừng sớm).

### 3.3. Nhận xét

- **Độ chính xác:** vẫn giải được 20/20 map ở cả hai cấu hình, không có hồi quy về khả năng giải.
- **Số bước:** chỉ hai map có lời giải dài hơn (map 10: 30 lên 32; map 19: 73 lên 75), tổng tăng 4 bước (khoảng 0,6%). Các map còn lại giữ nguyên số bước.
- **Thời gian:** các map trước đây chạm 102-104 giây nay chỉ còn 15-25 giây; map 11 và 12 cũng giảm mạnh (68,3 s xuống 16,8 s và 34,1 s xuống 16,7 s). Điều này cho thấy lời giải tốt nhất thường xuất hiện sớm, phần thời gian còn lại của bản gốc chủ yếu đóng góp rất ít.
- **Khoảng đệm an toàn:** thời gian chạy tối đa hiện khoảng 25 giây so với giới hạn 120 giây, nên rủi ro bị tính timeout trên máy chậm hơn giảm đáng kể.
- Hai map 10 và 19 cho thấy rõ đánh đổi: dừng sớm đã cắt mất một vài lần cải thiện cuối. Nếu ưu tiên số bước hơn thời gian có thể tăng `STALL_FRAC` hoặc `SOFT_FRAC`.

## 4. Hạn chế và nguồn tham khảo

### 4.1. Hạn chế

1. **Heuristic yếu:** chỉ đếm số lần đẩy, không tính quãng đường đi bộ của robot và không xét việc các thùng cản nhau. Do đó trên các map lớn lượt `w = 1` không cạn được và không thể tuyên bố tối ưu.
2. **Deadlock chưa đầy đủ:** mới có ô chết đơn giản và deadlock đóng băng. Chưa có các kiểu nâng cao như corral deadlock, cơ sở dữ liệu mẫu deadlock 2x3, hay kiểm tra tính liên thông của vùng robot tới được.
3. **Chi phí sinh trạng thái kế:** solver gọi `successors()` của framework, mỗi lần tạo nhiều đối tượng `SokobanState` (kể cả cho nước đi bộ). Một hướng cải thiện là dùng nút nhẹ trong lúc tìm kiếm rồi chỉ dựng lại đường đi thật ở cuối; nhóm chưa làm vì rủi ro lỗi cao hơn lợi ích.
4. **Không gian trạng thái lớn:** vì mỗi bước đi bộ cũng là một nút nên số trạng thái lớn hơn nhiều so với tìm kiếm theo lần đẩy. Giới hạn `MAX_STATES` có thể khiến một lượt dừng sớm do `CAP`.
5. **Dừng sớm có thể cắt cải thiện:** như đã thấy ở map 10 và 19, tham số `STALL_FRAC`, `SOFT_FRAC` được chọn bằng thực nghiệm chứ không có bảo đảm lý thuyết.
6. **Heuristic dùng cận lỏng khi có hơn 12 ô đích** (`MAX_MASK_STORAGES`), làm cận yếu đi trong trường hợp đó.
7. **Hiệu năng Python:** thuần Python nên chậm hơn đáng kể so với các solver chuyên dụng viết bằng ngôn ngữ biên dịch.
8. **Số liệu thực nghiệm** chỉ dựa trên một lần chạy mỗi map trên một máy; thời gian có thể dao động giữa các lần chạy.

### 4.2. Nguồn tham khảo và công cụ đã dùng

- Hart, Nilsson, Raphael (1968), *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*: thuật toán A\*.
- Pohl (1970), *Heuristic Search Viewed as Path Finding in a Graph*: Weighted A\*.
- Culberson (1997/1999), *Sokoban is PSPACE-complete*: độ phức tạp của bài toán.
- Junghanns & Schaeffer, các bài về solver Rolling Stone: ô chết, deadlock, heuristic ghép thùng - đích.
- Wiki cộng đồng Sokoban (sokobano.de/wiki): các kỹ thuật deadlock và ghép thùng - đích.
- Mã nguồn cung cấp bởi giảng viên: `sokoban.py`, `state.py`, `validation.py`, `benchmark.py`, `autograder.py`.
- Công cụ AI đã sử dụng: Claude (Anthropic) để thảo luận hướng cải thiện thời gian chạy (cơ chế dừng sớm) và hỗ trợ soạn báo cáo. \[Nhóm kiểm tra lại và sửa lại nếu có lỗi sai.\]
- Toàn bộ mã nguồn đã được nhóm đọc, chạy và kiểm chứng trước khi nộp.
