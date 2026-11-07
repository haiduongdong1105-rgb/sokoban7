# Mô tả thiết kế solver Sokoban

Tài liệu này dùng để giải thích bài nộp, không có điểm riêng trong thang chấm.
Không bắt buộc một thuật toán, heuristic hay cấu trúc dữ liệu cụ thể.

## 1. Thiết kế
Mô tả phương pháp giải, cách tổ chức chương trình và lý do lựa chọn.

## 2. Chất lượng lời giải
Giải thích phương pháp tìm hoặc cải thiện lời giải. Nếu tuyên bố tối ưu,
trình bày cơ sở cho tuyên bố đó; nếu không, nêu rõ giới hạn.

## 3. Thực nghiệm
Ghi các map đã thử, có giải được hay không và số bước của lời giải.
Có thể ghi thêm thời gian để phân tích, dù thời gian không trực tiếp tính điểm.

## 4. Hạn chế và nguồn tham khảo
Nêu phần chưa hoàn thiện, tài liệu, mã nguồn và công cụ AI đã sử dụng.
Mô tả thiết kế solver Sokoban

Nhóm: 07 — [Nguyễn Khắc Nhật Quang - 25023364], [Đồng Hải Dương – 25023206], [Trịnh Văn Hiệp – 25023242], [Nguyễn Tương Thị Quỳnh Anh - 25023159]

1. Thiết kế

Nhóm dùng Weighted A* chạy nhiều lượt với trọng số w giảm dần (3 → 2 → 1.5 → 1), f = g + w·h. Lượt đầu w lớn nên ra lời giải nhanh nhưng chưa chắc ngắn; các lượt sau w nhỏ dần để tìm lời giải ngắn hơn. Lý do chọn: A* thường (w = 1) với map lớn nhiều robot có thể không ra kết quả kịp thời gian, còn chỉ chạy tham lam thì số bước hay bị dài. Cách này đảm bảo có đáp án sớm rồi cải thiện dần.
Code chia ba phần:
- Tìm kiếm (_weighted_astar, solve): dùng heapq, lưu g tốt nhất của từng trạng thái trong seen_g để bỏ trạng thái trùng. Khóa trạng thái là (vị trí robot đã sắp xếp, tập thùng), vì các robot giống nhau nên đổi chỗ không tạo trạng thái mới.
- Heuristic (_build_heuristic): với mỗi ô đích, BFS ngược để tính số lần đẩy tối thiểu đưa thùng từ mỗi ô tới đích đó. h là tổng chi phí ghép thùng–đích rẻ nhất (mỗi đích một thùng), tính bằng quy hoạch động bitmask. Nếu số đích quá nhiều (> 12) thì lấy tổng min của từng thùng cho nhẹ.
- Cắt nhánh deadlock: (1) ô chết: ô mà thùng đặt vào sẽ không bao giờ tới được đích nào (tìm bằng BFS ngược từ mọi đích); (2) freeze deadlock: thùng bị chặn cả hai trục (bởi tường, thùng khác cũng bị kẹt, hoặc ô chết) mà không nằm trên đích thì bỏ trạng thái đó. Hai kiểm tra này chỉ chạy khi vừa có thùng bị đẩy.
Nếu h của trạng thái đầu là vô cực thì trả False luôn vì map vô nghiệm.

2. Chất lượng lời giải
- Mỗi lần tìm được lời giải, nhóm lưu lại và dùng chi phí của nó làm ngưỡng: lượt sau bỏ mọi nhánh có g + h ≥ chi phí đó, nên lời giải mới luôn ngắn hơn lời giải cũ. h chỉ đếm số lần đẩy tối thiểu (mỗi lần đẩy tốn đúng 1 bước), không vượt chi phí thật nên cắt như vậy không làm mất lời giải tốt hơn.
- Nhóm không tuyên bố mọi kết quả đều tối ưu. Chỉ khi một lượt duyệt hết hàng đợi mà không còn nhánh nào tốt hơn thì lời giải đó mới tối ưu (thường xảy ra ở lượt w = 1 trên map nhỏ). Nếu hết giờ hoặc hết bộ nhớ trước đó thì kết quả là lời giải tốt nhất tìm được, hợp lệ nhưng chưa chứng minh tối ưu.
- Về thời gian: dùng tối đa 85% timebound. Khi đã có lời giải, nếu một đoạn (12% timebound) không cải thiện thêm thì dừng, và tổng thời gian không quá 50% timebound. Nếu số trạng thái vượt 1,5 triệu mà chưa có lời giải thì thử w lớn hơn (6, 12, 40) để đỡ tốn bộ nhớ.

3. Thực nghiệm
Nhóm chạy benchmark.py trên 20 map thầy cung cấp (map 0–19). Môi trường: [CPU / RAM / phiên bản Python], timebound = [120] giây.
Map	Giải được?	Số bước	Thời gian (s)
0	Có	16	0,1
1	Có	4	0,0
2	Có	21	0,0
3	Có	10	0,0
4	Có	8	0,0
5	Có	30	0,1
6	Có	16	0,0
7	Có	41	0,0
8	Có	14	0,1
9	Có	24	14,6
10	Có	30	27,4
11	Có	26	15,6
12	Có	24	9,0
13	Có	27	3,8
14	Có	25	2,6
15	Có	69	20,9
16	Có	54	18,2
17	Có	100	15,7
18	Có	94	15,6
19	Có	73	25,3

Kết quả: giải được 20/20 map, tổng 706 bước.

Nhận xét:
Các map 0–8 giải trong chưa tới 0,2 giây. Vì solver dừng sớm như vậy nên tìm kiếm đã duyệt hết các nhánh còn lại ở lượt w = 1, nên nhóm cho rằng lời giải trên các map này là tối ưu.
Các map 13, 14 giải trong 2,6–3,8 giây, cũng dừng rất sớm so với timebound nên nhiều khả năng tối ưu.
Các map 9–12 và 15–19 mất khoảng 9–27 giây. Con số này gần với ngưỡng dừng khi không cải thiện thêm (12% timebound), nên với các map này nhóm chỉ khẳng định lời giải hợp lệ và tốt nhất tìm được, chưa chứng minh tối ưu.

4. Hạn chế và nguồn tham khảo

Hạn chế:
- h không tính quãng đường robot đi bộ nên khá lỏng, map lớn phải mở nhiều trạng thái.
Vị trí robot nằm trong khóa trạng thái, chưa gộp theo vùng robot tới được.
Chỉ bắt ô chết và freeze deadlock, chưa có các deadlock phức tạp hơn (corral, v.v.).
Các tham số (WEIGHTS, STALL_FRAC, MAX_STATES...) chọn bằng thử tay, chưa tinh chỉnh bài bản.
- Phân công: [A: tìm kiếm — Quỳnh Anh], [B: heuristic — Nhật Quang], [C: deadlock — Hải Dương], [D: test - Hiệp].

Tài liệu tham khảo:
Slide môn Artificial Intelligence về tìm kiếm có thông tin (A*).
P. E. Hart, N. J. Nilsson, B. Raphael (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107. (A*, heuristic chấp nhận được)
I. Pohl (1970). Heuristic Search Viewed as Path Finding in a Graph. Artificial Intelligence, 1(3–4), 193–204. (Weighted A*)
M. Likhachev, G. Gordon, S. Thrun (2003). ARA*: Anytime A* with Provable Bounds on Sub-Optimality. NeurIPS 16. (ý tưởng giảm dần trọng số w để cải thiện lời giải)
A. Junghanns, J. Schaeffer (2001). Sokoban: Enhancing General Single-Agent Search Methods Using Domain Knowledge. Artificial Intelligence, 129(1–2), 219–251. (heuristic ghép thùng–đích, cắt tỉa deadlock trong Sokoban)
J. Culberson (1997). Sokoban is PSPACE-complete. Technical Report TR97-02, University of Alberta. (độ khó của bài toán)
S. Russell, P. Norvig (2020). Artificial Intelligence: A Modern Approach, 4th ed., Pearson. Chương 3. (tìm kiếm, heuristic)
Sokoban Wiki, trang "Deadlocks" (sokobano.de/wiki). (ô chết, freeze deadlock)