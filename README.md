# AGV tự hành dò line và tránh vật cản trên ESP32 (MicroPython)

Robot AGV cỡ nhỏ bám theo đường kẻ đen bằng mảng 5 cảm biến hồng ngoại và tránh vật cản phía trước bằng cảm biến siêu âm HC-SR04. Toàn bộ chương trình viết bằng MicroPython, chạy trên ESP32. Đồ án môn học, Trường Đại học Công nghệ, ĐHQGHN.

## Chức năng

- **Bám line:** sai số vị trí là trung bình có trọng số của 5 cảm biến, đưa vào bộ điều khiển PD có lọc đạo hàm (chu kỳ vòng điều khiển 10 ms).
- **Mất line:** nếu line lệch về một bên, robot quay về phía đó tối đa 3000 ms rồi dừng; nếu line biến mất khi đang ở giữa, robot đi thẳng tối đa 1000 ms rồi dừng. Cả 5 cảm biến cùng tắt hoặc cùng bật quá 3000 ms thì robot dừng.
- **Tránh vật cản:** khi vật cản cách không quá 20 cm (trung vị của 3 mẫu, 2 lần liên tiếp) và có 1–4 cảm biến thấy line, robot thực hiện đường vòng 8 bước theo thời gian rồi quay lại line.
- **Chế độ thử:** `sensors`, `motors`, `sonar`, `turn`, `forward` để kiểm tra từng khối.
- Robot không có encoder và không dùng Wi-Fi; chế độ tự động không in dữ liệu ra cổng nối tiếp.

## Phần cứng

- ESP32 (bo phát triển 30 chân) gắn trên bo mở rộng loại phổ thông
- Mạch điều khiển động cơ TB6612FNG
- 2 động cơ DC loại phổ thông (hai bánh gắn motor và một bánh caster)
- Mảng 5 cảm biến hồng ngoại TCRT5000, cảm biến siêu âm HC-SR04
- Nguồn: 2 pin Li-ion 18650 nối tiếp (6,0–8,4 V)

## Nối chân

| Khối | Tín hiệu | Chân |
|---|---|---|
| Động cơ trái (kênh A) | PWMA / AIN1 / AIN2 | GPIO13 / GPIO12 / GPIO14 |
| Động cơ phải (kênh B) | PWMB / BIN1 / BIN2 | GPIO15 / GPIO4 / GPIO2 |
| Mảng IR (trái → phải) | OUT1 … OUT5 | GPIO34, 32, 33, 25, 27 (mức LOW = trên line) |
| HC-SR04 | Trig / Echo | GPIO21 / GPIO19 |
| TB6612FNG | VCC, STBY | 3,3 V từ bo mở rộng |
| TB6612FNG | VM, GND | Cực + / cực − của pin |

Mảng IR và HC-SR04 được cấp 3,3 V từ bo mở rộng. Các khối dùng chung GND. Sơ đồ đấu nối nằm trong thư mục `diagrams/`.

## Tham số chính (v3.3)

| Tham số | Giá trị |
|---|---|
| KP / KI / KD | 30 / 0 / 0,7 |
| D_SMOOTH | 0,6 |
| BASE_PWM | 150 |
| TURN_PWM / TURN_RATE_DEG_S | 120 / 225 °/s |
| FWD_PWM / FWD_SPEED_CM_S | 110 / 18 cm/s |
| Đường vòng | quay trái 70°, đi 23 cm, quay phải 60°, đi 28 cm, quay phải 60°, đi tới khi thấy line (tối đa 45 cm), quay trái 30° |

Danh sách đầy đủ nằm ở đầu file mã nguồn.

## Cấu trúc thư mục

```
code/        mã nguồn v3.3 (goodworking_AGV_v3_3_Hoang.py)
diagrams/    sơ đồ khối, kiến trúc, đấu nối và lưu đồ
evidence/    ảnh chụp minh chứng các phép thử trên bàn thử
docs/        báo cáo (PDF)
```

Video minh chứng không nằm trong repo mà được lưu trên Google Drive (xem mục Minh chứng).

## Cách chạy

1. Mở `code/goodworking_AGV_v3_3_Hoang.py` trong Thonny, chọn interpreter là MicroPython (ESP32).
2. **Chạy tự động:** để `TEST_MODE = None`, lưu file lên ESP32 với tên `main.py`, rồi cấp nguồn bằng pin. Robot chờ 2 giây rồi tự chạy, hãy đặt robot lên đường kẻ trong thời gian này.
3. **Chạy thử từng khối:** đặt `TEST_MODE` là `'sensors'`, `'motors'`, `'sonar'`, `'turn'` hoặc `'forward'`, nhấn F5 trong Thonny và xem kết quả ở cửa sổ Shell. Đặt lại `TEST_MODE = None` sau khi thử.
4. Khi thử `motors`, phải nhấc bánh xe khỏi mặt đất.

## Kết quả thử nghiệm

Điều kiện: tuyến nhỏ trên sàn, đường kẻ bằng băng dính đen rộng 5 cm, vật cản là hộp 16 × 16 × 16 cm.

| Phép thử | Kết quả |
|---|---|
| Bám line, không vật cản | ĐẠT 10/10 vòng |
| Vòng đầy đủ có vật cản | ĐẠT 20/20 vòng |
| Tốc độ quay đo được (PWM 120) | 225 °/s |
| Tốc độ tiến đo được (PWM 110) | 18 cm/s |

Chi tiết từng phép thử xem báo cáo trong `docs/` và thư mục minh chứng trên Drive.

## Giới hạn

- Đường vòng tránh vật cản tính theo thời gian nên phụ thuộc vào điện áp pin và có thể thay đổi giữa các lần chạy.
- Bước quay đầu tiên của đường vòng không có khoảng nghỉ ổn định; góc quay thực tế quan sát được dao động khoảng 70°–90°.
- Cảm biến siêu âm không thấy vật gần hơn khoảng 2 cm.
- Chưa thử trên đoạn thẳng dài và trên sân khác (độ tương phản, ánh sáng có thể khác).

## Hướng phát triển

Điều khiển từ xa qua Wi-Fi, điều chỉnh tốc độ, và dùng encoder để đo quãng đường và điều khiển vòng kín.

## Minh chứng

- Video và ảnh chụp đầy đủ: https://drive.google.com/drive/folders/17QGdMTQ-vWNX3h6_7SqFHj77o1Ku7Mqy?usp=sharing
- Báo cáo: `docs/Bao_Cao_Nhom7_DKTM.pdf`


## Thành viên

| Họ và tên | Mã SV |
|---|---|
| Nguyễn Hoàng | 24022127 |
| Hà Ngọc Bảo | 24022087 |
| Lê Trọng Tuấn Dũng | 24022103 |
