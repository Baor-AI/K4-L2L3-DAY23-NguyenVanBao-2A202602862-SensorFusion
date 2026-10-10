# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Văn Bảo
- MSSV: 2A202602862
- Email: ngodoanngoc60@gmail.com
- Link repo (fork): https://github.com/Baor-AI/K4-L2L3-DAY23-NguyenVanBao-2A202602862-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`:
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`:
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?**
   - LiDAR: `z = (x, y, z)` tâm hộp 3D (mét), `R = diag(0.1², 0.1², 0.1²)`. `h(x) = R_vs·p + t`
     là tuyến tính nên `H` là ma trận 3×6 hằng (khối xoay + 3 cột vận tốc bằng 0) — đây là KF thường.
   - Camera: `z = (u, v)` pixel, `R = diag(5², 5²)` (`build_camera_measurement`, `camera_fusion.py`).
     `h(x)` là phép chiếu pinhole `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s` — phi tuyến,
     nên EKF phải tuyến tính hoá bằng Jacobian 2×6 tính tại trạng thái dự báo. Camera không đo
     trực tiếp độ sâu: một đo 2D chỉ ràng buộc hướng nhìn, nên nó chủ yếu tinh chỉnh vị trí ngang/dọc,
     còn độ sâu vẫn do LiDAR quyết định.
2. **Vì sao cần gating Mahalanobis trước khi gán?**
   `d² = γᵀ S⁻¹ γ` (`mahalanobis_distance`, `association.py`) chuẩn hoá residual theo độ bất định
   `S = H P Hᵀ + R`, nên so sánh được giữa track mới (P lớn) và track ổn định (P nhỏ), và giữa
   mét với pixel. Cổng `χ²(0.995, dim_meas)` (12.84 với LiDAR 3D, 10.60 với camera 2D) loại các
   cặp không thể khớp về mặt thống kê, tránh để greedy gán một detection ở xa / false positive vào
   track, làm hỏng trạng thái EKF và đổi ID. Đo bị loại khỏi mọi track sẽ thành ứng viên khởi tạo track mới.
3. **Track-then-fuse hay fuse-then-track?** Track-then-fuse. Chỉ có một `track_list`; trong
   `platform/fusion_lab/scripts/run_lab.py` mỗi frame: `KF.predict(track)` cho mọi track (một lần),
   rồi `associate_and_update(manager, lidar_obs, KF, lidar_sensor)` (AssocL), rồi nếu mode `fused`
   mới gọi `associate_and_update(..., camera_sensor)` (AssocC), cuối cùng ghi một record JSONL.
   Trên log: `grade_run_lidar.log` và `grade_run_fused.log` có **cùng** `det_tp/det_fp/det_fn`
   ở mọi frame (detector LiDAR giống hệt), chỉ khác `confirmed/matches/sum_sq_err/ghosts/misses`
   — tức camera không đi vào bước detection mà chỉ cập nhật trạng thái track đã có.
4. **Camera lệch calibration → triệu chứng?** Innovation camera `γ = z − h(x)` không còn trung bình
   0 mà có **độ lệch hệ thống** (bias) cùng dấu cho mọi track (ví dụ lệch yaw → `u` lệch đều sang
   một phía, lệch tịnh tiến → bias phụ thuộc 1/độ sâu). `d²` trung bình tăng vượt `dim_meas = 2`;
   lệch nhỏ thì vẫn lọt cổng và kéo trạng thái lệch đi (RMSE fused > lidar, LiDAR và camera
   "giằng co" làm residual LiDAR cũng tăng); lệch lớn thì bị cổng χ² chặn → nhiều track không được
   cập nhật camera (fused tiến về lidar-only).
5. **Vì sao cần `sensor` tường minh ở frame rỗng? Vì sao chỉ LiDAR quyết định tồn tại?**
   Khi `meas_list` rỗng không có `meas.sensor` để biết đây là lượt nào, nhưng `manage_tracks` vẫn
   phải chạy: với lượt LiDAR, mọi track trong FOV bị tính **miss** (giảm score) và có thể bị xoá —
   nếu bỏ qua frame rỗng thì ghost track sống mãi. Với lượt camera, cùng lời gọi đó phải là no-op.
   `associate_and_update` luôn kết thúc bằng `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`.
   LiDAR quyết định score/init/delete vì nó cho vị trí 3D đầy đủ và phủ 360°; camera chỉ thấy FOV
   phía trước, không có độ sâu (không khởi tạo được state 3D), và trong lab đo camera là nhãn GT có
   nhiễu — nếu cho camera cộng score sẽ "nuôi" ghost track và làm metric bị lệch theo GT.
   Vì vậy `handle_updated_track` / `manage_tracks` chỉ hành động khi `sensor.name == "lidar"`.
6. **Điều kiện xác nhận / giữ confirmed / xoá** (`track_management.py`, `window = 6`):
   - Khởi tạo từ đo LiDAR chưa gán: `score = 1/6`, `state = initialized`, vận tốc 0 với
     `σ_v = (50, 50, 5)`.
   - Mỗi frame LiDAR: hit `+1/6` (tối đa 1), miss trong FOV `−1/6`. Unconfirmed được hit → `tentative`.
   - **Xác nhận** khi `score > 0.8`: khởi tạo 1/6 + 4 hit liên tiếp = 5/6 ≈ 0.833 > 0.8 (4/6 thì chưa).
   - **Đã confirmed thì không hạ trạng thái** khi miss (score 1 → 5/6 vẫn confirmed).
   - **Xoá** (OR): `P[0,0] > 9` hoặc `P[1,1] > 9` (σ > 3 m) bất kể score; confirmed và
     `score < 0.6`; unconfirmed và `score ≤ 0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Claude Opus 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): Viết code Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`), cài môi trường, chạy pipeline, soạn nháp phần giải thích 6 câu.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` (128 passed), đối chiếu công thức EKF/pinhole/χ² với docs/HUONG_DAN_KY_THUAT.md, chạy `fusion-run-lab --fusion compare --seed 0`.

## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
