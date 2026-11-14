#!/usr/bin/env python3
# ==============================================================================
# Script đo đạc và so sánh hiệu năng các thuật toán lập kế hoạch đường đi (Nav2)
# Hỗ trợ: Navfn (Dijkstra, A*), SmacPlanner2D, ThetaStar
# ==============================================================================

import csv
import math
import os
import select
import sys
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

DEFAULT_CSV = os.path.expanduser(
    '~/dev_ws/src/linorobot2/linorobot2_navigation/planner_comparison_records.csv')
CSV_FIELDS = ['timestamp', 'planner', 'start_x', 'start_y', 'goal_x', 'goal_y',
              'planned_length_m', 'avg_cost', 'max_cost', 'actual_time_s', 'actual_odom_m']

ODOM_MIN_STEP = 0.01    # m: chỉ cộng quãng đường khi xe dịch chuyển tối thiểu từng này
GOAL_TOL = 0.40         # m: coi là đã tới đích
STOP_SPEED = 0.05       # m/s: coi là đã dừng
RUN_TIMEOUT_S = 180.0   # s (giờ mô phỏng): quá thời gian này thì hủy lần chạy
COOLDOWN_S = 5.0        # s: bỏ qua /plan lặp lại của goal vừa xong

STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 4, 5, 6


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def num_or_none(v, nd):
    return None if math.isnan(v) else round(v, nd)


def cell(v, nd):
    return '-' if math.isnan(v) else f'{v:.{nd}f}'


def resample_xy(xy, step):
    """Lấy mẫu lại đường đi đều nhau theo khoảng cách."""
    if len(xy) < 2:
        return xy
    d = np.diff(xy, axis=0)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
    total = s[-1]
    if total < 1e-9:
        return xy[:1]
    n = max(int(math.ceil(total / step)), 1)
    si = np.linspace(0.0, total, n + 1)
    return np.column_stack([np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])])


class InteractiveSinglePlannerBenchmark(Node):
    def __init__(self, initial_planner='NavfnDijkstra', csv_path=DEFAULT_CSV):
        super().__init__(
            'interactive_planner_benchmark',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.active_planner = initial_planner
        self.detected_planner = None
        self.planner_map = {
            '1': 'NavfnDijkstra',
            '2': 'NavfnAStar',
            '3': 'Smac2D',
            '4': 'ThetaStar',
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        costmap_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap = None
        self.costmap_info = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.costmap_cb, costmap_qos)

        self.latest_odom = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.amcl_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

        self.path_pub = self.create_publisher(Path, '/benchmark_path', 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.nav_status_cb, 10)
        self.params_client = self.create_client(
            GetParameters, '/planner_server/get_parameters')

        self.last_status = []          # [(goal_id_bytes, status)]
        self.is_tracking = False
        self.current_run = None
        self.cooldown_until = 0.0
        self.last_goal = None
        self.lock = threading.RLock()

        self.csv_file_path = csv_path
        self.history_records = []
        self.load_history_from_csv()

    # ------------------------------------------------------------------ tiện ích
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def costmap_cb(self, msg):
        self.costmap_info = msg.info
        self.costmap = msg.data

    def amcl_cb(self, msg):
        self.amcl_pose = msg.pose.pose

    def get_robot_pose(self):
        """Vị trí xe trong khung map."""
        for frame in ('base_footprint', 'base_link'):
            try:
                t = self.tf_buffer.lookup_transform('map', frame, Time())
                return t.transform.translation.x, t.transform.translation.y, t.transform.rotation
            except Exception:
                continue
        if self.amcl_pose is not None:
            p = self.amcl_pose
            return p.position.x, p.position.y, p.orientation
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose
            return p.position.x, p.position.y, p.orientation
        return 0.0, 0.0, None

    def get_cost_at(self, x, y):
        """Cost (thang 0-100) tại điểm (x, y). Trả về None nếu không có dữ liệu."""
        if self.costmap is None or self.costmap_info is None or len(self.costmap) == 0:
            return None
        info = self.costmap_info
        mx = int(math.floor((x - info.origin.position.x) / info.resolution))
        my = int(math.floor((y - info.origin.position.y) / info.resolution))
        if 0 <= mx < info.width and 0 <= my < info.height:
            val = self.costmap[my * info.width + mx]
            if val < 0:          # -1 = chưa biết
                return None
            return float(val)
        return None

    # ------------------------------------------------------------------ nhãn planner
    def refresh_planner_label(self):
        """Hỏi planner_server đang nạp planner nào (không chặn)."""
        try:
            if not self.params_client.service_is_ready():
                return
            req = GetParameters.Request()
            req.names = ['planner_plugins']
            self.params_client.call_async(req).add_done_callback(self._on_planner_param)
        except Exception:
            pass

    def _on_planner_param(self, fut):
        try:
            v = fut.result().values[0]
            if v.type == ParameterType.PARAMETER_STRING_ARRAY:
                ids = list(v.string_array_value)
                known = set(self.planner_map.values())
                self.detected_planner = ids[0] if len(ids) == 1 and ids[0] in known else None
        except Exception:
            pass

    def resolve_label(self):
        if self.detected_planner and self.detected_planner != self.active_planner:
            print(f'\n⚠ Nhãn gõ tay [{self.active_planner}] khác planner đang chạy '
                  f'[{self.detected_planner}] -> dùng [{self.detected_planner}]')
            self.active_planner = self.detected_planner
        return self.active_planner

    # ------------------------------------------------------------------ callbacks
    def goal_pose_cb(self, msg):
        gx, gy = msg.pose.position.x, msg.pose.position.y
        rx, ry, _ = self.get_robot_pose()
        print(f'\n📍 [NHẬN GOAL MỚI TỪ RVIZ]: Start ({rx:.2f}, {ry:.2f})  -->  Goal ({gx:.2f}, {gy:.2f})')

    def odom_cb(self, msg):
        self.latest_odom = msg
        with self.lock:
            if not (self.is_tracking and self.current_run):
                return
            run = self.current_run
            cx = msg.pose.pose.position.x
            cy = msg.pose.pose.position.y

            # Khởi tạo điểm bắt đầu odom nếu chưa có mốc
            if run['last_odom_x'] is None or run['last_odom_y'] is None:
                run['last_odom_x'] = cx
                run['last_odom_y'] = cy
                return

            # Chỉ cộng khi dịch chuyển đủ lớn để nhiễu không bị cộng dồn
            step = math.hypot(cx - run['last_odom_x'], cy - run['last_odom_y'])
            if step >= ODOM_MIN_STEP:
                run['actual_distance'] += step
                run['last_odom_x'], run['last_odom_y'] = cx, cy

            elapsed = self.now_s() - run['start_time']
            if elapsed > RUN_TIMEOUT_S:
                print('\n⏱ Quá thời gian cho phép, hủy lần chạy này.')
                self.finish_tracking_run(success=False)
                return

            # Kiểm tra tới đích bằng vị trí trong khung map (goal cũng ở khung map)
            mx, my, _ = self.get_robot_pose()
            dist_to_goal = math.hypot(run['goal_x'] - mx, run['goal_y'] - my)
            v_lin = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if elapsed > 1.5 and dist_to_goal < GOAL_TOL and v_lin < STOP_SPEED:
                run['stopped_frames'] += 1
                if run['stopped_frames'] >= 5:
                    self.finish_tracking_run(success=True)
            else:
                run['stopped_frames'] = 0

    def nav_status_cb(self, msg):
        statuses = [(bytes(s.goal_info.goal_id.uuid), s.status) for s in msg.status_list]
        with self.lock:
            self.last_status = statuses
            if not (self.is_tracking and self.current_run):
                return
            run = self.current_run
            elapsed = self.now_s() - run['start_time']
            for gid, st in statuses:
                if gid in run['stale_ids']:      # goal đã kết thúc từ trước, bỏ qua
                    continue
                if st == STATUS_SUCCEEDED:
                    self.finish_tracking_run(success=True)
                    return
                if st in (STATUS_CANCELED, STATUS_ABORTED) and elapsed > 1.0:
                    self.finish_tracking_run(success=False)
                    return

    def plan_cb(self, msg):
        if msg is None or len(msg.poses) < 2:
            return
        with self.lock:
            if self.is_tracking:       # đang chạy: đây là replan, không reset
                return

            gx = msg.poses[-1].pose.position.x
            gy = msg.poses[-1].pose.position.y
            if (self.last_goal is not None and self.now_s() < self.cooldown_until and
                    math.hypot(gx - self.last_goal[0], gy - self.last_goal[1]) < 0.15):
                return                 # replan muộn của goal vừa xong

            self.refresh_planner_label()
            rx, ry, _ = self.get_robot_pose()

            # 1. Độ dài đường đi (trên các điểm gốc)
            xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)
            d = np.diff(xy, axis=0)
            planned_length = float(np.sum(np.hypot(d[:, 0], d[:, 1])))

            # 2. Cost: lấy mẫu lại đều theo độ phân giải costmap
            res = self.costmap_info.resolution if self.costmap_info else 0.05
            costs, missing = [], 0
            for px, py in resample_xy(xy, res):
                c = self.get_cost_at(float(px), float(py))
                if c is None:
                    missing += 1
                else:
                    costs.append(c)
            if costs:
                avg_cost, max_cost = float(np.mean(costs)), float(np.max(costs))
            else:
                avg_cost = max_cost = float('nan')

            # Khởi tạo mốc odom ban đầu một cách an toàn
            if self.latest_odom is not None:
                odom_x = self.latest_odom.pose.pose.position.x
                odom_y = self.latest_odom.pose.pose.position.y
            else:
                odom_x = None
                odom_y = None

            stale = {gid for gid, st in self.last_status
                     if st in (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED)}

            self.current_run = {
                'start_x': rx, 'start_y': ry, 'goal_x': gx, 'goal_y': gy,
                'planned_length': planned_length,
                'avg_cost': avg_cost, 'max_cost': max_cost,
                'start_time': self.now_s(),
                'last_odom_x': odom_x, 'last_odom_y': odom_y,
                'actual_distance': 0.0, 'stopped_frames': 0,
                'stale_ids': stale,
            }
            self.is_tracking = True
            self.path_pub.publish(msg)

            print('\n' + '=' * 80)
            print(f'🚀 BẮT ĐẦU THEO DÕI XE CHẠY VỚI THUẬT TOÁN: [{self.active_planner}]')
            print(f'   • Vị trí xuất phát  : ({rx:6.2f}, {ry:6.2f})')
            print(f'   • Điểm đích đến     : ({gx:6.2f}, {gy:6.2f})')
            print(f'   • Độ dài đường đi   : {planned_length:6.3f} m')
            if self.costmap is None:
                print('   • Chi phí Costmap   : ⚠ CHƯA NHẬN ĐƯỢC COSTMAP -> không có số liệu cost')
            else:
                print(f'   • Chi phí Costmap   : Avg = {cell(avg_cost, 1)} | Max = {cell(max_cost, 1)}'
                      f'  (thang 0-100, 99 = sát vật cản)')
                if missing:
                    print(f'   • Lưu ý             : {missing} điểm nằm ngoài bản đồ hoặc vùng chưa biết, đã bỏ qua')
            print('   • Xe đang lăn bánh trên Gazebo... Vui lòng đợi xe đến đích!')
            print('=' * 80)

    # ------------------------------------------------------------------ kết thúc lần chạy
    def finish_tracking_run(self, success=True):
        with self.lock:
            if not self.is_tracking or not self.current_run:
                return
            run = self.current_run
            planner = self.resolve_label()
            drive_duration = self.now_s() - run['start_time']
            planned_len = run['planned_length']
            avg_cost, max_cost = run['avg_cost'], run['max_cost']
            actual_dist = run['actual_distance']
            status_str = 'THÀNH CÔNG' if success else 'DỪNG/HỦY'

            print('\n' + '=' * 80)
            print(f'🎯 [HOÀN THÀNH LẦN CHẠY CỦA: {planner} - {status_str}]')
            print('=' * 80)
            print(f'  1. Thuật toán đánh giá                      : {planner}')
            print(f'  2. Độ dài đường đi hoạch định (Path Length) : {planned_len:6.3f} m')
            print(f'  3. Cost trung bình (Average Cost, 0-100)    : {cell(avg_cost, 2):>6}')
            print(f'  4. Cost lớn nhất (Max Cost, 0-100)          : {cell(max_cost, 1):>6}')
            print(f'  5. Thời gian xe chạy (giờ mô phỏng)         : {drive_duration:6.2f} s')
            print(f'  6. Quãng đường thực tế lăn bánh (Odom Dist) : {actual_dist:6.3f} m')
            print('=' * 80)

            record = {
                'timestamp': time.strftime('%H:%M:%S'),
                'planner': planner,
                'start_x': round(run['start_x'], 2), 'start_y': round(run['start_y'], 2),
                'goal_x': round(run['goal_x'], 2), 'goal_y': round(run['goal_y'], 2),
                'planned_length_m': round(planned_len, 3),
                'avg_cost': num_or_none(avg_cost, 2),
                'max_cost': num_or_none(max_cost, 1),
                'actual_time_s': round(drive_duration, 2),
                'actual_odom_m': round(actual_dist, 3),
            }
            self.history_records.append(record)
            self.save_record_to_csv(record)
            self.print_comparison_summary()

            self.last_goal = (run['goal_x'], run['goal_y'])
            self.cooldown_until = self.now_s() + COOLDOWN_S
            self.is_tracking = False
            self.current_run = None

            print('\n' + '-' * 80)
            print(f'👉 Hiện tại đang gắn nhãn thuật toán: [{self.active_planner}].')
            print('   • Chạy tiếp với thuật toán này: sang RViz và chấm Goal tiếp theo.')
            print('   • Nếu đã đổi thuật toán trong config: gõ [1/2/3/4] rồi bấm Enter:')
            print('       [1] NavfnDijkstra | [2] NavfnAStar | [3] Smac2D | [4] ThetaStar')
            print('-' * 80 + '\n')

    # ------------------------------------------------------------------ CSV
    def save_record_to_csv(self, record):
        path = self.csv_file_path
        try:
            has_data = os.path.exists(path) and os.path.getsize(path) > 0
            fields = CSV_FIELDS
            if has_data:
                with open(path, newline='', encoding='utf-8') as f:
                    header = next(csv.reader(f), None)
                if header:
                    fields = header          # giữ đúng cấu trúc file cũ
            with open(path, mode='a', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
                if not has_data:
                    w.writeheader()
                w.writerow(record)
            print(f'💾 Đã lưu dữ liệu vào: {path}')
        except Exception as e:
            self.get_logger().warn(f'Không thể ghi file CSV: {e}')

    def load_history_from_csv(self):
        if os.path.exists(self.csv_file_path):
            try:
                with open(self.csv_file_path, mode='r', encoding='utf-8') as f:
                    self.history_records = [r for r in csv.DictReader(f) if r.get('planner')]
            except Exception:
                pass

    # ------------------------------------------------------------------ bảng tổng hợp
    def print_comparison_summary(self):
        if not self.history_records:
            return
        line = '|' + '-' * 6 + '|' + '-' * 16 + '|' + '-' * 15 + '|' + '-' * 12 + '|' + \
               '-' * 11 + '|' + '-' * 15 + '|' + '-' * 12 + '|'
        head = (f"| {'Lần':<4} | {'Thuật toán':<14} | {'Path Len (m)':<13} | {'Avg Cost':<10} | "
                f"{'Max Cost':<9} | {'Run Time (s)':<13} | {'Odom (m)':<10} |")

        print('\n' + '#' * 88)
        print(f"{'BẢNG TỔNG HỢP SO SÁNH CÁC LẦN CHẠY (BENCHMARK RESULTS)':^88}")
        print('#' * 88)
        print(head)
        print(line)
        for idx, r in enumerate(self.history_records):
            p_name = r.get('planner', 'Unknown')
            v = [to_float(r.get(k)) for k in
                 ('planned_length_m', 'avg_cost', 'max_cost', 'actual_time_s', 'actual_odom_m')]
            print(f"| {idx + 1:<4} | {p_name:<14} | {cell(v[0], 3):^13} | {cell(v[1], 2):^10} | "
                  f"{cell(v[2], 1):^9} | {cell(v[3], 2):^13} | {cell(v[4], 3):^10} |")
        print('#' * 88)

        groups = {}
        for r in self.history_records:
            p_name = r.get('planner', 'Unknown')
            groups.setdefault(p_name, []).append(r)
        print(f"\n{'TRUNG BÌNH THEO THUẬT TOÁN':^88}")
        print(head.replace('Lần', 'Số '))
        print(line)
        for name, rs in groups.items():
            means = []
            for k in ('planned_length_m', 'avg_cost', 'max_cost', 'actual_time_s', 'actual_odom_m'):
                a = np.array([to_float(x.get(k)) for x in rs], dtype=float)
                a = a[~np.isnan(a)]
                means.append(float(a.mean()) if a.size else float('nan'))
            print(f"| {len(rs):<4} | {name:<14} | {cell(means[0], 3):^13} | {cell(means[1], 2):^10} | "
                  f"{cell(means[2], 1):^9} | {cell(means[3], 2):^13} | {cell(means[4], 3):^10} |")
        print('(Ô trống "-" nghĩa là không có số liệu hợp lệ, không phải bằng 0)\n')


def user_input_loop(node):
    """Đọc lệnh từ terminal để đổi nhãn thuật toán mà không chặn ROS."""
    while rclpy.ok():
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not ready:
                continue
            line = sys.stdin.readline()
            if line == '':
                break
            cmd = line.strip()
            if cmd in node.planner_map:
                node.active_planner = node.planner_map[cmd]
                print(f'\n🔄 [ĐÃ GẮN NHÃN THUẬT TOÁN MỚI] --> [{node.active_planner}]')
                print('   Hãy đảm bảo đã mở comment thuật toán này trong navigation_sim.yaml,')
                print('   sau đó sang RViz chấm Goal để thực hiện lần chạy mới!\n')
            elif cmd in ('p', 'print'):
                node.print_comparison_summary()
            elif cmd in ('q', 'quit', 'exit'):
                break
        except KeyboardInterrupt:
            break
        except Exception:
            pass


def main():
    rclpy.init()

    print('\n' + '=' * 70)
    print('      CHƯƠNG TRÌNH SO SÁNH TỪNG THUẬT TOÁN ĐIỀU HƯỚNG NAV2')
    print('=' * 70)
    print('Chọn thuật toán hiện tại bạn đang mở comment trong navigation_sim.yaml:')
    print('  [1] NavfnDijkstra (NavfnPlanner - Dijkstra) [Mặc định]')
    print('  [2] NavfnAStar    (NavfnPlanner - A*)')
    print('  [3] Smac2D        (SmacPlanner2D - A* trên lưới 2D)')
    print('  [4] ThetaStar     (Theta* Planner - Any-angle)')
    print('=' * 70)

    names = {'1': 'NavfnDijkstra', '2': 'NavfnAStar', '3': 'Smac2D', '4': 'ThetaStar'}
    choice = input('👉 Nhập số (1/2/3/4) [Mặc định 1]: ').strip()
    node = InteractiveSinglePlannerBenchmark(initial_planner=names.get(choice, 'NavfnDijkstra'))

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print('\n' + '=' * 75)
    print(f'✔ ĐÃ SẴN SÀNG THEO DÕI THUẬT TOÁN: [{node.active_planner}]')
    print('👉 HƯỚNG DẪN:')
    print("   1. Sang RViz, bấm 'Nav2 Goal' hoặc '2D Goal Pose' và chấm đích.")
    print('   2. Xe chạy trên Gazebo, chương trình tự đo Path Length, Cost, Time, Odom.')
    print('   3. Khi xe tới đích, kết quả được in ra và lưu vào CSV.')
    print('   4. Đổi thuật toán (sửa yaml + restart Nav2): gõ 1/2/3/4 rồi Enter để đổi nhãn.')
    print("      Gõ 'p' để in lại bảng, 'q' để thoát.")
    print('=' * 75 + '\n')

    try:
        user_input_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
