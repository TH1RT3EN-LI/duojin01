#!/usr/bin/env python3
"""
相机标定工具
============
使用棋盘格图案（chessboard）一键完成相机内参标定，
生成可供 usb_camera_node 和 AprilTagDetector 使用的 YAML 文件。

用法::

    ros2 run duojin01_camera calibrate_camera \\
        --device 2 \\
        --cols 9 --rows 6 \\
        --square 0.025 \\
        --output ~/calibration.yaml

操作说明：
    SPACE  ── 当前帧检测到棋盘格时，捕获该帧
    c      ── 已采集足够帧时立即开始计算
    q/ESC  ── 退出

棋盘格打印建议：
    默认 9×6 内角点（即 10×7 方格）。
    方格边长默认 25 mm，请按实际打印尺寸修改 --square 参数。
    打印后贴在平整硬板上。
"""

import argparse
import sys
import os
import threading
import time
import cv2
import numpy as np
import yaml


# ── 参数解析 ──────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='USB 相机标定工具（棋盘格）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--device',  type=int,   default=2,
                   help='摄像头设备编号（/dev/videoN 的 N）')
    p.add_argument('--cols',    type=int,   default=9,
                   help='棋盘格内角点列数')
    p.add_argument('--rows',    type=int,   default=6,
                   help='棋盘格内角点行数')
    p.add_argument('--square',  type=float, default=0.025,
                   help='方格边长，单位：米')
    p.add_argument('--min-frames', type=int, default=20,
                   help='计算标定所需的最少采集帧数')
    p.add_argument('--width',   type=int,   default=1280,
                   help='采集分辨率宽度')
    p.add_argument('--height',  type=int,   default=720,
                   help='采集分辨率高度')
    p.add_argument('--output',  type=str,
                   default=os.path.expanduser('~/calibration.yaml'),
                   help='标定结果保存路径')
    return p.parse_args()


# ── 后台抓帧线程 ──────────────────────────────────────────────────────────

class CameraReader:
    """独立线程持续读取摄像头，始终保存最新帧，避免主线程阻塞。"""

    def __init__(self, cap: cv2.VideoCapture):
        self._cap    = cap
        self._frame  = None
        self._lock   = threading.Lock()
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def read(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


# ── 后台棋盘检测线程 ──────────────────────────────────────────────────────

class ChessboardDetector:
    """独立线程对最新帧做棋盘格检测（耗时操作不阻塞显示）。

    结果结构：{'frame': ndarray, 'gray': ndarray,
               'found': bool, 'corners': ndarray|None}
    """

    def __init__(self, board_size: tuple, criteria):
        self._board_size = board_size
        self._criteria   = criteria
        self._input      = None   # 待检测帧
        self._result     = None   # 最新检测结果
        self._lock_in    = threading.Lock()
        self._lock_out   = threading.Lock()
        self._event      = threading.Event()
        self._stop       = threading.Event()
        self._thread     = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame: np.ndarray):
        """提交新帧（丢弃尚未处理的旧帧，始终处理最新帧）。"""
        with self._lock_in:
            self._input = frame
        self._event.set()

    def _run(self):
        while not self._stop.is_set():
            triggered = self._event.wait(timeout=0.1)
            if not triggered:
                continue
            self._event.clear()

            with self._lock_in:
                frame = self._input
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, self._board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self._criteria
                )
            with self._lock_out:
                self._result = {
                    'frame': frame,
                    'gray':  gray,
                    'found': found,
                    'corners': corners if found else None,
                }

    def result(self):
        """返回最新检测结果（dict），尚无结果时返回 None。"""
        with self._lock_out:
            return self._result

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


# ── 标定核心 ──────────────────────────────────────────────────────────────

def calibrate(obj_pts, img_pts, img_shape):
    """运行 OpenCV 标定，返回 (camera_matrix, dist_coeffs, rms_error)。"""
    flags = cv2.CALIB_RATIONAL_MODEL
    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_pts, img_pts, img_shape, None, None, flags=flags
    )
    return K, dist, rms


def save_yaml(path: str, K: np.ndarray, dist: np.ndarray,
              width: int, height: int) -> None:
    data = {
        'image_width':  width,
        'image_height': height,
        'camera_name':  'usb_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': K.flatten().tolist(),
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': int(dist.size),
            'data': dist.flatten().tolist(),
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0],
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': [
                float(K[0, 0]), 0.0,            float(K[0, 2]), 0.0,
                0.0,            float(K[1, 1]), float(K[1, 2]), 0.0,
                0.0,            0.0,            1.0,            0.0,
            ],
        },
    }
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


# ── 主程序 ────────────────────────────────────────────────────────────────

WIN = '相机标定工具 - Camera Calibration'


def main():
    args = parse_args()

    board_size  = (args.cols, args.rows)
    square_size = args.square
    min_frames  = args.min_frames

    # 世界坐标系下棋盘格角点（z=0 平面）
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_pts = []   # 3D 点（世界坐标）
    img_pts = []   # 2D 点（图像坐标）

    # ── 打开摄像头 ──
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS,          30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # 减少缓冲，降低延迟

    if not cap.isOpened():
        print(f'[ERROR] 无法打开摄像头 /dev/video{args.device}')
        sys.exit(1)

    print(f'\n棋盘格规格：{board_size[0]}×{board_size[1]} 内角点，'
          f'方格边长 {square_size*1000:.0f} mm')
    print(f'目标采集帧数：{min_frames}')
    print('操作：SPACE=采集  c=立即计算  q/ESC=退出\n')
    print('[INFO] 正在连接摄像头，请稍候...')

    criteria   = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    reader     = CameraReader(cap)
    detector   = ChessboardDetector(board_size, criteria)
    img_shape  = None
    last_frame = None

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 640, 360)

    wait_start = time.time()

    while True:
        # ── 取最新原始帧，提交给检测线程 ──
        new_frame = reader.read()
        if new_frame is not None:
            last_frame = new_frame
            detector.submit(new_frame)

        # ── 尚未收到有效帧：显示等待画面 ──
        if last_frame is None:
            elapsed = time.time() - wait_start
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, f'等待摄像头画面... {elapsed:.1f}s',
                        (40, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)
            cv2.imshow(WIN, blank)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                print('[INFO] 用户退出。')
                break
            if elapsed > 10.0:
                print('[ERROR] 摄像头超过 10 秒未返回画面，请检查设备。')
                break
            continue

        # ── 取检测线程最新结果（可能比当前帧稍旧，不影响流畅度）──
        det    = detector.result()
        found   = det['found']   if det else False
        corners = det['corners'] if det else None

        # 显示最新原始帧（保证流畅），在其上叠加检测结果
        display = last_frame.copy()
        if det is not None:
            img_shape = det['gray'].shape[::-1]
        n = len(obj_pts)

        if found and corners is not None:
            cv2.drawChessboardCorners(display, board_size, corners, True)
            status_color = (0, 220, 0)
            status_text  = f'Board found  {n}/{min_frames} captured  SPACE to save'
        else:
            status_color = (0, 100, 255)
            status_text  = f'No board detected  {n}/{min_frames} captured'

        cv2.putText(display, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        if n >= min_frames:
            cv2.putText(display, 'Enough frames! Press c to calibrate',
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 200), 2)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):
            print('[INFO] 用户退出，未保存标定结果。')
            break

        if key == ord(' '):
            # 按空格时对当前帧做一次同步检测，结果即为用户意图捕获的那一帧
            snap = last_frame.copy()
            snap_gray = cv2.cvtColor(snap, cv2.COLOR_BGR2GRAY)
            snap_found, snap_corners = cv2.findChessboardCorners(
                snap_gray, board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if snap_found:
                snap_corners = cv2.cornerSubPix(
                    snap_gray, snap_corners, (11, 11), (-1, -1), criteria
                )
                img_shape = snap_gray.shape[::-1]
                obj_pts.append(objp)
                img_pts.append(snap_corners)
                print(f'  Frame {len(obj_pts):>2} captured')
            else:
                print('[WARN] No chessboard in current frame, please aim at the board')

        if key == ord('c'):
            if len(obj_pts) < 10:
                print(f'[WARN] 采集帧数太少（{len(obj_pts)}），建议至少 10 帧')
                continue
            print(f'\n[INFO] 开始计算标定（共 {len(obj_pts)} 帧）...')
            K, dist, rms = calibrate(obj_pts, img_pts, img_shape)

            print('\n===== 标定结果 =====')
            print(f'重投影误差（RMS）: {rms:.4f} px  （< 0.5 为优秀）')
            print(f'fx={K[0,0]:.2f}  fy={K[1,1]:.2f}')
            print(f'cx={K[0,2]:.2f}  cy={K[1,2]:.2f}')
            print(f'畸变系数: {dist.flatten().round(6).tolist()}')

            w, h = img_shape
            save_yaml(args.output, K, dist, w, h)
            print(f'\n[OK] 标定文件已保存: {args.output}')
            break

    detector.stop()
    reader.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
