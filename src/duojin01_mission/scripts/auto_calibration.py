#!/usr/bin/env python3
"""简易串口交互工具，独立于 ROS。
用法: python3 auto_calibration.py [端口] [波特率]
示例: python3 auto_calibration.py /dev/ttyUSB0 115200
"""

import sys
import time
import serial
import cv2
import numpy as np
from datetime import datetime
from pathlib import Path


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.5   # 发送后等待回复的秒数
DEEP = 180
MOVE_DELAY = 3.0
HOME_DELAY = 8.0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

base_dir = Path("/tmp/ros_image")
folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
folder_path = base_dir / folder_name

# "rl_x" 取值[0,60] , "rl_y" 取值[-60,100] "img_x" 默认初始为0 "img_y" 默认初始为0
TEST_XY = [
    {"rl_x": 0,  "rl_y": -60, "img_x": 0, "img_y": 0},
    {"rl_x": 60, "rl_y": -60, "img_x": 0, "img_y": 0},
    {"rl_x": 0,  "rl_y": 100, "img_x": 0, "img_y": 0},
    {"rl_x": 60, "rl_y": 100, "img_x": 0, "img_y": 0},

    {"rl_x": 30, "rl_y": -60, "img_x": 0, "img_y": 0},
    {"rl_x": 30, "rl_y": 100, "img_x": 0, "img_y": 0},
    {"rl_x": 0,  "rl_y": 20,  "img_x": 0, "img_y": 0},
    {"rl_x": 60, "rl_y": 20,  "img_x": 0, "img_y": 0},

    {"rl_x": 30, "rl_y": 20,  "img_x": 0, "img_y": 0},
    {"rl_x": 15, "rl_y": -20, "img_x": 0, "img_y": 0},
    {"rl_x": 45, "rl_y": -20, "img_x": 0, "img_y": 0},
    {"rl_x": 30, "rl_y": 60,  "img_x": 0, "img_y": 0},
]

def send_cmd(ser, cmd, delay):
    print(cmd.encode())
    ser.write(cmd.encode())
    ser.flush()
    time.sleep(delay)


class LinearFitter:
    def __init__(self):
        self.samples = []

    def add_sample(self, center_u, center_v, adj_x, adj_y):
        self.samples.append((center_u, center_v, adj_x, adj_y))
        print(f"已添加样本: best.center_u={center_u}, best.center_v={center_v}, adj_x={adj_x}, adj_y={adj_y}")

    def fit(self):
        if len(self.samples) < 3:
            raise ValueError("样本数量不足，至少需要3个样本")

        center_u = np.array([s[0] for s in self.samples])
        center_v = np.array([s[1] for s in self.samples])
        adj_x = np.array([s[2] for s in self.samples])
        adj_y = np.array([s[3] for s in self.samples])

        X = np.column_stack((center_u, center_v, np.ones_like(center_u)))

        coefficients_x, residuals_x, _, _ = np.linalg.lstsq(X, adj_x, rcond=None)
        a1, b1, c1 = coefficients_x

        coefficients_y, residuals_y, _, _ = np.linalg.lstsq(X, adj_y, rcond=None)
        a2, b2, c2 = coefficients_y

        adj_x_pred = a1 * center_u + b1 * center_v + c1
        adj_y_pred = a2 * center_u + b2 * center_v + c2

        ss_res_x = np.sum((adj_x - adj_x_pred) ** 2)
        ss_res_y = np.sum((adj_y - adj_y_pred) ** 2)
        ss_tot_x = np.sum((adj_x - np.mean(adj_x)) ** 2)
        ss_tot_y = np.sum((adj_y - np.mean(adj_y)) ** 2)
        r2_x = 1 - ss_res_x / ss_tot_x if ss_tot_x != 0 else 0.0
        r2_y = 1 - ss_res_y / ss_tot_y if ss_tot_y != 0 else 0.0

        return {
            'x_coeff': (a1, b1, c1),
            'y_coeff': (a2, b2, c2),
            'r2_x': r2_x,
            'r2_y': r2_y,
            'residuals_x': ss_res_x,
            'residuals_y': ss_res_y,
            'actual': {'adj_x': adj_x, 'adj_y': adj_y},
            'predicted': {'adj_x': adj_x_pred, 'adj_y': adj_y_pred},
        }

    def print_equation(self, results):
        a1, b1, c1 = results['x_coeff']
        a2, b2, c2 = results['y_coeff']

        print("\n拟合得到的线性方程:")
        print(f"adj_x = {a1:.6f} * best.center_u + {b1:.6f} * best.center_v + {c1:.6f}")
        print(f"adj_y = {a2:.6f} * best.center_u + {b2:.6f} * best.center_v + {c2:.6f}")
        print(f"\n拟合优度 R²:")
        print(f"adj_x: {results['r2_x']:.6f} (越接近1越好)")
        print(f"adj_y: {results['r2_y']:.6f} (越接近1越好)")
        print(f"\n残差:")
        print(f"adj_x: {results['residuals_x']:.6f} (越小越好)")
        print(f"adj_y: {results['residuals_y']:.6f} (越小越好)")

    def print_comparison(self, results):
        print("\n预测值与实际值对比:")

        for i in range(len(results['actual']['adj_x'])):
            actual_x = results['actual']['adj_x'][i]
            actual_y = results['actual']['adj_y'][i]
            pred_x = results['predicted']['adj_x'][i]
            pred_y = results['predicted']['adj_y'][i]
            error_x = abs(actual_x - pred_x)
            error_y = abs(actual_y - pred_y)

            print(f"样本{i+1}实际值({actual_x:.6f},{actual_y:.6f})，计算值({pred_x:.6f},{pred_y:.6f}),二者误差({error_x:.6f},{error_y:.6f})")

        avg_error_x = np.mean(np.abs(results['actual']['adj_x'] - results['predicted']['adj_x']))
        avg_error_y = np.mean(np.abs(results['actual']['adj_y'] - results['predicted']['adj_y']))
        print(f"\n平均误差: ({avg_error_x:.6f},{avg_error_y:.6f})")


def fit_samples(samples):
    fitter = LinearFitter()
    for sp in samples:
        if sp["img_x"] == 0 and sp["img_y"] == 0:
            continue
        fitter.add_sample(sp["img_x"], sp["img_y"], sp["rl_x"], sp["rl_y"])

    try:
        results = fitter.fit()
        fitter.print_equation(results)
        fitter.print_comparison(results)
    except ValueError as e:
        print(f"拟合失败: {e}")


def open_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    print(
        f"摄像头实际分辨率: "
        f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
        f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
    )
    return cap


def sampling(ser, cap):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    folder_path.mkdir(parents=True, exist_ok=True)
    index = 1
    for sp in TEST_XY:
        # 移动到制定位置
        move_cmd = f"M20 G91 G00 X{sp['rl_x']} Y{sp['rl_y']} Z-{DEEP}\n"
        send_cmd(ser, move_cmd, MOVE_DELAY)

        # 等待用户input任意字符将物块放入
        input("放入物块后按回车继续采样...")

        # 回到原位
        back_cmd = f"M20 G91 G00 X{-sp['rl_x']} Y{-sp['rl_y']} Z{DEEP}\n"
        send_cmd(ser, back_cmd, MOVE_DELAY)

        cap.release()
        cap = open_camera()
        # 获取一帧
        ret, frame = cap.read()

        # 获取这一帧 AprilTag 坐标，写入TEST_XY的img_x和img_y
        corners, ids, _ = detector.detectMarkers(frame)
        tag_points = corners[0][0]
        center = tag_points.mean(axis=0)
        sp["img_x"] = int(center[0])
        sp["img_y"] = int(center[1])
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.circle(frame, (sp["img_x"], sp["img_y"]), 5, (0, 0, 255), -1)
        image_path = folder_path / f"{index}.jpg"
        cv2.imwrite(str(image_path), frame)
        print(f"图片已保存: {image_path}")
        index += 1
        print(sp)
        time.sleep(1)

    print("采样结果:")
    for sp in TEST_XY:
        print(sp)
    fit_samples(TEST_XY)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    cap = open_camera()
    
    try:
        ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
    except serial.SerialException as e:
        print(f'无法打开串口 {port}: {e}')
        sys.exit(1)

    print(f'已连接 {port} @ {baud}bps。')
    time.sleep(1)

    home_cmd = '$h\n'
    send_cmd(ser, home_cmd, HOME_DELAY)

    # 采样
    sampling(ser, cap)
    cap.release()
    cv2.destroyAllWindows()
    ser.close()


if __name__ == '__main__':
    main()
