#!/usr/bin/env python3
"""简易串口交互工具，独立于 ROS。
用法: python3 serial_tool.py [端口] [波特率]
示例: python3 serial_tool.py /dev/ttyUSB0 115200
"""

import sys
import time
import serial


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.5   # 发送后等待回复的秒数


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    try:
        ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
    except serial.SerialException as e:
        print(f'无法打开串口 {port}: {e}')
        sys.exit(1)

    print(f'已连接 {port} @ {baud}bps，输入内容后按回车发送，Ctrl+C 退出。')

    try:
        while True:
            try:
                line = input('> ')
            except EOFError:
                break

            ser.write((line + '\n').encode())
            ser.flush()

            # 等待并读取回复
            time.sleep(READ_TIMEOUT)
            response = ser.read(ser.in_waiting)
            if response:
                print(response.decode(errors='replace'), end='')
    except KeyboardInterrupt:
        print('\n退出。')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
