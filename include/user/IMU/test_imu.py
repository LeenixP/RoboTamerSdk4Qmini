import serial
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE
import struct
import time

# 宏定义参数
FRAME_HEAD = 'fc'
FRAME_END = 'fd'
TYPE_IMU = '40'
TYPE_AHRS = '41'
IMU_LEN = '38'
AHRS_LEN = '30'

port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
baudrate = 921600
timeout = 1

print(f"正在连接到设备: {port}")
print(f"波特率: {baudrate}")

try:
    serial_ = serial.Serial(port=port, baudrate=baudrate, bytesize=EIGHTBITS,
                           parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=timeout)
    print(f"✓ 串口打开成功")
    print(f"✓ 实际波特率: {serial_.baudrate}")
except Exception as e:
    print(f"✗ 无法打开串口: {e}")
    exit(1)

print("\n开始接收数据...\n")

result = {
    "Accelerometer_X": 0, "Accelerometer_Y": 0, "Accelerometer_Z": 0,
    "RollSpeed": 0, "PitchSpeed": 0, "HeadingSpeed": 0,
    "Roll": 0, "Pitch": 0, "Heading": 0,
    "qw": 0, "qx": 0, "qy": 0, "qz": 0,
}

temp1 = False
temp2 = False
byte_count = 0
frame_count = 0
start_time = time.time()

try:
    while serial_.isOpen():
        # 读取一个字节
        byte_data = serial_.read(1)
        if not byte_data:
            elapsed = time.time() - start_time
            print(f"超时 ({elapsed:.1f}秒) - 已读取 {byte_count} 字节, {frame_count} 帧")
            if byte_count == 0:
                print("✗ 没有接收到任何数据！请检查:")
                print("  1. IMU设备是否已上电")
                print("  2. 串口连接是否正确")
                print("  3. 波特率设置是否匹配")
            break

        byte_count += 1
        check_head = byte_data.hex()

        # 每1000字节显示一次进度
        if byte_count % 1000 == 0:
            print(f"已接收 {byte_count} 字节, {frame_count} 帧...")

        # 校验帧头
        if check_head != FRAME_HEAD:
            continue

        frame_count += 1
        print(f"\n[帧 {frame_count}] 检测到帧头 0x{check_head}")

        head_type = serial_.read().hex()
        print(f"  数据类型: 0x{head_type}", end="")

        # 校验数据类型
        if head_type not in [TYPE_IMU, TYPE_AHRS]:
            print(" (跳过)")
            continue

        if head_type == TYPE_IMU:
            print(" (IMU数据)")
        elif head_type == TYPE_AHRS:
            print(" (AHRS数据)")

        check_len = serial_.read().hex()
        print(f"  数据长度: 0x{check_len}")

        # 校验数据长度
        if head_type == TYPE_IMU and check_len != IMU_LEN:
            print(f"  ✗ 长度错误 (期望: 0x{IMU_LEN})")
            continue
        elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
            print(f"  ✗ 长度错误 (期望: 0x{AHRS_LEN})")
            continue

        check_sn = serial_.read().hex()
        head_crc8 = serial_.read().hex()
        crc16_H_s = serial_.read().hex()
        crc16_L_s = serial_.read().hex()

        # 读取并解析IMU数据
        if head_type == TYPE_IMU:
            data_s = serial_.read(int(IMU_LEN, 16))
            IMU_DATA = struct.unpack('12f ii', data_s[0:56])
            result["Accelerometer_X"] = IMU_DATA[3]
            result["Accelerometer_Y"] = IMU_DATA[4]
            result["Accelerometer_Z"] = IMU_DATA[5]
            print(f"  ✓ 加速度: X={IMU_DATA[3]:.3f}, Y={IMU_DATA[4]:.3f}, Z={IMU_DATA[5]:.3f} m/s²")
            temp1 = True

        # 读取并解析AHRS数据
        elif head_type == TYPE_AHRS:
            data_s = serial_.read(int(AHRS_LEN, 16))
            AHRS_DATA = struct.unpack('10f ii', data_s[0:48])
            result["RollSpeed"] = AHRS_DATA[0]
            result["PitchSpeed"] = AHRS_DATA[1]
            result["HeadingSpeed"] = AHRS_DATA[2]
            result["Roll"] = AHRS_DATA[3]
            result["Pitch"] = AHRS_DATA[4]
            result["Heading"] = AHRS_DATA[5]
            result["qw"] = AHRS_DATA[6]
            result["qx"] = AHRS_DATA[7]
            result["qy"] = AHRS_DATA[8]
            result["qz"] = AHRS_DATA[9]
            print(f"  ✓ 姿态: Roll={AHRS_DATA[3]:.3f}, Pitch={AHRS_DATA[4]:.3f}, Heading={AHRS_DATA[5]:.3f} rad")
            print(f"  ✓ 四元数: qw={AHRS_DATA[6]:.3f}, qx={AHRS_DATA[7]:.3f}, qy={AHRS_DATA[8]:.3f}, qz={AHRS_DATA[9]:.3f}")
            temp2 = True

        # 当IMU和AHRS数据都收到时，显示完整结果
        if temp1 and temp2:
            print("\n" + "="*60)
            print("完整数据包:")
            print("="*60)
            print(f"加速度 (m/s²):")
            print(f"  X: {result['Accelerometer_X']:>10.3f}")
            print(f"  Y: {result['Accelerometer_Y']:>10.3f}")
            print(f"  Z: {result['Accelerometer_Z']:>10.3f}")
            print(f"\n角速度 (rad/s):")
            print(f"  Roll:    {result['RollSpeed']:>10.3f}")
            print(f"  Pitch:   {result['PitchSpeed']:>10.3f}")
            print(f"  Heading: {result['HeadingSpeed']:>10.3f}")
            print(f"\n姿态角 (rad):")
            print(f"  Roll:    {result['Roll']:>10.3f}")
            print(f"  Pitch:   {result['Pitch']:>10.3f}")
            print(f"  Heading: {result['Heading']:>10.3f}")
            print(f"\n四元数:")
            print(f"  qw: {result['qw']:>10.3f}")
            print(f"  qx: {result['qx']:>10.3f}")
            print(f"  qy: {result['qy']:>10.3f}")
            print(f"  qz: {result['qz']:>10.3f}")
            print("="*60)
            print("\n✓ 设备工作正常！所有数据都能正确接收。\n")
            temp1 = False
            temp2 = False

            # 继续接收更多数据包
            print("继续接收数据... (按 Ctrl+C 停止)\n")

except KeyboardInterrupt:
    print("\n\n用户中断")
except Exception as e:
    print(f"\n✗ 错误: {e}")
finally:
    serial_.close()
    print(f"\n串口已关闭")
    print(f"总计: {byte_count} 字节, {frame_count} 帧")
