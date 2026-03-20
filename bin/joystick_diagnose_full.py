#!/usr/bin/env python3
"""
完整手柄按键诊断工具
用于确定手柄的完整按键映射
支持按键事件(KEY)和轴事件(ABS/HAT)
"""
import evdev
import select
import time

DEVICE_NAME = "Wireless Controller"

def find_device():
    """查找手柄设备"""
    for path in evdev.list_devices():
        d = evdev.InputDevice(path)
        if d.name == DEVICE_NAME:
            return d
    return None

def main():
    print("=" * 60)
    print("        完整手柄按键诊断工具")
    print("=" * 60)
    print("\n请依次按下手柄上的每个按键，我会记录事件码")
    print("按 Ctrl+C 退出\n")
    
    dev = find_device()
    if not dev:
        print("[错误] 未找到手柄设备")
        return
    
    print(f"[成功] 已连接到: {dev.name}")
    print(f"[路径] {dev.path}\n")
    
    # 需要测试的按键列表（含位置说明）
    # 方向键使用特殊格式: ("方向键_XX", ..., ..., "HAT") 表示需要检测HAT事件
    test_buttons = [
        ("✕键 (X/Cross)", "站立模式", "手柄右侧下方，标记X或✕", "KEY"),
        ("○键 (O/Circle)", "退出", "手柄右侧右方，标记O或○", "KEY"),
        ("□键 (Square)", "RL站立模式", "手柄右侧左方，标记□", "KEY"),
        ("△键 (Triangle)", "行走模式", "手柄右侧上方，标记△", "KEY"),
        ("L1 (左肩键)", "横向移动", "手柄左上方肩键（非扳机）", "KEY"),
        ("R1 (右肩键)", "模式7", "手柄右上方肩键（非扳机）", "KEY"),
        ("L2 (左扳机)", "模式8", "手柄左上方扳机键", "KEY"),
        ("R2 (右扳机)", "模式9", "手柄右上方扳机键", "KEY"),
        ("SELECT/Share", "sin测试", "手柄中间偏左小按键", "KEY"),
        ("START/Options", "就绪模式", "手柄中间偏右小按键", "KEY"),
        ("方向键 上", "模式8", "十字方向键的上方", "HAT"),
        ("方向键 下", "横向移动", "十字方向键的下方", "HAT"),
        ("方向键 左", "模式7", "十字方向键的左方", "HAT"),
        ("方向键 右", "模式9", "十字方向键的右方", "HAT"),
    ]
    
    recorded = {}  # {事件码: (按键名, 功能)}
    hat_recorded = {}  # 记录HAT事件
    button_idx = 0
    last_hat = (0, 0)  # 上一次HAT状态
    
    print("-" * 60)
    print("按键映射测试")
    print("-" * 60)
    
    if button_idx < len(test_buttons):
        btn_name, btn_func, btn_desc, btn_type = test_buttons[button_idx]
        print(f"\n[{button_idx+1}/{len(test_buttons)}] 请按下: {btn_name}")
        print(f"    功能: {btn_func}")
        print(f"    位置: {btn_desc}")
    
    try:
        while True:
            if select.select([dev.fd], [], [], 0.1)[0]:
                for event in dev.read():
                    detected = False
                    
                    # 检测按键事件 (EV_KEY)
                    if event.type == evdev.ecodes.EV_KEY:
                        if event.value == 1:  # 按下
                            code = event.code
                            btn_name, btn_func, btn_desc, btn_type = test_buttons[button_idx] if button_idx < len(test_buttons) else ("未知", "未知", "未知", "KEY")
                            if btn_type == "KEY":
                                if code not in recorded:
                                    recorded[code] = (btn_name, btn_func)
                                    print(f"\n  ✓ [记录] 按键事件码 {code} → {btn_name}")
                                    detected = True
                    
                    # 检测HAT事件 (EV_ABS - ABS_HAT0X/ABS_HAT0Y)
                    elif event.type == evdev.ecodes.EV_ABS:
                        if event.code == 16:  # ABS_HAT0X
                            current_hat = (event.value, last_hat[1])
                            if event.value != 0 and last_hat[0] == 0:  # 刚按下
                                btn_name, btn_func, btn_desc, btn_type = test_buttons[button_idx] if button_idx < len(test_buttons) else ("未知", "未知", "未知", "HAT")
                                if btn_type == "HAT":
                                    direction = "右" if event.value == 1 else "左"
                                    hat_key = f"HAT_X_{direction}"
                                    if hat_key not in hat_recorded:
                                        hat_recorded[hat_key] = (btn_name, btn_func, event.value)
                                        print(f"\n  ✓ [记录] HAT事件 ABS_HAT0X={event.value} ({direction}) → {btn_name}")
                                        detected = True
                            last_hat = current_hat
                        elif event.code == 17:  # ABS_HAT0Y
                            current_hat = (last_hat[0], event.value)
                            if event.value != 0 and last_hat[1] == 0:  # 刚按下
                                btn_name, btn_func, btn_desc, btn_type = test_buttons[button_idx] if button_idx < len(test_buttons) else ("未知", "未知", "未知", "HAT")
                                if btn_type == "HAT":
                                    direction = "下" if event.value == 1 else "上"
                                    hat_key = f"HAT_Y_{direction}"
                                    if hat_key not in hat_recorded:
                                        hat_recorded[hat_key] = (btn_name, btn_func, event.value)
                                        print(f"\n  ✓ [记录] HAT事件 ABS_HAT0Y={event.value} ({direction}) → {btn_name}")
                                        detected = True
                            last_hat = current_hat
                    
                    # 前进到下一个按键
                    if detected:
                        button_idx += 1
                        if button_idx < len(test_buttons):
                            btn_name, btn_func, btn_desc, btn_type = test_buttons[button_idx]
                            print(f"\n[{button_idx+1}/{len(test_buttons)}] 请按下: {btn_name}")
                            print(f"    功能: {btn_func}")
                            print(f"    位置: {btn_desc}")
                        else:
                            print("\n" + "=" * 60)
                            print("        所有按键已记录完成！")
                            print("=" * 60)
                            break
                        
            if button_idx >= len(test_buttons):
                break
    except KeyboardInterrupt:
        print("\n\n用户中断")
    
    # 输出映射表
    print("\n" + "=" * 60)
    print("        按键映射结果")
    print("=" * 60)
    print("\n【复制以下内容到 joystick.py】\n")
    print("# 手柄 evdev 事件码（诊断结果）")
    print("ABS_X  = 0   # 左摇杆X")
    print("ABS_Y  = 1   # 左摇杆Y")
    print("ABS_Z  = 2   # 右摇杆X")
    print("ABS_RZ = 5   # 右摇杆Y")
    print("ABS_HAT0X = 16  # 方向键X (左=-1, 右=1)")
    print("ABS_HAT0Y = 17  # 方向键Y (上=-1, 下=1)")
    print()
    
    # 查找各按键的事件码
    def find_code(keyword):
        for code, (name, func) in recorded.items():
            if keyword in name:
                return code
        return "??"
    
    print(f"BTN_A  = {find_code('Cross')}  # ✕键 -> 站立模式")
    print(f"BTN_B  = {find_code('Circle')}  # ○键 -> 退出")
    print(f"BTN_X  = {find_code('Square')}  # □键 -> RL站立模式")
    print(f"BTN_Y  = {find_code('Triangle')}  # △键 -> 行走模式")
    print(f"BTN_TL = {find_code('L1')}  # L1 -> 横向移动")
    print(f"BTN_TR = {find_code('R1')}  # R1 -> 模式7")
    print(f"BTN_TL2 = {find_code('L2')} # L2 -> 模式8")
    print(f"BTN_TR2 = {find_code('R2')} # R2 -> 模式9")
    print(f"BTN_SELECT = {find_code('SELECT')}  # SELECT -> sin测试")
    print(f"BTN_START  = {find_code('START')}  # START -> 就绪模式")
    
    print("\n# 方向键通过 ABS_HAT0X/ABS_HAT0Y 轴事件检测，不是按键事件")
    print("# 在 joystick.py 中已通过 hatX/hatY 处理")
    
    print("\n" + "-" * 60)
    print("按键事件映射表：")
    print("-" * 60)
    for code in sorted(recorded.keys()):
        btn_name, btn_func = recorded[code]
        print(f"  按键事件码 {code:3d} → {btn_name:20s} → {btn_func}")
    
    if hat_recorded:
        print("\n" + "-" * 60)
        print("方向键(HAT)事件映射：")
        print("-" * 60)
        for key, (name, func, val) in sorted(hat_recorded.items()):
            print(f"  {key} = {val} → {name} ({func})")

if __name__ == "__main__":
    main()