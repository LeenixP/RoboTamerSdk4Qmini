import evdev
import select
import json

# 盗版手柄 evdev 事件码（根据实际按键诊断结果 2026-03-20）
# 注意：此手柄按键标记与标准PS4手柄不同！
ABS_X  = 0   # 左摇杆X
ABS_Y  = 1   # 左摇杆Y
ABS_Z  = 2   # 右摇杆X
ABS_RZ = 5   # 右摇杆Y
ABS_HAT0X = 16  # 方向键X
ABS_HAT0Y = 17  # 方向键Y

# 按键映射（物理按键 -> 事件码 -> 功能）
# ✕键(305) -> 站立模式
# ○键(306) -> 退出
# □键(304) -> RL站立模式
# △键(307) -> 行走模式
# L1(308) -> 横向移动
# R1(309) -> 模式7
# L2(310) -> 模式8
# R2(311) -> 模式9
# SELECT(312) -> sin测试
# START(313) -> 就绪模式

BTN_A  = 305  # ✕键 -> 站立模式 (butA)
BTN_B  = 306  # ○键 -> 退出 (butB)
BTN_X  = 304  # □键 -> RL站立模式 (butX)
BTN_Y  = 307  # △键 -> 行走模式 (butY)
BTN_TL = 308  # L1 -> 横向移动 (L1)
BTN_TR = 309  # R1 -> 模式7 (R1)
BTN_TL2 = 310 # L2 -> 模式8 (L2)
BTN_TR2 = 311 # R2 -> 模式9 (R2)
BTN_SELECT = 312  # SELECT -> sin测试
BTN_START  = 313  # START -> 就绪模式

DEVICE_NAME = "Wireless Controller"

class JoyStick:
    LaxiX = 0.0
    LaxiY = 0.0
    RaxiX = 0.0
    RaxiY = 0.0
    hatX = 0
    hatY = 0
    butA = 0
    butB = 0
    butX = 0
    butY = 0
    L1 = 0
    R1 = 0
    L2 = 0
    R2 = 0
    SELECT = 0
    START = 0

    def __init__(self):
        self.dev = None

    def _find_device(self):
        for path in evdev.list_devices():
            d = evdev.InputDevice(path)
            if d.name == DEVICE_NAME:
                self.dev = d
                print(f"[JOYSTICK] Found: {d.path} - {d.name}")
                return
        self.dev = None

    def _normalize_axis(self, value, min_val=0, max_val=255):
        """将 0~255 映射到 -1.0~1.0"""
        center = (min_val + max_val) / 2.0
        half = (max_val - min_val) / 2.0
        return (value - center) / half

    def initjoystick(self):
        self._find_device()

    def getjoystickstates(self):
        if self.dev is None:
            self._find_device()
            if self.dev is None:
                return

        try:
            # 非阻塞读取所有待处理事件
            while select.select([self.dev.fd], [], [], 0)[0]:
                for event in self.dev.read():
                    if event.type == evdev.ecodes.EV_ABS:
                        if event.code == ABS_X:
                            self.LaxiX = self._normalize_axis(event.value)
                        elif event.code == ABS_Y:
                            self.LaxiY = self._normalize_axis(event.value)
                        elif event.code == ABS_Z:
                            self.RaxiX = self._normalize_axis(event.value)
                        elif event.code == ABS_RZ:
                            self.RaxiY = self._normalize_axis(event.value)
                        elif event.code == ABS_HAT0X:
                            self.hatX = event.value
                        elif event.code == ABS_HAT0Y:
                            self.hatY = event.value

                    elif event.type == evdev.ecodes.EV_KEY:
                        v = event.value  # 1=pressed, 0=released
                        if event.code == BTN_A:
                            self.butA = v
                        elif event.code == BTN_B:
                            self.butB = v
                        elif event.code == BTN_X:
                            self.butX = v
                        elif event.code == BTN_Y:
                            self.butY = v
                        elif event.code == BTN_TL:
                            self.L1 = v
                        elif event.code == BTN_TR:
                            self.R1 = v
                        elif event.code == BTN_TL2:
                            self.L2 = v
                        elif event.code == BTN_TR2:
                            self.R2 = v
                        elif event.code == BTN_SELECT:
                            self.SELECT = v
                        elif event.code == BTN_START:
                            self.START = v
        except OSError:
            # 手柄断开
            print("[JOYSTICK] Device disconnected, will retry...")
            self.dev = None

joy = JoyStick()

def init_joystick():
    global joy
    joy.initjoystick()

def read_joystick():
    global joy
    joy.getjoystickstates()

    result = {
        "LaxiX": joy.LaxiX,
        "LaxiY": joy.LaxiY,
        "RaxiX": joy.RaxiX,
        "RaxiY": joy.RaxiY,
        "hatX": joy.hatX,
        "hatY": joy.hatY,
        "butA": joy.butA,
        "butB": joy.butB,
        "butX": joy.butX,
        "butY": joy.butY,
        "L1": joy.L1,
        "R1": joy.R1,
        "L2": joy.L2,
        "R2": joy.R2,
        "SELECT": joy.SELECT,
        "START": joy.START,
    }

    return json.dumps(result)
