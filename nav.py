from serial import Serial
import serial.tools.list_ports as ports
import time
import os
from threading import Thread

class NavigatorEntry:

    def __init__(self, buf: bytes):
        self.__yaw: int = self.__get_int_value(buf, 4)
        self.__pitch: int = self.__get_int_value(buf, 6)
        self.__roll: int = self.__get_int_value(buf, 8)
        self.__compass: int = self.__get_int_value(buf, 10, False)
        self.__altitude: float = self.__get_q_value(buf, 12)
        self.__fused_heading: int = self.__get_int_value(buf, 16, False)
        self.__accel_x: int = self.__get_int_value(buf, 18)
        self.__accel_y: int = self.__get_int_value(buf, 20)
        self.__accel_z: int = self.__get_int_value(buf, 22)
        self.__velocity_x: float = self.__get_q_value(buf, 24)
        self.__velocity_y: float = self.__get_q_value(buf, 28)
        self.__velocity_z: float = self.__get_q_value(buf, 32)
        self.__displacement_x: float = self.__get_q_value(buf, 36)
        self.__displacement_y: float = self.__get_q_value(buf, 40)
        self.__displacement_z: float = self.__get_q_value(buf, 44)
        self.__quaternion_w: int = self.__get_int_value(buf, 48)
        self.__quaternion_x: int = self.__get_int_value(buf, 50)
        self.__quaternion_y: int = self.__get_int_value(buf, 52)
        self.__quaternion_z: int = self.__get_int_value(buf, 54)
        self.__temp: int = self.__get_int_value(buf, 56)
        self.__op_status: int = buf[58]
        self.__sensor_status: int = buf[59]
        self.__cal_status: int = buf[60]
        self.__selftest_status: int = buf[61]
 

    def __str__(self):
        return f"Yaw={self.yaw}, Pitch={self.pitch}, Roll={self.roll}, compass={self.fused_heading}"

    def __get_int_value(self, buf: bytes, start: int, signed:bool = True)->int:
        return int.from_bytes(bytes([buf[start+1], buf[start]]), byteorder="big", signed=signed)

    def __get_q_value(self, buf: bytes, start: int)->float:
        return int.from_bytes([buf[start+3], buf[start+2], buf[start+1], buf[start+0]], byteorder="big", signed=True)/65536.0
    
    @property 
    def yaw(self)->float:
        return self.__yaw/100.0
    
    @property
    def pitch(self)->float:
        return self.__pitch/100.0
    
    @property
    def roll(self)->float:
        return self.__roll/100.0
    
    @property
    def compass(self)->float:
        return self.__compass/100.0
    
    @property
    def altitude(self)->float:
        return self.__altitude
    
    @property
    def fused_heading(self)->float:
        return self.__fused_heading/100.0
    
    @property
    def accel_x(self)->float:
        return self.__accel_x/1000.0
    
    @property
    def accel_y(self)->float:
        return self.__accel_y/1000.0

    @property
    def accel_z(self)->float:
        return self.__accel_z/1000.0
    
    @property
    def velocity_x(self)->float:
        return self.__velocity_x
    
    @property
    def velocity_y(self)->float:
        return self.__velocity_y

    @property
    def velocity_z(self)->float:
        return self.__velocity_z

    @property
    def displacement_x(self)->float:
        return self.__displacement_x

    @property
    def displacement_y(self)->float:
        return self.__displacement_y

    @property
    def displacement_z(self)->float:
        return self.__displacement_z

    @property
    def quaternion_w(self)->int:
        return self.__quaternion_w

    @property
    def quaternion_x(self)->int:
        return self.__quaternion_x

    @property
    def quaternion_y(self)->int:
        return self.__quaternion_y

    @property
    def quaternion_z(self)->int:
        return self.__quaternion_z

    @property
    def temp(self)->float:
        return self.__temp/100.0
    
    @property
    def op_status(self)->int:
        return self.__op_status
    
    @property
    def sensor_status(self)->int:
        return self.__sensor_status
    
    @property
    def cal_status(self)->int:
        return self.__cal_status
    
    @property
    def selftest_status(self)->int:
        return self.__selftest_status
    
class Navigator:
    """
    Protocol Information: https://pdocs.kauailabs.com/navx-mxp/advanced/serial-protocol/
    """
    __device_path:str = "/dev/serial/by-id/"
    __device_name_v1: str="usb-STMicroelectronics_STM32_Virtual_ComPort__Kauai_Labs__00000000001A-if00"
    def __init__(self):
        if os.name == "nt":
            for port_info in ports.comports():
                if "STMicroelectronics Virtual COM Port" in str(port_info):
                    self.__com: Serial = Serial(port_info.device, baudrate=9600)
                    break            
        else:
            self.__com: Serial = Serial(Navigator.__device_path + Navigator.__device_name_v1, 9600)
        self.__buf: bytes = []
        self.__position: int = 0

    def __read_next(self):
        return self.__com.read(self.__com.in_waiting)      
    
    def buffer_next(self):
        b = self.__read_next()
        if len(self.__buf) == 0:
            start: int = self.__find_msg_beginning(b)
            self.__buf = b[start:len(b)]
        else:
            self.__buf = b''.join([self.__buf[self.__position:len(self.__buf)], b])
            self.__position = 0

    def __find_msg_beginning(self, b: bytes, pos: int = 0)->int:
        ret: int = -1        
        while True:
            t: int = b.find(33, pos)
            if t < 0:
                break
            l: int = b[t+2]
            if t+l > len(b):
                break
            if b[l+t] == 13:
                ret = t
                break
            else:
                pos = t+1
        return ret
        
    @property
    def position(self)->int:
        return self.__position
    
    def get_next_nav_event(self)->NavigatorEntry:
        if len(self.__buf) <= 0:
            if self.__com.in_waiting < 256:
                return None
            else:
                self.buffer_next()
        elif self.__buf[self.__position+2]+self.__position > len(self.__buf)-256:
            if self.__com.in_waiting < 256:
                return None
            else:
                self.buffer_next()
        chunk:bytes = self.__buf[self.__position:self.__position+self.__buf[self.__position+2]]        
        self.__position = self.__find_msg_beginning(self.__buf, self.__position+len(chunk)+1)
        return NavigatorEntry(chunk)

    def test(self):
        while(True):
            print(f"\r{self.get_next_nav_event()}", end="", flush=True)
            time.sleep(0.1)

class NavigationMonitor:

    def __init__(self):
        self.__current: NavigatorEntry = None
        self.__nav: Navigator = Navigator()
        self.__is_running:bool = True
        self.__thread: Thread = Thread(target=self.__run)
        self.__thread.start()

    def stop(self, wait_for_exit: bool = True):
        self.__is_running = False
        if wait_for_exit:
            self.__thread.join()

    def __run(self):
        while(self.__is_running):
            temp: NavigatorEntry = self.__nav.get_next_nav_event()
            if not temp is None:
                self.__current = temp
            else:
                time.sleep(0.1)

    def __del__(self):
        if self.__is_running:
            self.stop()

    @property
    def current(self)->NavigatorEntry:
        return self.__current
