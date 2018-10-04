import serial
import serial.tools.list_ports
from time import sleep as wait
from struct import pack, unpack
from collections import namedtuple


robot_vid = 0x0403
robot_pid = 0x6001
robot_pid2 = 0x6015

handshake  = 0x77
end_flag   = 0x33

drive_flag = 0x45
left_flag  = 0x36
right_flag = 0x35
lights_flag = 0x30

ir_read_flag = 0x27
ir_pos_flag  = 0x28

buzzer_flag  = 0x29

class Robot:

    def __init__(self, baud, file=None):
        self.port = serial.Serial()
        if file != None:
            self.location = file
        else:
            self.location = None
        self.port.baudrate = baud
        self.port.port = self.location
        self.port.timeout = 1
        self.port.dtr = 1
        self.connected = False
        self.connect()

    def find_robot(self):
        port_list = serial.tools.list_ports.comports()
        for i in port_list:
            print(i.device)
            if (i.vid == robot_vid) and (i.pid == robot_pid or i.pid == robot_pid2):
                return str(i.device)
        return None

    def connect(self, port=None):
        while self.connected == False:
            self.location = self.find_robot()
            if self.location == None:
                print("No Geekbot found, trying again!")
                wait(1)
            else:
                print("Geekbot found:" + self.location)
                self.port.port = self.location
                wait(1)
                self.port.open()
                wait(2)
                self.port.write(chr(handshake).encode())
                while self.port.read() != chr(0x77).encode():
                    print("Waiting for handshake")
                self.connected = True


    def is_connected(self):
        return self.connected

    def shutdown(self):
        self.halt()
        self.port.close()

    def map_short(self, num): #where num is a num 0 - 100
        temp = (num * 32767)/100
        if temp > 32767:
            return 32767
        elif temp < -32767:
            return -32767
        return int(temp)

    def pack_short(self,num):
        return pack("h", int(num))

    def send_cmd(self,flag, data):
        self.port.write(chr(flag).encode())
        self.port.write(self.pack_short(self.map_short(data)))

    def lights_on(self):
        self.send_cmd(lights_flag, 0x01)

    def lights_off(self):
        self.send_cmd(lights_flag, 0x00)

    def halt(self):
        self.send_cmd(drive_flag, 0)

    def turn(self, speed, seconds=None):
        self.send_cmd(left_flag, -speed)
        self.send_cmd(right_flag, speed)
        if seconds != None:
            wait(seconds)
            self.halt()
            wait(0.2)
        return

    def turn_left(self, speed, seconds=None):
        if seconds == None:
            self.turn(-speed)
        else:
            self.turn(-speed, seconds)
        return

    def turn_right(self, speed, seconds=None):
        if seconds == None:
            self.turn(speed)
        else:
            self.turn(speed, seconds)
        return

    def drive_forward(self, speed, adjust=None, seconds=None):
        if adjust == None:
            self.send_cmd(drive_flag, speed)
        else:
            self.drive_left_wheel(speed)
            adjusted = speed+adjust
            if adjusted > 100:
                self.drive_right_wheel(100)
            elif adjusted < 0:
                self.drive_right_wheel(0)
            else:
                self.drive_right_wheel(adjusted)
        if seconds == None:
            return
        wait(seconds)
        self.halt()
        wait(0.25)


    def drive_backward(self, speed, adjust=None, seconds=None):
        if adjust == None:
            self.send_cmd(drive_flag, -speed)
        else:
            self.drive_left_wheel(-speed)
            adjusted = speed+adjust
            if   adjusted > 100:
                self.drive_right_wheel(-100)
            elif adjusted < 0:
                self.drive_right_wheel(0)
            else:
                self.drive_right_wheel(-(adjusted))
        if seconds == None:
            return
        wait(seconds)
        self.halt()
        wait(0.25)

    def drive_right_wheel(self, speed):
        self.send_cmd(right_flag, -speed)

    def drive_left_wheel(self, speed):
        self.send_cmd(left_flag, -speed)

    def get_ir_distance(self):
        self.send_cmd(ir_read_flag, 1)
        data = self.port.read(2)
        dist = unpack(">H", data)
        return dist[0]

    def set_ir_position(self, angle):
        self.send_cmd(ir_pos_flag, angle-90)

    def buzzer_on(self):
        self.send_cmd(buzzer_flag, 2)

    def buzzer_off(self):
        self.send_cmd(buzzer_flag, 0)

    def beep(self, beeps):
        for i in range (0,beeps):
            self.buzzer_on()
            wait(.125)
            self.buzzer_off()
            wait(.125)

