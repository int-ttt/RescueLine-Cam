from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from ucollections import namedtuple
from pybricks.iodevices import UARTDevice
import struct


ser = UARTDevice(Port.S4, 115200)
ser.clear()
wait(100)

CamData = namedtuple("CamData", ["type", "t1", "t2", "objects"])
Block = namedtuple('Block', ['type', 'x', 'y', 'height', 'width'])
null_tof = CamData(False, 0, 0, [])

rcvPACKET = bytearray([])
Object = list([])
objectCount = 0

def _parsing(_data) -> tuple["uInt8", "Int16", "Int16", "uInt8", "uInt8"]: # 7byte
    buf = struct.unpack("<BhhBB", _data)
    return tuple(buf)

def readPacket() -> bool:
    global PacketIndex
    global ObjectIndex
    global rcvPACKET
    global objectCount
    read = False

    rcvPACKET = bytearray([])
    rcvPACKET += ser.read()
    PacketIndex = 0
    if rcvPACKET[0] == 0xAA:
        PacketIndex = 1
        rcvPACKET += ser.read()
        if rcvPACKET[1] == 0xCC:
            PacketIndex = 2
            rcvPACKET += ser.read(6)
            rcvPACKET += ser.read() # ObjectCount
            objectCount = rcvPACKET[8]
            wait(10)
            for i in range(objectCount):
                rcvPACKET += ser.read(7)


            rcvPACKET += ser.read() #CHECKSUM

            chksum = 0
            for r in rcvPACKET:     #CHECKSUM 계산
                chksum ^= r
                
            if chksum != 0:         #CHECKSUM ERROR
                print("CHECK SUM ERROR")
                return False

            return True
    return False
            
def readCam():
    read = True
    if ser.waiting() >= 5:  # 수신 데이터가 있으면 (최소 사이즈는 5 바이트임)
        wait(1) 
        p = readPacket()
        if p == 1:    # 데이터 수신에 성공했으면 Parsing 함
            tof1 = struct.unpack("<H",rcvPACKET[4:6])[0]
            tof2 = struct.unpack("<H",rcvPACKET[6:8])[0]
            objects = []
            for i in range(objectCount):
                index = 9 + i * 7
                data = _parsing(rcvPACKET[index:index+7])
                objects.append(Block(data[0], data[1], data[2], data[3], data[4]))
            # print("OK")
            # print("TOF1  : ",tof1)
            # print("TOF2  : ",tof2)
            # print("TOF3  : ",tof3)
            # print("TOF4  : ",tof4)
            # print(rcvPACKET, ser.read_all())
            # if tof1 < 50:
            #     print("TOF1  : ",tof1)
            # if tof2 < 50:
            #     print("TOF2  : ",tof2)
            return CamData(1, tof1, tof2, objects)
        elif p == 2:
            tof = struct.unpack("<H",rcvPACKET[4:6])[0]
            imu = struct.unpack("<H",rcvPACKET[6:8])[0]
            # print("OK")
            # print("TOF1  : ",tof1)
            # print("TOF2  : ",tof2)
            # print("TOF3  : ",tof3)
            # print("TOF4  : ",tof4)
            # print(rcvPACKET, ser.read_all())
            # if tof1 < 50:
            #     print("TOF1  : ",tof1)
            # if tof2 < 50:
            #     print("TOF2  : ",tof2)
            return CamData(2, tof, imu, [])
        else: return null_tof
    else:
        return null_tof

def getCam():
    ser.clear()
    while True:
        cam = readCam()
        if cam.type:
            break
    print("gTOF_debug : {}".format(cam))
    return cam

def getCoCam():
    while True:
        ser.clear()
        while True:
            cam = readCam()
            if cam.type:
                break
            yield False, 0
        yield True, cam