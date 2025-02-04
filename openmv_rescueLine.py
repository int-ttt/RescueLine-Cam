# Untitled - By: JEON - 2023년 5월 11일

# 이프로그램은 OpenMV 카메라에서 물체를 찾아 해당 데이터를 UART로 보내는 프로그램.
#
# 보내는 PACKET의 포맷은 아래와 같음.
#
# PACKET FORMAT (Unsigned Integer 2 Byte는 LSB, MSB순으로 되어 있으며 Little Endian 형태로 전송됨.)
# ---------------------------------------------------------------------------------------------------------
# PACKET HEADER, PACKET SeqNo, ULTRA, TOF, YAW, ROLL, PITCH, Object Count n, OBJECT[0]...OBJECT[n-1], CHECKSUM
# ---------------------------------------------------------------------------------------------------------
# 1. PACKET HEADER  : 2 Byte (0xAA , 0xCC)              -- 0XAA와 0xCC 가 연달아 들어오면 PCKET의 시작임.
# 2. PACKET SeqNo   : Unsigned Integer 1 Byte (0 ~ 127) -- PACKET 송신시마다 1씩 증가함. (수신측에서 이값이 바껴야 새로운 PACKET으로 인식)
# 3. TOF[4]         : 4개의 TOF 정보. 구조는 아래와 같음. (전체 4바이트)
#       -----------------------------------------
#       : TOF1, TOF2, TOF3, TOF4
#       -----------------------------------------
#       : TOF1          : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
#       : TOF2          : Unsigned Integer 2 Byte
#       : TOF3          : Unsigned Integer 2 Byte
#       : TOF4          : Unsigned Integer 2 Byte
#       -----------------------------------------
# ---------------------------------------------------------------------------------------------------------
# 9. CHECKSUM : PACKET HEADER를 포함한 모든 수신데이터의 Exclusive Or 값. (chksum ^= all recieve data)
# ---------------------------------------------------------------------------------------------------------
mode = 0
tof1 = 0
tof2 = 0
yaw = 0.0
roll = 0.0
pitch = 0.0

imuV = []

Blobs = [(35, 71, 25, 61, 10, 53) ,
         (56, 83, -65, -18, -4, 41),
         (0, 100, -128, 127, -128, 127) ] #Blobs[x] ; x=0: red, x=1: green, x=2: blue

Circle = []
Blob = []
Blue = []
Median = []
Mean = []

PACKET = b''

import sensor, image, pyb, time, sys
import micropython
import math, struct
from pyb import UART, Pin, Timer
from time import sleep_ms, sleep_us
from pyb import I2C
from machine import SoftI2C, Pin

from VL53L0X import VL53L0X #TOF 거리측정 센서
from BNO055 import BNO055   # 9DOF IMU 센서

#LED = (pyb.LED(1), pyb.LED(2), pyb.LED(3))

#for e in LED:
#    e.on()

micropython.alloc_emergency_exception_buf(200)

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H', 'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

def addData(type, array):
    global PACKET
    if isinstance(array,list):
        array = array[:8]     # max array size is 5 byte
        for element in array:
            PACKET += struct.pack(format[type], element)
    else:
        PACKET += struct.pack(format[type], array)

#---------------------------------------------------------------------
# 사용할 I2C 채널을 선택하세요.
#---------------------------------------------------------------------
#---------------------------------------------------------------------
# Software I2C 채널
#---------------------------------------------------------------------
I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 400000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 400000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 200000, timeout=50000) #Soft I2C
I2C_J15= SoftI2C(scl = 'P6', sda = 'P9', freq = 400000, timeout=50000) #Soft I2C

#---------------------------------------------------------------------
# Hardware I2C 채널
#---------------------------------------------------------------------
#I2C_J8 = I2C(2, I2C.MASTER, baudrate = 100000)    #Hardware I2C
#I2C_J9 = I2C(4, I2C.MASTER, baudrate = 400000)    #Hardware I2C
#---------------------------------------------------------------------
TOF1 = VL53L0X(I2C_J9)
TOF2 = VL53L0X(I2C_J15)
IMU = BNO055(I2C_J9)
TOF1.start()
TOF2.start()

#TOF4.start()

# -------------------------------------------------------------
# UART 1, and baudrate.
# -------------------------------------------------------------
uart = UART(1, 115200)

##Camera Define
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # Format: RGB565.
sensor.set_framesize(sensor.QQVGA)   # Size: QQVGA (120x160)
#sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False)         # Close auto gain (must be turned off for color tracking)
sensor.set_auto_whitebal(False)     # Close white balance (must be turned off for color tracking)
sensor.set_framerate(30)
clock = time.clock() # Create a clock object to track the FPS.

PacketSeqNO = 0

#for e in LED:
#    e.off()
Blob = []
def findEvacue(img):
    global mode, tof1, tof2, tof3
    global Silver, Black, Blue, Red, Green, Blob

    Blob = img.find_blobs([Blobs[0], Blobs[1]], merge= True, area_threshold=10)
    if Blob:
        for b in Blob:
            print(b)
            color = [0, 0, 255]
            if b.code() == 1:
                color = [255, 0, 0]
            elif b.code() == 2:
                color = [0, 255, 0]
            img.draw_rectangle(b.rect(), color = color)
asdf = 0
def CAMERA(i=0):
    global Silver, Black, Green, Red, Blue
    global PacketSeqNO, PACKET

# -------------------------------------------------------------
# 센서 읽기
# -------------------------------------------------------------
    tof1 = TOF1.read()
    tof2 = TOF2.read()
    imuV = (0, 0, 0)
    imuV = IMU.euler()
   

    print("TOF1 : ", tof1)
    print("TOF2 : ", tof2)
    print("IMU : ", imuV)
    clock.tick()
    img = sensor.snapshot()
    findEvacue(img)
# -------------------------------------------------------------
# #PACKET 만들기
# -------------------------------------------------------------
    PacketSeqNO = (PacketSeqNO + 1) & 0x7F

    PACKET = b''
    addData('uInt8', 0xAA)          # PACKET HEADER 0xAA, 0xCC
    addData('uInt8', 0xCC)
    addData('uInt16', PacketSeqNO)   # Packet Seq No.

    addData('uInt16', tof1)
    addData('uInt16', tof2)
    addData('uInt16', imuV[1])
    imuV = []
    #PACKET 만들기

    ObjectCount= len(Circle) + len(Blob) + len(Blue)

#    addData('uInt8', ObjectCount)     # Object Count
    lists = []
    if ObjectCount:   # 보낼 Object 데이터가 있다면
        if Blob:
            for b in Blob:
                if b.code() > 2:
                    continue
                buf = [b.code(), b.cx() ,b.cy() ,b.w(), b.h()]   # Object Type = 4
#                addData('uInt16', buf)                          # OBJECT[n]
                lists.append(buf)
#                img.draw_rectangle(b.rect(), color = (0, 255, 0))
    addData('uInt8', len(lists))
    for e in lists:
        addData('uInt8', e[0])
        addData('uInt16', e[1])
        addData('uInt16', e[2])
        addData('uInt8', e[3])
        addData('uInt8', e[4])
    print(lists)
    chksum = 0
    for b in PACKET:
       chksum ^= b
    addData('uInt8', chksum)        #CHECKSUM
    Silver = []
    Black = []
    Green = []
    Blue = []
    Red = []
    print(PACKET)
    # 프레임 처리 속도 (Frame Per Seocond(
    print("FPS %f" % clock.fps())
    print("")

def TOF():
    tof = TOF2.read()
    imuv = IMU.euler()
    print("TOF1 : ", tof1)
    print('imuV', imuv)


    addData('uInt8', 0xAA)          # PACKET HEADER 0xAA, 0xEE
    addData('uInt8', 0xDD)

    addData('uInt16', tof1)
    addData('uInt16', imuv[0])
    chksum = 0
    for b in PACKET:
        chksum ^= b
#    pyb.delay(10)


mode = 0
tempMode=0
while(True):
#    print(TOF1.read())
#    if TOF1.read() < 100:
#        break
    if not mode:
        CAMERA()
    else:
        TOF()

    cmd = b''
    if uart.any() > 0:  # 수신 데이터가 있다면 출력한다.
        cmd = uart.read(1)
        if cmd[0] == 0xAA:
            sleep_us(100)
            mode = 0
            if(uart.any()):
                cmd += uart.read(1)
                mode = int(cmd[1])
            print("MODE:",mode)
            uart.write(b'0xEE')
    uart.write(PACKET)

    pyb.delay(10)
#    print("median", sum(Median) / len(Median))
#    print("mean", sum(Mean) / len(Mean))  24 25 25 7 1 35 20 30



#while True:
#    objectCount = 0
#    PACKET = b''
#    t1 = TOF1.read()
#    t2 = TOF2.read()

#    errorPacket = 0

#    addData('uInt8', 0xAA)         # PACKET HEADER 0xAA, 0xCC
#    addData('uInt8', 0xCC)
#    addData('uInt8', PacketSeqNO)
#    addData('uInt8', errorPacket)        # Error Packet

#    addData('uInt16', t1)        # TOF1 : 부호 없는 2 바이트 정수('<H').
#    addData('uInt16', t2)        # TOF2 : 부호 없는 2 바이트 정수('<H').

#    chksum = 0
#    for b in PACKET:
#       chksum ^= b
#    addData('uInt8', chksum)

#    uart.write(PACKET)
#    pyb.delay(15)

