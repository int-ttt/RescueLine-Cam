IMU = namedtuple('IMU', ['yaw', 'pitch', 'roll'])

print(imu)
def _read_register(register, length = 1) -> int:
    global ReadFail, imu
    
    while True:
        i = 0
        ReadFail = True

        while i < 10:
            imu.write(bytes([0xAA, 0x01, register, length]))
            w = StopWatch()
            now = w.time()
            while imu.waiting() < length + 2 and w.time() - now < 250:
                yield
            resp = imu.read(imu.waiting())
            if len(resp) >= 2 and resp[0] == 0xBB:
                ReadFail = False
                break
            #wait(10)
            i += 1
            yield
        if len(resp) < 2:                       #BNO055로부터의 응답은 최소 2바이트임, 2바이트보다 작으면 에러
            print("UART access error.")
        if resp[0] != 0xBB:                     #BNO055로부터의 정상적인 Data는 0xBB로 시작하며, 에러는 0xEE로 시작함.
            print('UART read error: ', resp[1])
        if length > 1:                          #요청한 Data의 사이즈가 1바이트 이상이면 Data 어레이 리턴, 아니면 1바이트만 리턴
            yield resp[2:]
        yield resp[2]

def _write_register(register: int, data: int) -> None:
    if not isinstance(data, bytes):
        data = bytes([data])
    print(bytes([0xAA, 0x00, register, len(data)]) + data)
    imu.write(bytes([0xAA, 0x00, register, len(data)]) + data)
    w = StopWatch()
    now = w.time()
    while imu.waiting() < 2 and w.time() - now < 700:
        print(imu.waiting())
        pass
    resp = imu.read(imu.waiting())
    if len(resp) < 2:
        print("UART access error.")
    if resp[0] != 0xEE or resp[1] != 0x01:
        print('UART write error: ', resp[1])



def euler() -> Tuple[float, float, float]:
    imu_task = _read_register(0x1A, 6)
    yield
    while True:
        while True:
            a = next(imu_task)
            if a is not None:
                break
            yield
        wait(10)
        if isinstance(a, int):
            yield
            continue
        resp = struct.unpack("<hhh", a)
        l =[]
        for x in resp:
            l.append(x / 16)
        _euler = tuple(l)
        yield IMU(_euler[0], _euler[1], _euler[2]-180 if _euler[2] > 0 else _euler[2]+180)