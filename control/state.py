from control import pid

class CarState(object):
    def __init__(self):
        self.speed = 0  # 车辆当前速度
        self.cao = 0  # 车辆当前姿态
        self.cardecision = 'speedup'  # planning计算得到决策
        self.midlane = -1  # 7 0 -7 latpid 参考 target

        self.positionnow = 0  # 两车道线A1求和
        self.changing = False  # 处于超车状态时为True

        self.saftydistance = 25  # 与前车的安全距离

        self.delta_v = 0

        self.acc = 0

        self.dist = 0 # 与前车的实际距离
        self.direction = 'mid'
        self.yr = 0  # 车辆当前姿态


class ControlData(object):
    def __init__(self):
        '''
        self.lat_kp = 1.10
        self.lat_ki = 0.08
        self.lat_kd = 6.2

                '''
        self.lat_kp = 1.1
        self.lat_ki = 0.14
        self.lat_kd = 6.2

        self.latPid = pid.PID(self.lat_kp, self.lat_ki, self.lat_kd)

        self.yr_kp = 1.0
        self.yr_ki = 0.10
        self.yr_kd = 0
        self.yrPid = pid.PID(self.yr_kp, self.yr_ki, self.yr_kd)

        self.targetSpeedInit = 60.0  # 想要到达的速度
        self.speed_kp = 1.20
        self.speed_ki = 0.02
        self.speed_kd = 0.5
        self.speedPid = pid.PID(self.speed_kp, 0, self.speed_kp)
        self.speedPidThread_1 = 10
        self.speedPidThread_2 = 2

    def initPID(self):
        self.speedPid.clear()  # lon
        self.latPid.clear()  # lat
        self.yrPid.clear()  # lat
        self.speedPid.setSetpoint(self.targetSpeedInit)  # 保持40km/h
        self.latPid.setSetpoint(0)  # lat aim 0
        self.yrPid.setSetpoint(0)  # lat aim 0