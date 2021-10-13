import ADCPlatform
import numpy as np

speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2

# 角速度控制器
def latitudeyrControlpos(yr, yrPid):
    yrPid.update(yr)
    yrPid.yrsteer_ = yrPid.output * -1

# 转向角控制器
def latitudeControlpos(positionnow, latPid):
    latPid.update(positionnow)
    latPid.steer_ = latPid.output * -1
    if abs(latPid.steer_) > 40:
        latPid.steer_ = 40 if latPid.steer_ > 0 else -40


def lontitudeControlSpeed(speed, lonPid):
    lonPid.update(speed - 5.0)
    if (lonPid.output > speedPidThread_1):  # 加速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 1')
        # print("==============================================")
        lonPid.thorro_ = 1
        lonPid.brake_ = 0

    elif (lonPid.output > speedPidThread_2):  # 稳定控速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 2')
        # print("==============================================")

        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)

    elif (lonPid.output > 0):  # 下侧 微调

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        # print("==============================================")
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        lonPid.brake_ = ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.2

    elif (lonPid.output < -1 * speedPidThread_1):  # 减速阶段

        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 4')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.1
        lonPid.brake_ = 0.5

    else:
        # print("==============================================")
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 5')
        # print("==============================================")
        lonPid.thorro_ = (-1 * lonPid.output / speedPidThread_2) * 0.2
        lonPid.brake_ = ((speedPidThread_2 - (-1 * lonPid.output)) / speedPidThread_2) * 0.4
    # print(lonPid.thorro_, '    ', lonPid.brake_)

# 这种情况的速度控制为25

def getTTC(current_speed, current_acceleration, dist, safe_dist):

    v = current_speed
    a = current_acceleration

    if a !=0:
        ttc = ( -v + np.sqrt(v **2 + 2 * a * (dist-safe_dist)) ) / a
    else:
        ttc = 999

    return ttc

def run_task2(myCar, Controller):

    Controller.speedPid.setSetpoint(25)

    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(myCar.speed, Controller.speedPid)

    # 加入角速度约束
    # latitudeyrControlpos(MyCar.yr, Controller.yrPid)

    # 横向控制 steer_
    latitudeControlpos(myCar.positionnow, Controller.latPid)

    if myCar.dist < 10:

        lontitudeControlSpeed(10, Controller.latPid)

        # print("dist:",dist)

        if myCar.speed != 0:
            ttc = myCar.dist / myCar.speed
        else:
            ttc = 999

        if myCar.dist < 3:
            print("还有{}".format(np.round(myCar.dist, 3)), "米就到了！")
            ADCPlatform.control(0, Controller.latPid.steer_,
                                1, 0)

        if ttc < 3:
            print("还有{}".format(np.round(ttc, 3)), "秒就到了！")
            ADCPlatform.control(0, Controller.latPid.steer_,
                                0.6, 0)
        else:
            ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_,
                                0.2, 1)
    else:
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_,
                            0, 1)


def run_task2_test(myCar, Controller):

    Controller.speedPid.setSetpoint(25)

    # 纵向控制 thorro_ and brake_
    lontitudeControlSpeed(myCar.speed, Controller.speedPid)

    # 加入角速度约束
    # latitudeyrControlpos(MyCar.yr, Controller.yrPid)

    # 横向控制 steer_
    latitudeControlpos(myCar.positionnow, Controller.latPid)

    dt = np.round(myCar.dist / myCar.delta_v, 3)

    a = np.round(myCar.delta_v / dt, 3)

    ttc = getTTC(myCar.speed, a, myCar.dist, safe_dist=0.5)


    DANGER = False

    if myCar.dist < 1:
        DANGER = True

    if myCar.dist < 15 and a > 5.5:
        DANGER = True

    if ttc > 0.5 and not DANGER and ttc != 999:
        # 正常情况下使用PID来进行纵向控制
        print("ttc:", ttc)
        ADCPlatform.control(Controller.speedPid.thorro_, Controller.latPid.steer_,
                            0, 1)

    elif ttc <= 0.5 and not DANGER:
        # 如果ttc时间在一定范围了，软刹车
        print("dist:", myCar.dist, "ttc:", ttc, "a:", a)

        # 如果加速度过小，那么刹车再软一点点 4

        if a > 4:
            ADCPlatform.control(0, Controller.latPid.steer_,
                                0.5, 1)
        else:
            ADCPlatform.control(0, Controller.latPid.steer_,
                                0.5, 1)


    elif DANGER:
        # 如果太近了，硬刹车
        print("danger!")
        ADCPlatform.control(0, Controller.latPid.steer_, 1, 1)