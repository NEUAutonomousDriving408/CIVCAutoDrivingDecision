import ADCPlatform
from control import pid
from control.state import CarState, ControlData

def run(task, final_task=False):
    running = True
    # 毫米波真值传感器id
    radarId = 0
    # 摄像机传感器id
    cameraId = 0
    # 车道线传感器id
    landLineId = 0

    print("当前车辆安装传感器为")
    sensors = ADCPlatform.get_sensors()
    for sensor in sensors:
        if sensor.Name == "毫米波雷达":
            radarId = sensor.ID
        elif sensor.Name == "摄像机":
            cameraId = sensor.ID
        elif sensor.Name == "车道线传感器":
            landLineId = sensor.ID
        print("名称：" + sensor.Name + ",ID:" + str(sensor.ID))

    ## 初始化智能体和控制器

    controller = ControlData()
    myCar = CarState()

    # 给一个相反的角度控制，让PID的震荡更少
    # 初始油门不能给太大，不然会收不回来

    test_flag = True
    if task == 2:
        if not final_task:
            # 0.4 可选
            ADCPlatform.control(1, 20, 0, 1)
        elif final_task and not test_flag:

            ADCPlatform.control(1, 40, 0, 1)

        elif final_task and test_flag:
            ADCPlatform.control(0, 30, 1, 1)
            ADCPlatform.control(0, 30, 1, 1)
            ADCPlatform.control(0, 30, 1, 1)
            ADCPlatform.control(0, 30, 1, 1)
            ADCPlatform.control(0, 30, 1, 1)

            ADCPlatform.control(1, 30, 0, 1)

    while running:


        # 获取车辆控制数据包
        control_data_package = ADCPlatform.get_control_data()

        radar_data_package = ADCPlatform.get_data(radarId)

        line_data_package = ADCPlatform.get_data(landLineId)

        if not control_data_package:
            print("任务结束")
            running = False
            break

        # 当前车速
        spd = control_data_package.json['FS']

        # 当前偏航角
        yaw = control_data_package.json['CAO']

        # 当前角速度
        yr = control_data_package.json['YR']


        if radar_data_package is not None:

            # v: cm/s
            # 相对前车的速度
            delta_spd = radar_data_package.json[0]["Speed"] / 100  # m/s

            # 相对前车的距离
            dist = radar_data_package.json[0]["Range"] / 100  # m

            myCar.dist = dist

            myCar.delta_v = delta_spd

        if line_data_package is not None:

            if len(line_data_package.json) == 4:
                myCar.positionnow = -6.5 + (line_data_package.json[2]['A1'] + line_data_package.json[1]['A1'])
            else:
                print("ERROR")
                myCar.positionnow = -7.0


        # 保存当前的状态

        myCar.speed = spd
        myCar.cao = yaw
        myCar.yr = yr

        # 如果是决赛题，默认是False
        if final_task:

            if task == 0:

                from control.final_task.task0 import run_task0_test
                run_task0_test(myCar, controller)

            elif task == 1:

                from control.final_task.task1 import run_task1_test
                run_task1_test(myCar, controller)

            elif task == 2:
                from control.final_task.task2 import run_task2_test
                run_task2_test(myCar, controller, use_w=False)
