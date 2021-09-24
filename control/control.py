import ADCPlatform
from control import pid
from control.state import CarState, ControlData

def run(task):
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

        # print("yaw:",yaw)

        if radar_data_package is not None:

            # v: cm/s
            # 相对前车的速度
            delta_spd = radar_data_package.json[0]["Speed"] / 100

            # 相对前车的距离
            dist = radar_data_package.json[0]["Range"] / 100

        if line_data_package is not None:

            myCar.positionnow = -7.0 + (line_data_package.json[2]['A1'] + line_data_package.json[1]['A1'])

        # 保存当前的状态

        myCar.speed = spd
        myCar.cao = yr
        myCar.dist = dist
        myCar.delta_v = delta_spd

        if task == 0:

            from control.task.task0 import run_task0
            run_task0(myCar, controller)

        elif task == 1:
            x = []

        elif task == 2:


            from control.task.task2 import run_task2
            run_task2(myCar, controller)
