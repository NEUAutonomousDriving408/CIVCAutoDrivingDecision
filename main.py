import ADCPlatform
import control


if __name__ == '__main__':
    # 开启平台SDK
    # 设置服务器访问地址
    serverUrl = 'https://web.simu.widc.icvrc.cn/api/'
    # 设置登录用户名
    username = 'QTDS_jsfz_jc'
    # 设置登录密码
    password = 'xxxxxx'
    result = ADCPlatform.start(serverUrl, username, password)

    if result:
        print("算法接入成功！")
        print("启动任务")
        ADCPlatform.start_task()

        # 此时使用决赛时候的代码，设置对应的Flag=True
        final_task = True

        if final_task:
            # 创建任务列表
            task_list = ['前车制动至停车', '行人从视觉盲区横穿马路', '曲线道路，前方多车阻碍', '自动驾驶']
        else:
            task_list = ['前车静止', '行人横穿马路', '曲线道路，前车阻碍', '自动驾驶']

        # 选择任务的序号
        task_num = 2
        task = task_list[task_num]

        print("========================")
        print("选择任务{}: {}".format(task_num,task_list[task_num]))
        print("========================")

        # 根据不同任务执行不同的算法
        if task_num == 0:
            control.run(0, final_task=final_task)
        elif task_num == 1:
            control.run(1, final_task=final_task)
        elif task_num == 2:
            control.run(2, final_task=final_task)
        else:
            print("没有'{}'任务,请将任务序号修改为0,1或2".format(task))

        # 停止平台
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()
