from CommunicationTool import *

# 生成控制指令（参考案例）
jidong_time = [50000]
launchFlag = 1
a = 0


def create_action_cmd(info, step_num):

    global launchFlag
    global a

    if (step_num <= jidong_time[0]):
        output_cmd = SendData()
        output_cmd.sPlaneControl.CmdIndex = 1
        output_cmd.sPlaneControl.CmdID = 1
        output_cmd.sPlaneControl.isApplyNow = True
        output_cmd.sPlaneControl.CmdHeadingDeg = 180
        output_cmd.sPlaneControl.CmdAlt = 10000
        output_cmd.sPlaneControl.CmdSpd = 0.9
        output_cmd.sPlaneControl.TurnDirection = 1
        if(info.DroneID==600000):
            if info.AttackEnemyList[0].EnemyID == 200000:
                if(info.AttackEnemyList[0].NTSstate == 1):
                    output_cmd.sSOCtrl.isNTSAssigned = 1
                    output_cmd.sSOCtrl.NTSEntityIdAssigned = info.AttackEnemyList[0].EnemyID
                    output_cmd.sOtherControl.isLaunch = 0
                elif info.AttackEnemyList[0].NTSstate == 2 and launchFlag == 1:
                    output_cmd.sOtherControl.isLaunch = 1
                    launchFlag = 0
    else:
        output_cmd = SendData()
        output_cmd.sPlaneControl.CmdIndex = 7
        output_cmd.sPlaneControl.CmdID = 1
        output_cmd.sPlaneControl.VelType = 0
        output_cmd.sPlaneControl.CmdSpd = 0.9
        output_cmd.sPlaneControl.isApplyNow = True
        output_cmd.sPlaneControl.CmdHeadingDeg = 180
        output_cmd.sPlaneControl.CmdAlt = 5000

    return output_cmd


# 规整上升沿
def check_cmd(cmd, last_cmd):
    # if last_cmd is None:
    #     cmd.sPlaneControl.isApplyNow = False
    #     cmd.sOtherControl.isLaunch = 0
    #     cmd.sSOCtrl.isNTSAssigned = 0
    # else:
    #     if cmd.sPlaneControl == last_cmd.sPlaneControl:
    #         cmd.sPlaneControl.isApplyNow = False
    #     if cmd.sSOCtrl == last_cmd.sSOCtrl:
    #         cmd.sSOCtrl.isNTSAssigned = 0
    return cmd


# 获取传输数据，生成对应无人机command指令，并传输指令逻辑
def solve(platform, plane):
    global save_last_cmd

    if platform.step > save_last_cmd[plane][1]:
        # if platform.recv_info.MissileTrackList[0].WeaponID != 0:
        #     print(platform.recv_info.DroneID, ":  vars(MissileTrackList[0])", vars(platform.recv_info.MissileTrackList[0]))
        cmd_created = create_action_cmd(platform.recv_info, platform.step)  # 生成控制指令
        # 保存上一个发送的指令
        save_last_cmd[plane][0] = cmd_created  # 更新保存指令

        cmd_created = check_cmd(cmd_created, save_last_cmd[plane][0])  # 比较得到上升沿
        platform.cmd_struct_queue.put(cmd_created)  # 发送数据
        save_last_cmd[plane][1] = save_last_cmd[plane][1] + 1


def main(IP, Port, drone_num):
    data_serv = DataService(IP, Port, drone_num)  # 本机IP与设置的端口，使用config文件
    data_serv.run()  # 启动仿真环境

    global save_last_cmd  # 用于比较指令变化的字典全局变量
    save_last_cmd = {}

    for plane in data_serv.platforms:  # 初始化全局变量为None
        save_last_cmd[plane] = [None, 0]

    while True:  # 交互循环
        try:
            for plane in data_serv.platforms:
                solve(data_serv.platforms[plane], plane)  # 处理信息
                # print(plane, "'s step is  ", data_serv.platforms[plane].step)
        except Exception as e:
            print("Error break", e)
            break
    data_serv.close()


if __name__ == "__main__":
    IP = "192.168.50.173"
    Port = 60010
    drone_num = 4
    main(IP, Port, drone_num)

