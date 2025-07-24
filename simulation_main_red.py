from CommunicationTool import *
import math
import numpy as np
import KU_v18 as KU
import defensexample as DE
#生成控制指令（参考案例）
#flag=1#蛇形机动flag
jidong_time = [100000,150000,160000,170000,180000,190000,200000]
# flagDefence=[1,1,1,1]#防御flag
# flag2Defence=[1,1,1,1]#防御flag
# DefenseStage=[0,0,0,0]#每个飞机的防御阶段
# DefenseDeg=[0,0,0,0]#每个飞机的防御航向
# DefenseMode=[0,0,0,0]#0侧面导弹，1正面导弹,2导弹过于分散,3两导弹
# DefenseJudge=[0,0,0,0]#0未判断，1已经判断
def create_action_cmd(info, step_num):
    global flag
    output_cmd = SendData()
    RD=KU.RD()
    Ob=KU.Obstacle(info,123.9,42,0,20000,30000)
    JDDZ=KU.JDDZ(output_cmd,info)
    if (step_num <= jidong_time[0]): 
        if Ob.is_inside(info.DroneID):
            print(info.DroneID,"进入威胁区")
        if info.isMisWarning == True :
            output_cmd.sPlaneControl.isApplyNow = True
            DroneID=info.DroneID
            plane_Yaw=info.Yaw
            DE.DefenseAction(output_cmd,info,DroneID,plane_Yaw)
        elif info.isMisWarning == False :
            DroneID=info.DroneID
            DE.DefenseStage[int((DroneID/100000)-1)]=0
            DE.DefenseDeg[int((DroneID/100000)-1)]=0
            DE.DefenseMode[int((DroneID/100000)-1)]=0
            DE.flagDefence[int((DroneID/100000)-1)]=1
            DE.flag2Defence[int((DroneID/100000)-1)]=1
            DE.DefenseJudge[int((DroneID/100000)-1)]=0
            DE.DefenseAlt[int((DroneID/100000)-1)]=0
            TargetID=KU.GetTargetID(info,DroneID)
            KU.APF_Valpha(output_cmd,info,DroneID,TargetID,110,130,40,45,123.9,42,0,20000,1.0,130,3.0,300,900000,0.05)
            # if 0<min(info.FoundEnemyList[i].TargetDis for i in range(len(info.FoundEnemyList)))<230000 :#执行攻击     
            # else:
            #     output_cmd.sPlaneControl.isApplyNow = False   
        
    # elif(step_num<=jidong_time[1]):
    #     output_cmd = SendData()
        
    # elif(step_num<=jidong_time[2]):
    #     output_cmd = SendData()
    #     output_cmd.sPlaneControl.CmdIndex = 3
    #     output_cmd.sPlaneControl.CmdID = 1
    #     output_cmd.sPlaneControl.VelType = 0
    #     output_cmd.sPlaneControl.TurnDirection = 1 
    #     output_cmd.sPlaneControl.CmdHeadingDeg = 0  
    #     if (step_num == jidong_time[2]):
    #         print("stage 3 finish")
    #         output_cmd.sPlaneControl.isApplyNow = False
    #     output_cmd.sPlaneControl.isApplyNow = True
    #     output_cmd.sPlaneControl.CmdPhi = 40
    #     output_cmd.sPlaneControl.CmdSpd = 0.6 
    #     output_cmd.sPlaneControl.CmdNy = 4
    #     output_cmd.sPlaneControl.CmdThrust = 200
    #     output_cmd.sPlaneControl.ThrustLimit = 200
    # # 0.7Ma，1.5过载 4-5秒转10°
    
    # elif (step_num <= jidong_time[3]):
    #     output_cmd = SendData()
    #     output_cmd.sPlaneControl.CmdIndex = 4
    #     output_cmd.sPlaneControl.CmdID = 6
    #     output_cmd.sPlaneControl.VelType = 0
    #     output_cmd.sPlaneControl.TurnDirection = -1 
    #     output_cmd.sPlaneControl.CmdHeadingDeg = 0  
    #     if (step_num == jidong_time[3]):
    #         print("stage 4 finish")
    #         output_cmd.sPlaneControl.isApplyNow = False
    #     output_cmd.sPlaneControl.isApplyNow = True
    #     output_cmd.sPlaneControl.CmdPhi = 40
    #     output_cmd.sPlaneControl.CmdSpd = 0.7
    #     output_cmd.sPlaneControl.CmdNy = 5
    #     output_cmd.sPlaneControl.CmdThrust = 90
    #     output_cmd.sPlaneControl.ThrustLimit = 90
       
    
    # elif(step_num <= jidong_time[4]):
    #     output_cmd = SendData()
        
    #     output_cmd.sPlaneControl.CmdIndex = 5
    #     output_cmd.sPlaneControl.CmdID = 6
    #     output_cmd.sPlaneControl.VelType = 0
    #     output_cmd.sPlaneControl.TurnDirection = -1 
    #     output_cmd.sPlaneControl.CmdHeadingDeg = 30  
    #     if (step_num == jidong_time[4]):
    #         print("stage 5 finish")
    #         output_cmd.sPlaneControl.isApplyNow = False
    #     output_cmd.sPlaneControl.isApplyNow = True
    #     output_cmd.sPlaneControl.CmdPhi = 80
    #     output_cmd.sPlaneControl.CmdSpd = 1.2 
    #     output_cmd.sPlaneControl.CmdNy = 7
    #     output_cmd.sPlaneControl.CmdThrust = 120
    #     output_cmd.sPlaneControl.ThrustLimit = 120
    
    # elif(step_num <= jidong_time[5]):
    #     output_cmd = SendData()
    #     output_cmd.sPlaneControl.CmdIndex = 6
    #     output_cmd.sPlaneControl.CmdID = 6
    #     output_cmd.sPlaneControl.VelType = 0
    #     output_cmd.sPlaneControl.TurnDirection = -1 
    #     output_cmd.sPlaneControl.CmdHeadingDeg = 180  
    #     if (step_num == jidong_time[5]):
    #         print("stage 6 finish")
    #         output_cmd.sPlaneControl.isApplyNow = False
    #     output_cmd.sPlaneControl.isApplyNow = True
    #     output_cmd.sPlaneControl.CmdPhi = 40
    #     output_cmd.sPlaneControl.CmdSpd = 0.7 
    #     output_cmd.sPlaneControl.CmdNy = 7
    #     output_cmd.sPlaneControl.CmdThrust = 120
    #     output_cmd.sPlaneControl.ThrustLimit = 120
    else:
        output_cmd = SendData()
        output_cmd.sPlaneControl.CmdIndex = 7
        output_cmd.sPlaneControl.CmdID = 6
        output_cmd.sPlaneControl.VelType = 0
        output_cmd.sPlaneControl.TurnDirection = -1 
        output_cmd.sPlaneControl.CmdHeadingDeg = 300  
        if (step_num == jidong_time[6]):
            print("stage 7 finish")
            output_cmd.sPlaneControl.isApplyNow = False
        output_cmd.sPlaneControl.isApplyNow = True
        output_cmd.sPlaneControl.CmdPhi = 40
        output_cmd.sPlaneControl.CmdSpd = 1.2 
        output_cmd.sPlaneControl.CmdNy = 7
        output_cmd.sPlaneControl.CmdThrust = 80
        output_cmd.sPlaneControl.ThrustLimit = 80

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
        # if platform.recv_info.AlarmList[0].MisAzi != 0:
        #     print(platform.recv_info.DroneID, ": vars(AlarmList[0])", vars(platform.recv_info.AlarmList[0]))
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
    Port = 60001
    drone_num = 4
    main(IP, Port, drone_num)
