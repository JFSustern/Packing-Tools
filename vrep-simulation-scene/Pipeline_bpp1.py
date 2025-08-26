import os
import numpy as np
from simulation import vrep
from simulation import vrepConst as vc
import time
import dataprocessing_bpp1 as boxes
import threading
from matplotlib import colors


class Robot(object):
    def __init__(self, connectIp, connectPort, objectMeshDir = '', objectNumber = 0, workspaceLimits = [[]],**kwargs):

        ### Connect
        self.ip   = connectIp
        self.port = connectPort

        ### Object
        ### Define colors for object meshes (Tableau palette)
        self.colorSpace = np.asarray([[78.0, 121.0, 167.0], # blue
                                       [89.0, 161.0, 79.0], # green
                                       [156, 117, 95], # brown
                                       [242, 142, 43], # orange
                                       [237.0, 201.0, 72.0], # yellow
                                       [186, 176, 172], # gray
                                       [255.0, 87.0, 51.0], # red
                                       [176, 122, 161], # purple
                                       [118, 183, 178], # cyan
                                       [255, 157, 167], # pink
                                       [0, 0, 0], # black
                                       [0, 0, 255], # pure blue
                                       [0, 255, 0], # pure green
                                       [255, 0, 0], # pure red
                                       [0, 255, 255], # pure cyan
                                       [255, 255, 0], # pure yellow
                                       [255, 0, 255], # magenta
                                       [255, 255, 255], # white
                                      ])/255.0

        ### Read files in object mesh directory
        self.objectMeshDir = objectMeshDir
        self.objectNumber = objectNumber
        try:
            if objectMeshDir and os.path.exists(objectMeshDir):
                self.meshList = os.listdir(self.objectMeshDir)
                ### Randomly choose objects to add to scene
                self.objectMeshIndex = np.random.randint(0, len(self.meshList), size=self.objectNumber)
                self.objectMeshColor = self.colorSpace[np.asarray(range(self.objectNumber)) % len(self.colorSpace), :]
            else:
                self.meshList = None
                if objectNumber == 0:
                    self.cerr("No mesh directory specified, using createPureShape instead", 1)
                else:
                    self.cerr("Mesh directory not found, using createPureShape instead", 1)
        except Exception as e:
            self.meshList = None
            if objectNumber == 0:
                self.cerr("Failed to read mesh, detected objectNumber = 0, no need to read mesh, you can ignore this warning", 1)
            else:
                self.cerr(f"Failed to read mesh: {e}, using createPureShape instead", 1)

        ### Workspace limits
        self.workspaceLimits = workspaceLimits

        self.targetName = kwargs['targetName']
        self.openParentObjectName = kwargs['openParentObjectName']
        self.closeParentObjectName = kwargs['closeParentObjectName']
        self.suckerName = kwargs['suckerName']

    def cerr(self, context, type):
        if type == 0:
            print("\033[1;31mError\033[0m: %s"%context)
        elif type == 1:
            print("\033[1;33mWarning\033[0m: %s"%context)

    # connect
    def connect(self):

        vrep.simxFinish(-1)  # close all opened connections
        self.clientID = vrep.simxStart(self.ip, self.port, True, True, 3000, 5)
        ### Connect to V-REP
        if self.clientID == -1:
            import sys
            sys.exit('\nV-REP remote API server connection failed (' + self.ip + ':' +
                     str(self.port) + '). Is V-REP running?')
        else:
            print('V-REP remote API server connection success')
        return

    def disconnect(self):

        ### Make sure that the last command sent has arrived
        vrep.simxGetPingTime(self.clientID)
        ### Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)
        return

    def listDistance(self, src, tar):
        return np.sqrt((src[0]-tar[0])**2 + (src[1]-tar[1])**2 + (src[2]-tar[2])**2)

    def start(self):

        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        print('''==========================================\n========== simulation start ==============\n==========================================\n''')

    def stop(self):

        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        print('''==========================================\n========== simulation stop ===============\n==========================================\n''')

    def restart(self):

        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        print('''==========================================\n========== simulation stop ===============\n==========================================\n''')
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        print('''==========================================\n========== simulation start ==============\n==========================================\n''')


    def createPureShape(self, toolShape, targetPosition, mass = 0.01):

        # the name of the scene object where the script is attached to, or an empty string if the script has no associated scene object
        scriptDescription = 'remoteApiCommandServer'
        # the name of the Lua function to call in the specified script

        retResp, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self.clientID, scriptDescription, vrep.sim_scripttype_childscript,
                            'CreatePureShape',
                            [],
                            [toolShape[0], toolShape[1], toolShape[2], targetPosition[0], targetPosition[1], targetPosition[2], mass],
                            [],
                            bytearray(),
                            vrep.simx_opmode_blocking)
        return retInts[0]

    def setColor(self,objHandle,color):
        scriptDescription = 'remoteApiCommandServer'
        retResp, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self.clientID, scriptDescription, vrep.sim_scripttype_childscript,
                            'SetColor',
                            [objHandle],
                            color,
                            [],
                            bytearray(),
                            vrep.simx_opmode_blocking)


    def untilReach(self,tar, targetHandle):
        pass
        time.sleep(0.08)
        # simRet, targetPosition = vrep.simxGetObjectPosition(self.clientID, targetHandle, -1, vrep.simx_opmode_blocking)
        # print('targetPosition:', targetPosition)
        # while True:
        #     simRet, currPosition = vrep.simxGetObjectPosition(self.clientID, targetHandle, -1,
        #                                                         vrep.simx_opmode_blocking)
        #     simRet, targetPosition = vrep.simxGetObjectPosition(self.clientID, targetHandle, -1, vrep.simx_opmode_blocking)
        #     print('targetPosition:', targetPosition)
        #     if self.listDistance(currPosition, tar) < 1e-3:
        #         break

    ### move and gripper
    def move(self, targetName, toolPosition, toolOrientation, moveSpeed=50, turnSpeed=50,sleepTime = 0.04):
        while True:
            simRet, targetPosition = vrep.simxGetObjectPosition(self.clientID, self.targetHandle,-1,vrep.simx_opmode_blocking)
            # print([simRet, targetPosition])
            if simRet == 0:
                break

        simRet, targetOrientation = vrep.simxGetObjectOrientation(self.clientID, self.targetHandle,-1,vrep.simx_opmode_blocking)
        originPosition = targetPosition

        if toolPosition != None:
            moveDirection = np.asarray([toolPosition[0] - originPosition[0], toolPosition[1] - originPosition[1], toolPosition[2] - originPosition[2]])
            moveMagnitude = np.linalg.norm(moveDirection)
            moveStep = 1 / moveSpeed * moveDirection / moveMagnitude
            moveStepNumber = int(np.floor(moveMagnitude * moveSpeed))

            for stepIter in range(moveStepNumber):
                tempTarget = [originPosition[0] + moveStep[0] * (stepIter + 1),
                              originPosition[1] + moveStep[1] * (stepIter + 1),
                              originPosition[2] + moveStep[2] * (stepIter + 1)]

                vrep.simxSetObjectPosition(self.clientID, self.targetHandle, -1, tempTarget, vrep.simx_opmode_blocking)
                self.untilReach(tempTarget, self.targetHandle)

            vrep.simxSetObjectPosition(self.clientID, self.targetHandle, -1, (toolPosition[0], toolPosition[1], toolPosition[2]), vrep.simx_opmode_blocking)

        if toolOrientation != None:
            turnDirection = np.asarray([toolOrientation[0] - targetOrientation[0], toolOrientation[1] - targetOrientation[1], toolOrientation[2] - targetOrientation[2]])
            turnMagnitude = np.linalg.norm(turnDirection)
            turnStep = 1 / turnSpeed * turnDirection / turnMagnitude
            turnStepNumber = int(np.floor(turnMagnitude * turnSpeed))

            for stepIter in range(turnStepNumber):
                ### turn
                vrep.simxSetObjectOrientation(self.clientID, self.targetHandle, -1, (targetOrientation[0] + turnStep[0], targetOrientation[1] + turnStep[1], targetOrientation[2] + turnStep[2]), vrep.simx_opmode_blocking)
                simRet, targetOrientation = vrep.simxGetObjectOrientation(self.clientID, self.targetHandle, -1, vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(self.clientID, self.targetHandle, -1, (toolOrientation[0], toolOrientation[1], toolOrientation[2]), vrep.simx_opmode_blocking)

    def openSucker(self, objectName, parentObjectName, boxHandle=None):
        try:
            simRet, objectHandle = vrep.simxGetObjectHandle(self.clientID, objectName, vrep.simx_opmode_blocking)
            if simRet != 0:
                print(f"警告: 无法获取对象 {objectName} 的句柄")
                return
                
            simRet, parentObject = vrep.simxGetObjectHandle(self.clientID, parentObjectName, vrep.simx_opmode_blocking)
            if simRet != 0:
                print(f"警告: 无法获取父对象 {parentObjectName} 的句柄")
                return
                
            print(f"打开吸盘: {objectName} -> {parentObjectName}")
            if parentObject != -1:
                for _ in range(10):
                    flag = vrep.simxSetObjectParent(self.clientID, objectHandle, parentObject, True, vrep.simx_opmode_blocking)
                    if flag == 0:
                        print("吸盘打开成功")
                        break
                    else:
                        print(f"吸盘打开失败，错误码: {flag}")
            else:
                print("打开父对象无效，跳过父子关系设置")
            
            # 如果提供了物块句柄，尝试释放物块
            if boxHandle is not None:
                print(f"尝试释放物块 {boxHandle}")
                # 释放瞬间将物块设为动态并可响应碰撞，设置碰撞掩码与质量
                vrep.simxSetObjectIntParameter(self.clientID, boxHandle, vc.sim_shapeintparam_static, 0, vrep.simx_opmode_blocking)
                vrep.simxSetObjectIntParameter(self.clientID, boxHandle, vc.sim_shapeintparam_respondable, 1, vrep.simx_opmode_blocking)
                vrep.simxSetObjectIntParameter(self.clientID, boxHandle, vc.sim_shapeintparam_respondable_mask, 0xFFFF, vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self.clientID, boxHandle, vc.sim_shapefloatparam_mass, 0.2, vrep.simx_opmode_blocking)
                # 将物块设置为世界坐标系的子对象（释放）
                for _ in range(10):
                    flag = vrep.simxSetObjectParent(self.clientID, boxHandle, -1, True, vrep.simx_opmode_blocking)
                    if flag == 0:
                        print("物块成功释放")
                        break
                    else:
                        print(f"物块释放失败，错误码: {flag}")
            
            time.sleep(0.01)
        except Exception as e:
            print(f"打开吸盘时出错: {e}")

    def closeSucker(self, objectName, parentObjectName, boxHandle=None):
        try:
            simRet, objectHandle = vrep.simxGetObjectHandle(self.clientID, objectName, vrep.simx_opmode_blocking)
            if simRet != 0:
                print(f"警告: 无法获取对象 {objectName} 的句柄")
                return
                
            simRet, parentObject = vrep.simxGetObjectHandle(self.clientID, parentObjectName, vrep.simx_opmode_blocking)
            if simRet != 0:
                print(f"警告: 无法获取父对象 {parentObjectName} 的句柄")
                return
                
            print(f"关闭吸盘: {objectName} -> {parentObjectName}")
            for _ in range(10):
                flag = vrep.simxSetObjectParent(self.clientID, objectHandle, parentObject, False, vrep.simx_opmode_blocking)
                if flag == 0:
                    print("吸盘关闭成功")
                    break
                else:
                    print(f"吸盘关闭失败，错误码: {flag}")
            
            # 如果提供了物块句柄，尝试将物块设置为吸盘的子对象
            if boxHandle is not None:
                print(f"尝试将物块 {boxHandle} 设置为吸盘的子对象 (父: {parentObjectName})")
                # 为避免与物理引擎冲突：抓取阶段设置为静态且不可响应
                vrep.simxSetObjectIntParameter(self.clientID, boxHandle, vc.sim_shapeintparam_static, 1, vrep.simx_opmode_blocking)
                vrep.simxSetObjectIntParameter(self.clientID, boxHandle, vc.sim_shapeintparam_respondable, 0, vrep.simx_opmode_blocking)
                # 将物块直接附到 closeSucker 的父对象（例如 suctionPadLink）
                for _ in range(10):
                    flag = vrep.simxSetObjectParent(self.clientID, boxHandle, parentObject, True, vrep.simx_opmode_blocking)
                    if flag == 0:
                        print("物块成功附加到吸盘父链接")
                        break
                    else:
                        print(f"物块附加失败，错误码: {flag}")
            
            time.sleep(0.01)
        except Exception as e:
            print(f"关闭吸盘时出错: {e}")

    def robotInit(self):
        simRet, self.targetHandle = vrep.simxGetObjectHandle(self.clientID, self.targetName, vrep.simx_opmode_blocking)
        simRet, tipHandle = vrep.simxGetObjectHandle(self.clientID, tipName, vrep.simx_opmode_blocking)
        simRet, tipPosition = vrep.simxGetObjectPosition(self.clientID, tipHandle, -1, vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.targetHandle, -1, tipPosition, vrep.simx_opmode_blocking)

        self.forwarder = vrep.simxGetObjectHandle(self.clientID, 'customizableConveyor_forwarder', vrep.simx_opmode_blocking)[1]
        simRet, currentSensor = vrep.simxGetObjectHandle(self.clientID, 'Current', vrep.simx_opmode_blocking)
        simRet, previewSensor = vrep.simxGetObjectHandle(self.clientID, 'Preview', vrep.simx_opmode_blocking)
        simRet, self.currentSensorPosition = vrep.simxGetObjectPosition(self.clientID, currentSensor, -1, vrep.simx_opmode_blocking)
        simRet, self.previewSensorPosition = vrep.simxGetObjectPosition(self.clientID, previewSensor, -1, vrep.simx_opmode_blocking)
        
        # 验证吸盘对象是否存在
        print("验证吸盘对象...")
        simRet, suckerHandle = vrep.simxGetObjectHandle(self.clientID, self.suckerName, vrep.simx_opmode_blocking)
        if simRet == 0:
            print(f"✓ 吸盘对象 '{self.suckerName}' 存在")
        else:
            print(f"✗ 吸盘对象 '{self.suckerName}' 不存在，错误码: {simRet}")
            
        simRet, openParentHandle = vrep.simxGetObjectHandle(self.clientID, self.openParentObjectName, vrep.simx_opmode_blocking)
        if simRet == 0:
            print(f"✓ 打开父对象 '{self.openParentObjectName}' 存在")
        else:
            print(f"✗ 打开父对象 '{self.openParentObjectName}' 不存在，错误码: {simRet}")
            # 回退：使用关闭父对象作为打开父对象，避免后续卡住
            self.openParentObjectName = self.closeParentObjectName
            print(f"使用回退父对象进行打开动作: '{self.openParentObjectName}'")
            
        simRet, closeParentHandle = vrep.simxGetObjectHandle(self.clientID, self.closeParentObjectName, vrep.simx_opmode_blocking)
        if simRet == 0:
            print(f"✓ 关闭父对象 '{self.closeParentObjectName}' 存在")
        else:
            print(f"✗ 关闭父对象 '{self.closeParentObjectName}' 不存在，错误码: {simRet}")

    def dropIt(self,currPosition, targetPosition, box, boxHandle, maxHeight):
        try:
            print(f"开始吸取物块，当前位置: {currPosition}")
            
            # 移动到物块上方
            self.move(self.targetName, [currPosition[0], currPosition[1], 0.35], None)
            print("已移动到物块上方")

            # 下降到物块表面进行吸取
            suckHeight = currPosition[2] + box[2] * 0.5 - 0.09 + 0.01  # 恢复到原始计算
            print(f'下降到吸取高度: {suckHeight}')
            self.move(self.targetName, [currPosition[0], currPosition[1], suckHeight], None)
            
            # 打开吸盘准备吸取
            print("打开吸盘")
            self.openSucker(self.suckerName, self.openParentObjectName)
            time.sleep(0.2)  # 等待吸盘打开
            
            # 关闭吸盘进行吸取
            print("关闭吸盘进行吸取")
            self.closeSucker(self.suckerName, self.closeParentObjectName, boxHandle)
            time.sleep(0.2)  # 等待吸取完成
            
            # 检查是否成功吸取
            simRet, newPosition = vrep.simxGetObjectPosition(self.clientID, boxHandle, -1, vrep.simx_opmode_blocking)
            print(f"吸取后物块位置: {newPosition}")
            
            # 提起物块
            print("提起物块")
            self.move(self.targetName, [currPosition[0], currPosition[1], 0.35], None)
            time.sleep(0.1)
            
            # 不再做位置同步 hack，依赖真实父子关系与物理属性切换
            
            # 再次检查物块是否跟随
            simRet, checkPosition = vrep.simxGetObjectPosition(self.clientID, boxHandle, -1, vrep.simx_opmode_blocking)
            print(f"提起后物块位置: {checkPosition}")
            
            # 移动到目标位置上方
            print(f"移动到目标位置上方: {targetPosition}")
            self.move(self.targetName, [targetPosition[0], targetPosition[1], maxHeight + 0.1], None)
            
            # 下降到放置位置
            placeHeight = targetPosition[2] + box[2] * 0.5 - 0.09 + 0.01
            print(f"下降到放置高度: {placeHeight}")
            self.move(self.targetName, [targetPosition[0], targetPosition[1], placeHeight], None)

            if targetPosition[2] + box[2] * 0.5 > maxHeight:
                maxHeight = targetPosition[2] + box[2] * 0.5

            # 打开吸盘释放物块
            print("释放物块")
            self.openSucker(self.suckerName, self.openParentObjectName, boxHandle)
            time.sleep(0.2)  # 等待释放完成
            
            # 设置物块颜色表示已放置
            self.setColor(boxHandle, color=colors.hex2color(colors.cnames['lightskyblue']))
            
            # 提起机械臂
            self.move(self.targetName, [targetPosition[0], targetPosition[1], maxHeight + 0.1], None)
            
            # 检查物块是否成功放置
            simRet, finalPosition = vrep.simxGetObjectPosition(self.clientID, boxHandle, -1, vrep.simx_opmode_blocking)
            print('物块最终位置: ', finalPosition, '目标位置: ', targetPosition)
            print('位置误差 X: %.3f, Y: %.3f' % (targetPosition[0] - finalPosition[0], targetPosition[1] - finalPosition[1]))
            
            return maxHeight
        except Exception as e:
            print(f"dropIt 方法出错: {e}")
        return maxHeight

    def clearBox(self, handleList):
        for boxHandle in handleList:
            vrep.simxRemoveObject(self.clientID, boxHandle, vrep.simx_opmode_blocking)

    def resetGame(self, handleList):
        self.clearBox(handleList)
        handleList = []
        self.boxLocation = []
        self.boxShape = []



### connect demo
connectIp = '127.0.0.1'
connectPort = 19997

objectMeshDir = ''  # 设置为空字符串，因为我们使用 createPureShape 创建物块
objectNumber = 10

targetName = 'Target'
tipName = 'Tip'
openCloseJointName = 'RG2_openCloseJoint'
speed = 0.02

objectName = 'suctionPadLoopClosureDummy1'
closeParentObjectName = 'suctionPadLink'
openParentObjectName = 'suctionPad'

# add cuboid
boxes = boxes.newBoxs
boxesNum = 0
for box in boxes:
    if box[3] != -1:
        boxesNum+=1
loss = 0.0

# Cols: min max, Rows: x y z (define workspace limits in robot coordinate)
workspaceLimits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])

robot = Robot(connectIp, connectPort, objectMeshDir, objectNumber, workspaceLimits,suckerName = objectName,
              openParentObjectName = openParentObjectName, closeParentObjectName = closeParentObjectName, targetName = targetName)

robot.connect()
robot.robotInit()

### start stop restart demo
robot.start()
simRet, tipHandle = vrep.simxGetObjectHandle(robot.clientID, tipName, vrep.simx_opmode_blocking)
simRet, tipPosition = vrep.simxGetObjectPosition(robot.clientID, tipHandle, -1, vrep.simx_opmode_blocking)
tipPosition[0] = tipPosition[0] + 0.2
vrep.simxSetObjectPosition(robot.clientID, robot.targetHandle, -1, tipPosition, vrep.simx_opmode_blocking)


nameList = [] # global
targetposition_list = [] # global
handleList = []
initX = 2.8
toPack = 0

# 着色可以在这里写
def addBox():
    toProduce = 0
    boxHandle  = None
    lastPosition = [0, 0, 0]  # global

    toColorList = []
    global targetposition_list
    global nameList
    global handleList

    toColorHandle = None

    while toProduce < boxesNum:
        time.sleep(0.13)

        if boxHandle is not None:
            simRet, lastPosition = vrep.simxGetObjectPosition(robot.clientID, boxHandle, -1, vrep.simx_opmode_blocking)

        # 修复循环逻辑：只有在满足条件时才生成新物块
        if toProduce == 0 or (boxHandle is not None and abs(lastPosition[0] - initX) > 0.3 and abs(lastPosition[0] - initX) < 1):
            if toProduce == 0 or (lastPosition[0] != 0):  # 修复条件判断
                shape = [boxes[toProduce][0], boxes[toProduce][1], boxes[toProduce][2]]
                targetPosition = [boxes[toProduce][3] + boxes[toProduce][0] * 0.5, boxes[toProduce][4] + boxes[toProduce][1] * 0.5, boxes[toProduce][5] + boxes[toProduce][2] * 0.5]
                targetposition_list.append(targetPosition)

                initPosition = [initX, -0.85, 0.01 + boxes[toProduce][2] / 2]
                scaling = 0.93
                boxHandle = robot.createPureShape([shape[0] * scaling, shape[1] * scaling, shape[2]], initPosition)
                boxName = 'Cuboid' if toProduce == 0 else 'Cuboid{}'.format(toProduce - 1)
                nameList.append(boxName)
                handleList.append(boxHandle)
                toColorList.append(boxHandle)
                # robot.setColor(boxHandle, color=colors.hex2color(colors.cnames['pink']))
                toProduce += 1
                print(f'生成物块 {toProduce-1}，句柄: {boxHandle}')

        print('toProduce:', toProduce)
    
    print("所有物块已生成完成")

t = threading.Thread(target=addBox)
t.start()

beltStop = False
time.sleep(0.5)

maxHeight = 0
lastPostion = None
while toPack < boxesNum:
    time.sleep(0.1)
    singnal = False
    if len(handleList) == 0:
        continue
    else:
        curr = handleList[toPack]

    simRet, currPosition = vrep.simxGetObjectPosition(robot.clientID, curr, -1, vrep.simx_opmode_blocking)
    if abs(currPosition[0] - robot.currentSensorPosition[0]) > 0.2:
        continue

    if lastPostion is not None:
        if abs(currPosition[0] - lastPostion[0]) <= 1e-3 and np.sum(currPosition)!=0:
            singnal = True

    lastPostion = currPosition


    if singnal:
        print(f"开始处理物块 {toPack}")
        targetPosition = targetposition_list[toPack]
        curr = handleList[toPack]
        for _ in range(10):
            simRet, currPosition = vrep.simxGetObjectPosition(robot.clientID, curr, -1, vrep.simx_opmode_blocking)
            print([toPack,simRet,currPosition])
            if abs(currPosition[0] - robot.currentSensorPosition[0]) <= 0.2:
                break

        print(f"物块 {toPack} 当前位置: {currPosition}")
        print(f"物块 {toPack} 目标位置: {targetPosition}")
        print(f"物块 {toPack} 尺寸: {boxes[toPack]}")

        maxHeight = robot.dropIt(currPosition, targetPosition, boxes[toPack], curr, maxHeight)

        print(f"物块 {toPack} 处理完成，当前最大高度: {maxHeight}")

        toPack += 1
        lastPostion = None

print("所有物块打包完成")
robot.stop()

