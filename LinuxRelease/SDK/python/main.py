#!/bin/bash
import sys
import math
from collections import defaultdict

robot_allocation_lists = []  # 机器人分配任务列表初始化，key为机器人编号


def judge_workst_produce(workst):
    sum_w7 = 0
    for woks in workstations:
        if woks.wid == 7:
            sum_w7 = sum_w7 + 1
    if workst.wid == 1 or workst.wid == 2 or workst.wid == 3:
        # 如果此工作台是1 2 3则返回True
        return True
    if workst.wid == 4 or workst.wid == 5 or workst.wid == 6 or workst.wid == 7:
        if sum_w7 != 1:
            if workst.product_slot != 1 and workst.remaining_time <= 0:
                # 如果此工作台是4 5 6 且
                return True
        else:
            if workst.product_slot != 1:
                # 如果此工作台是4 5 6 且
                return True
    if workst.wid == 8 or workst.wid == 9:
        return True
    return False


class Workstation:
    def __init__(self, wid, position, remaining_time, material_slot, product_slot, lock):
        self.wid = wid  # 工作台ID
        self.position = position  # 工作台位置
        self.remaining_time = remaining_time  # 剩余生产时间
        self.material_slot = material_slot  # 原材料格状态
        self.product_slot = product_slot  # 产品格状态
        self.lock = lock  # 产品格状态

    def distance_to(self, workstation_position):
        return ((self.position[0] - workstation_position[0]) ** 2 + (
                self.position[1] - workstation_position[1]) ** 2) ** 0.5

    def find_latest_workstation(self, target_wid):
        # 找最近的工作台 如果已经生产好的，则返回最近的生产好的工作台，如果都未生产好，则返回最近的不在生产中的工作台
        near_tagets = []
        produce_tagets = []
        produce_over = False

        for workst in workstations:
            if workst.wid == target_wid :
                near_tagets.append(workst)
            if workst.wid == target_wid and workst.product_slot == 1:
                produce_tagets.append(workst)
                produce_over = True
        min_distance = 100000
        min_workstation1 = Workstation(wid=None, position=None, remaining_time=None, material_slot=None,
                                       product_slot=None, lock=False)
        min_workstation2 = Workstation(wid=None, position=None, remaining_time=None, material_slot=None,
                                       product_slot=None, lock=False)
        workst2withproduce = []

        for workst2 in near_tagets:
            if workst2.material_slot & 0b1111111 != 0:
                workst2withproduce.append(workst2)

        if len(workst2withproduce)>0:
            for workst2withpi in workst2withproduce:
                distance = self.distance_to(workst2withpi.position)
                if distance < min_distance:
                    min_distance = distance
                    min_workstation1 = workst2withpi
        else:
            for workst2 in near_tagets:
                distance = self.distance_to(workst2.position)
                if distance < min_distance:
                    min_distance = distance
                    min_workstation1 = workst2

            # distance = self.distance_to(workst2.position)
            # if distance < min_distance:
            #     min_distance = distance
            #     min_workstation1 = workst2

        min_distance2 = 100000
        for workst3 in produce_tagets:
            distance = self.distance_to(workst3.position)
            if distance < min_distance2:
                min_distance2 = distance
                min_workstation2 = workst3

        # sys.stderr.write(str(min_workstation2.position))
        # sys.stderr.write(str(min_workstation1.position))
        # sys.stderr.write(str(produce_over))
        if produce_over:
            return min_workstation2, produce_over
        else:
            return min_workstation1, produce_over


class robot_act:
    def __init__(self, robot_id, actions, param):
        self.robot_id = robot_id  # 机器人ID
        self.actions = actions  # 机器人指令
        self.param = param  # 指令参数


class robot_allocation_list:
    def __init__(self, ra_id, local_place, dst_place, state):
        self.ra_id = ra_id  # 机器人id
        self.local_place = local_place  # 当前地址
        self.dst_place = dst_place  # 目的地址
        self.state = state  # 机器人状态

    def allocate_task(self, local_place, dst_place):
        self.local_place = local_place
        self.dst_place = dst_place
        robot_allocation_lists[self.ra_id].state = True
        robots[self.ra_id].state = True


class Robot:
    def __init__(self, rid, workstation_id, carry_type, time_value, collision_value, angular_velocity, linear_velocity,
                 position, orientation, state):
        self.rid = rid
        self.workstation_id = workstation_id  # 机器人所在工作台的ID
        self.carry_type = carry_type  # 机器人携带物品的类型
        self.time_value = time_value  # 时间价值系数
        self.collision_value = collision_value  # 碰撞价值系数
        self.angular_velocity = angular_velocity  # 二维向量描述角速度,单位 弧度/秒
        self.velocity = linear_velocity  # 二维向量描述线速度，单位：米/秒
        self.position = position  # 二维向量描述机器人坐标，单位：米
        self.orientation = orientation  # 弧度，范围[-π,π]

        self.state = state

    # def update_position(self, time):
    #     self.position = (self.position[0] + self.velocity[0] * time, self.position[1] + self.velocity[1] * time)
    def distance_to(self, workstation):
        return ((self.position[0] - workstation.position[0]) ** 2 + (
                self.position[1] - workstation.position[1]) ** 2) ** 0.5


class RobotCommand:
    """_summary_
    机器人指令类，用来存储需要输出的机器人指令

    input:

    command_type: 指令
    robot_id: 机器人ID
    command_param：参数

    Returns:

    """

    def __init__(self, command_type=None, robot_id=None, command_param=None):
        self.robot_id = robot_id
        self.command_type = command_type
        self.command_param = command_param


# 返回工作台ID为输入ID的所有坐标list
def get_wid_placelists(id):
    wid_placelists = []
    for i in workstations:
        if i.wid == id:
            wid_placelists.append(i)
    return wid_placelists


def init():
    """
    Returns:
        map: 二维list, map[i][j]表示i行j列初始位置的内容
        robots: dict, 机器人对象的字典，key为机器人的编号
        workstations: dict, 工作台对象的字典，key为工作台的编号
    """
    maps = []  # 整张地图
    robots = []  # 机器人对象的字典，key为机器人的编号
    workstations = []  # 工作台对象的字典，key为工作台的编号

    # 初始化整张地图的位置
    for i in range(100):
        line = sys.stdin.readline().strip()
        now_line = []
        for j, c in enumerate(line):
            now_line.append(c)
            if c != '.':
                pos = [0.25 + j * 0.5, 49.75 - i * 0.5]  # 当前点的坐标
                if c == 'A':  # 添加机器人
                    rid = len(robots)  # 机器人编号
                    robot = Robot(rid=rid, workstation_id=None, carry_type=None, time_value=None,
                                  collision_value=None, angular_velocity=[0, 0], linear_velocity=[0, 0],
                                  position=pos, orientation=0, state=False)
                    robots.append(robot)
                elif c.isdigit():  # 添加工作台
                    wid = int(c)  # 工作台编号
                    workstation = Workstation(wid=wid, position=pos, remaining_time=None,
                                              material_slot=None, product_slot=None, lock=False)

                    workstations.append(workstation)

        maps = [now_line] + maps

    for i in range(4):
        robot_list = robot_allocation_list(ra_id=i, local_place=robots[i].position, dst_place=robots[i].position,
                                           state=False)
        robot_allocation_lists.append(robot_list)

    assert input() == 'OK', 'Init Error in init'

    return maps, robots, workstations, robot_allocation_lists



def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


def deadlock(nearest_robot_workstation):
    pass


def next_robot_act(param, param1):
    robot_acts = []

    # 中间要用到robot_act
    # for .... in...
    #     ...
    #     robot_act={
    #          "robot_id" = robot_id  # 机器人ID
    #         ”actions“ = actions  # 机器人指令在{"forward","rotate","buy","sell","destroy"}中选
    #         “param" = param  # 机器人额外参数,具体可参考说明书
    #     }
    # finish()
    # return....


def judge_buy(woks_id):
    for woks in workstations:
        if judge_material(woks, woks_id):
            # 如果缺材料就return True
            return True
    return False


def allout_robots_acts(robots_acts):
    sys.stdout.write(str(frame_id) + "\n")

    for i in range(len(robots_acts)):
        if robots_acts[i].command_type == 'destroy':
            sys.stdout.write(robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + "\n")
            sys.stderr.write(robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + "\n")

            robot_allocation_lists[robots_acts[i].robot_id].state = False
            robots[robots_acts[i].robot_id].state = False
            continue
        if robots_acts[i].command_type == 'buy':
            sys.stdout.write(robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + "\n")

            robot_allocation_lists[robots_acts[i].robot_id].state = False
            robot_allocation_lists[robots_acts[i].robot_id].dst_place = [20, 20]
            robots[robots_acts[i].robot_id].state = False

            workstations[robots[robots_acts[i].robot_id].workstation_id].lock = False


        elif (robots_acts[i].command_type == "sell"):

            if workstations[robots[robots_acts[i].robot_id].workstation_id].product_slot == 1:
                sys.stdout.write(robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + "\n")
                robot_allocation_lists[robots_acts[i].robot_id].state = False
                robot_allocation_lists[robots_acts[i].robot_id].dst_place = [20, 20]
                robots[robots_acts[i].robot_id].state = False
                workstations[robots[robots_acts[i].robot_id].workstation_id].lock = False
                if judge_buy(workstations[robots[robots_acts[i].robot_id].workstation_id]):
                    sys.stdout.write("buy" + " " + str(robots_acts[i].robot_id) + "\n")


            elif (workstations[robots[robots_acts[i].robot_id].workstation_id].product_slot == 0):
                sys.stdout.write(robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + "\n")
                robot_allocation_lists[robots_acts[i].robot_id].state = False
                robot_allocation_lists[robots_acts[i].robot_id].dst_place = [20, 20]
                robots[robots_acts[i].robot_id].state = False

                workstations[robots[robots_acts[i].robot_id].workstation_id].lock = False

        else:
            sys.stdout.write(
                robots_acts[i].command_type + " " + str(robots_acts[i].robot_id) + " " + str(
                    robots_acts[i].command_param) + "\n")

    return 0


# 是否允许程序执行
def allow_run():
    allow_run = 0
    for robot in robots:
        # 每一帧的初始状态都会变化
        robot_allocation_lists[robot.rid].local_place = robot.position
        if not robot.state:
            allow_run = 1

    # 如果所有机器人都在工作态则直接跳过此函数所有
    if allow_run == 0:
        return True
    else:
        return False


def work_posToid(position):
    for i in workstations:
        if i.position == position:
            return i.wid


def lists_taget_define(target_place):
    # 查看任务列表是否有这个目的地，没有则返回True,有返回False
    for i in range(4):
        if robot_allocation_lists[i].dst_place == target_place and (robot_allocation_lists[i].dst_place != [20, 20]):
            return False
    return True

def lists_taget_define2(target_place,fetch_or_post,min_robot_id):
    # 如果是去取货，有不带货的机器人过去，则取冲突
    if fetch_or_post==1:
        for i in range(4):
            if work_posToid(robot_allocation_lists[i].dst_place) == work_posToid(target_place):
                #sys.exit()
                return False
    # 如果是送货，有机器人送自己相同的原材料，则送冲突
    if fetch_or_post == 0:
        for i in range(4):
            if robot_allocation_lists[i].dst_place == target_place and (robots[i].carry_type ==robots[min_robot_id].carry_type) and (robot_allocation_lists[i].dst_place != [20, 20]):
                return False
    return True


def judge_material(workstation, carry_type):
    # 4的原材料为 1 2
    if carry_type == 6:
        carry_type = 0b1000000
    if carry_type == 5:
        carry_type = 0b100000
    if carry_type == 4:
        carry_type = 0b10000
    if carry_type == 3:
        carry_type = 0b1000
    if carry_type == 2:
        carry_type = 0b100
    if carry_type == 1:
        carry_type = 0b10
    if workstation.wid == 4 and (carry_type == 0b10 or carry_type == 0b100):
        if workstation.material_slot & carry_type == 0:
            # 缺他 则返回True
            return True
    # 5的原材料为 1 3
    if workstation.wid == 5 and (carry_type == 0b10 or carry_type == 0b1000):
        if workstation.material_slot & carry_type == 0:
            # 缺他 则返回True
            return True
    # 6的原材料为 2 3
    if workstation.wid == 6 and (carry_type == 0b100 or carry_type == 0b1000):
        if workstation.material_slot & carry_type == 0:
            # 缺他 则返回True
            return True
    # 7的原材料为 4 5 6
    if workstation.wid == 7 and (carry_type == 0b10000 or carry_type == 0b100000 or carry_type == 0b1000000):
        if workstation.material_slot & carry_type == 0:
            # 缺他 则返回True
            return True
    return False


def lock_define(min_robot_id, workstation, fetch_or_post):
    # fetch_or_post为1时取，为0时放
    #if lists_taget_define2(workstation.position,fetch_or_post,min_robot_id):
    if lists_taget_define(workstation.position) and workstation.wid!=9:
        robot_allocation_lists[min_robot_id].allocate_task(robots[min_robot_id].position, workstation.position)


    else:

        # 找一个id相同的workstation去取或者送
        # if fetch_or_post == 1 and (workstation.wid == 4 or workstation.wid == 5 or workstation.wid == 6 or workstation.wid == 7):
            # 若是去取货，找一个id相同的，有货的，任务列表里没有的去取
            # for i in workstations:
            #     if i.wid == workstation.wid and i.product_slot == 1 and lists_taget_define(i.position) == True:
            #         robot_allocation_lists[min_robot_id].allocate_task(robots[min_robot_id].position, i.position)
            #     break

        if fetch_or_post == 1:
            pass
        else:
            #找原材料格不为空，且缺该原材料的去送
            for i in workstations:
                if i.material_slot != 0 and judge_material(i,robots[min_robot_id].carry_type) and lists_taget_define(
                        i.position) == True:
                    robot_allocation_lists[min_robot_id].allocate_task(robots[min_robot_id].position, i.position)
                    break

            # # 若是去送货，找一个id相同，且没有原材料的，任务列表里没有的去取
            # for i in workstations:
            #     if i.wid == workstation.wid and judge_material(i,
            #                                                    robots[min_robot_id].carry_type) and lists_taget_define(
            #             i.position) == True:
            #         robot_allocation_lists[min_robot_id].allocate_task(robots[min_robot_id].position, i.position)
            #         break


def judge_worknum(id):
    count = 0
    count2 = 0

    for i in range(4):
        if (work_posToid(workstations[id].position) == work_posToid(robot_allocation_lists[i].dst_place)):
            count2 += 1

    if count2 > 2:
        return False

    for woks in workstations:
        if woks.material_slot != 0 and woks.wid == id:
            count += 1
    if count > 2:
        return False
    return True


def judge_send(robot, workstation):
    # True代表缺这个材料，False代表不缺
    if robot.carry_type == 1:
        if workstation.wid == 4 or workstation.wid == 5:
            if workstation.material_slot & 0b0010 == 0:
                return True

    if robot.carry_type == 2:
        if workstation.wid == 4 or workstation.wid == 6:
            if workstation.material_slot & 0b100 == 0:
                return True

    if robot.carry_type == 3:
        if workstation.wid == 6 or workstation.wid == 5:
            if workstation.material_slot & 0b1000 == 0:
                return True
    return False


def judge_have(id):
    for woks in workstations:
        if woks.remaining_time > 0 and woks.wid == id:
            return False
    return True


def greedy_algorithm_ergodic_retrieval(have_8):
    # 第一部分 收购工作台部分
    # 先依次检索是否有9、8号工作台，并检索是否有对应的所缺(重点)收购原材料(已生成的)，对情况1，执行过程1，对情况2,执行过程2,对情况3不予执行动作
    # 1.有生产好的物品,在机器人身上
    # 2.有生产好的物品,没去拿
    # 3.没有生产好的物品
    # 过程1: 遍历判断当时所有携带生产好的物品的机器人是否工作态，如工作态则不做干扰，如空闲态则求到目标工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态.
    # 过程2: 对所有空闲态机器人求到目标工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态

    # 判断是否现在所有机器人都在工作态

    if allow_run():
        return False

    # 初始化一个空列表用于存放满足条件的机器人
    eligible_robots = []
    eligible_robots2 = []

    for workstation in workstations:
        if workstation.wid == 8:
            have_8=True
            # 1.检查机器人身上是否有生产好的7号物品
            for robot in robots:
                if robot.carry_type == 7 and robot.state == True:
                    pass
                elif robot.carry_type == 7 and robot.state == False:
                    # 如果机器人满足条件，将其添加到列表中
                    eligible_robots.append(robot)
                else:
                    # 机器人没有生产好的物品
                    pass

            # 过程1
            # 寻找最小距离的robot的id
            if len(eligible_robots) > 0:
                min_distance = 100000
                min_robot_id = 0
                for robot in eligible_robots:
                    distance = robot.distance_to(workstation)
                    if distance < min_distance:
                        min_distance = distance
                        min_robot_id = robot.rid
                # 对最小距离的robot执行任务

                lock_define(min_robot_id, workstation, 0)

            #   判断是否需要继续运行
            if allow_run():
                return have_8

            # 2.检查7号工作台是否生产好了七号物品而没有取
            workstation7, over_produce7 = workstation.find_latest_workstation(7)
            if over_produce7:
                # 有生产好的物品
                # 执行过程2
                for robot in robots:
                    if not robot.state and (
                            robot.carry_type == 0 or judge_material(workstation7, robot.carry_type)):
                        eligible_robots2.append(robot)

                min_distance = 10000
                min_robot_id = 0
                if len(eligible_robots2) > 0:
                    for robot in eligible_robots2:
                        distance = robot.distance_to(workstation7)
                        if distance < min_distance:
                            min_distance = distance
                            min_robot_id = robot.rid
                    # 对最小距离的robot执行任务

                    lock_define(min_robot_id, workstation7, 1)

            # 3. 没有生产好的物品，找4，5，6原材料

            else:

                material = workstation7.material_slot
                if material is not None:
                    if not material & 0b1000000:
                        # 缺少材料6
                        # 情况0有生产好的物品6,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 6 and robot.state == True:
                                pass
                            elif robot.carry_type == 6 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots2_1.append(robot)
                        # 过程1
                        # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b1000000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                            else:
                                # 机器人没有生产好的物品
                                pass

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料6生产好但没有拿.
                        workstation6, over_produce6 = workstation7.find_latest_workstation(6)

                        if over_produce6:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation6)):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation6)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation6, 1)


                        # 情况2 原材料都没生产好，执行流程3，找材料2，3
                        # 判断原材料2
                        else:
                            # sys.stderr.write(str(over_produce6)+"\n")
                            # sys.stderr.write(str(workstation6.position) + "\n")
                            # sys.exit()
                            if judge_have(6):
                                material = workstation6.material_slot
                                if material is not None:
                                    if not material & 0b100:
                                        # 1.判断小车上是否有材料2
                                        eligible_robots3_1 = []
                                        eligible_robots3_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 2 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 2 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots3_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots3_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots3_1:
                                                distance = robot.distance_to(workstation6)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation6.material_slot & 0b100 == False:
                                                lock_define(min_robot_id, workstation6, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                        # 2. 判断材料2是否生产好但没有拿.
                                        workstation2, over_produce2 = workstation6.find_latest_workstation(2)

                                        if over_produce2:
                                            # 执行过程2，找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation2)):
                                                    eligible_robots3_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots3_2) > 0:
                                                for robot in eligible_robots3_2:
                                                    distance = robot.distance_to(workstation2)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation2, 1)

                                        # 判断原材料3
                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                    # 1.判断小车上是否有材料3
                                    if not material & 0b1000:
                                        eligible_robots4_1 = []
                                        eligible_robots4_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 3 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 3 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots4_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots4_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots4_1:
                                                distance = robot.distance_to(workstation6)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            if workstation6.material_slot & 0b1000 == False:
                                                lock_define(min_robot_id, workstation6, 0)
                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                        # 2. 判断工作台是否有生产好的材料3
                                        workstation3, over_produce3 = workstation6.find_latest_workstation(3)
                                        if over_produce3:
                                            # 找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation3)):
                                                    eligible_robots4_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots4_2) > 0:
                                                for robot in eligible_robots4_2:
                                                    distance = robot.distance_to(workstation3)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation3, 1)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8

                    if not material & 0b100000:

                        # 缺少材料5
                        # 情况0有生产好的物品5,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 5 and robot.state == True:
                                pass
                            elif robot.carry_type == 5 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots2_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass

                                # 过程1
                                # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b100000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料5生产好但没有拿.
                        workstation5, over_produce5 = workstation7.find_latest_workstation(5)
                        if over_produce5:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation5)):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation5)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation5, 1)

                        # 情况2 原材料都没生产好，执行流程3，找材料1，3
                        # 判断原材料1
                        else:
                                material = workstation5.material_slot
                                if not material & 0b10:
                                    # 1.判断小车上是否有材料1
                                    eligible_robots3_1 = []
                                    eligible_robots3_2 = []
                                    for robot in robots:
                                        if robot.carry_type == 1 and robot.state == True:
                                            pass
                                        elif robot.carry_type == 1 and robot.state == False:
                                            # 如果机器人满足条件，将其添加到列表中
                                            eligible_robots3_1.append(robot)
                                        else:
                                            # 机器人没有生产好的物品
                                            pass
                                    if len(eligible_robots3_1) > 0:
                                        min_distance = 100000
                                        min_robot_id = 0
                                        for robot in eligible_robots3_1:
                                            distance = robot.distance_to(workstation5)
                                            if distance < min_distance:
                                                min_distance = distance
                                                min_robot_id = robot.rid

                                        # 对最小距离的robot执行任务
                                        if workstation5.material_slot & 0b10 == False:
                                            lock_define(min_robot_id, workstation5, 0)

                                    #   判断是否需要继续运行
                                    if allow_run():
                                        return have_8
                                    # 2. 判断材料1是否生产好但没有拿.
                                    workstation1, over_produce1 = workstation5.find_latest_workstation(1)
                                    if over_produce1:

                                        # 执行过程2，找距离最近的机器人
                                        for robot in robots:
                                            if not robot.state and (
                                                    robot.carry_type == 0 or judge_send(robot, workstation1)):
                                                eligible_robots3_2.append(robot)
                                        min_distance = 10000
                                        if len(eligible_robots3_2) > 0:
                                            for robot in eligible_robots3_2:
                                                distance = robot.distance_to(workstation1)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            lock_define(min_robot_id, workstation1, 1)

                                    # 判断原材料3
                                    #   判断是否需要继续运行
                                    if allow_run():
                                        return have_8
                                # 1.判断小车上是否有材料3
                                if not material & 0b1000:
                                    eligible_robots4_1 = []
                                    eligible_robots4_2 = []
                                    for robot in robots:
                                        if robot.carry_type == 3 and robot.state == True:
                                            pass
                                        elif robot.carry_type == 3 and robot.state == False:
                                            # 如果机器人满足条件，将其添加到列表中
                                            eligible_robots4_1.append(robot)
                                        else:
                                            # 机器人没有生产好的物品
                                            pass
                                    if len(eligible_robots4_1) > 0:
                                        min_distance = 100000
                                        min_robot_id = 0
                                        for robot in eligible_robots4_1:
                                            distance = robot.distance_to(workstation5)
                                            if distance < min_distance:
                                                min_distance = distance
                                                min_robot_id = robot.rid

                                        # 对最小距离的robot执行任务
                                        if workstation5.material_slot & 0b1000 == False:
                                            lock_define(min_robot_id, workstation5, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                    # 2. 判断工作台是否有生产好的材料3
                                    workstation3, over_produce3 = workstation5.find_latest_workstation(3)
                                    if over_produce3:
                                        # 找距离最近的机器人
                                        for robot in robots:
                                            if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation3)):
                                                eligible_robots4_2.append(robot)
                                        min_distance = 10000
                                        if len(eligible_robots4_2) > 0:
                                            for robot in eligible_robots4_2:
                                                distance = robot.distance_to(workstation3)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            lock_define(min_robot_id, workstation3, 1)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8

                    if not material & 0b10000:
                        # 缺少材料4
                        # 情况0有生产好的物品4,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 4 and robot.state == True:
                                pass
                            elif robot.carry_type == 4 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中

                                eligible_robots2_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass

                            # 过程1
                            # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b10000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料4生产好但没有拿.
                        workstation4, over_produce4 = workstation7.find_latest_workstation(4)
                        if over_produce4:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if (not robot.state) and (robot.carry_type == 0):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation4)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                if (robot.carry_type == 0):
                                    lock_define(min_robot_id, workstation4, 1)
                                else:
                                    lock_define(min_robot_id, workstation4, 0)

                        # 情况2 原材料都没生产好，执行流程3，找材料2，1
                        # 判断原材料2
                        else:
                            if judge_have(4):
                                material = workstation4.material_slot
                                if material is not None:
                                    if not material & 0b10:
                                        # 1.判断小车上是否有材料1
                                        eligible_robots4_1 = []
                                        eligible_robots4_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 1 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 1 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots4_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots4_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots4_1:
                                                distance = robot.distance_to(workstation4)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation4.material_slot & 0b10 == False:
                                                lock_define(min_robot_id, workstation4, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                        # 2. 判断工作台是否有生产好的材料1
                                        workstation1, over_produce1 = workstation4.find_latest_workstation(1)
                                        if over_produce1:
                                            # 找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation1)):
                                                    eligible_robots4_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots4_2) > 0:
                                                for robot in eligible_robots4_2:
                                                    distance = robot.distance_to(workstation1)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation1, 1)
                                    if allow_run():
                                        return have_8

                                    if not material & 0b100:
                                        # 1.判断小车上是否有材料2
                                        eligible_robots3_1 = []
                                        eligible_robots3_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 2 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 2 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots3_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots3_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots3_1:
                                                distance = robot.distance_to(workstation4)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation4.material_slot & 0b100 == False:
                                                lock_define(min_robot_id, workstation4, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                        # 2. 判断材料2是否生产好但没有拿.
                                        workstation2, over_produce2 = workstation4.find_latest_workstation(2)
                                        if over_produce2:

                                            # 执行过程2，找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation2)):
                                                    eligible_robots3_2.append(robot)
                                            min_distance = 10000
                                            if len(eligible_robots3_2) > 0:
                                                for robot in eligible_robots3_2:
                                                    distance = robot.distance_to(workstation2)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation2, 1)
                                        # 判断原材料1
                                        #   判断是否需要继续运行


                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8

        if workstation.wid == 9 and have_8:
            # 1.检查机器人身上是否有生产好的7号物品
            for robot in robots:
                if robot.carry_type == 7 and robot.state == True:
                    pass
                elif robot.carry_type == 7 and robot.state == False:
                    # 如果机器人满足条件，将其添加到列表中
                    eligible_robots.append(robot)
                else:
                    # 机器人没有生产好的物品
                    pass

            # 过程1
            # 寻找最小距离的robot的id
            if len(eligible_robots) > 0:
                min_distance = 100000
                min_robot_id = 0
                for robot in eligible_robots:
                    distance = robot.distance_to(workstation)
                    if distance < min_distance:
                        min_distance = distance
                        min_robot_id = robot.rid
                # 对最小距离的robot执行任务

                lock_define(min_robot_id, workstation, 0)

            #   判断是否需要继续运行
            if allow_run():
                return have_8

            # 2.检查7号工作台是否生产好了七号物品而没有取
            workstation7, over_produce7 = workstation.find_latest_workstation(7)
            if over_produce7:
                # 有生产好的物品
                # 执行过程2
                for robot in robots:
                    if not robot.state and (
                            robot.carry_type == 0 or judge_material(workstation7, robot.carry_type)):
                        eligible_robots2.append(robot)

                min_distance = 10000
                min_robot_id = 0
                if len(eligible_robots2) > 0:
                    for robot in eligible_robots2:
                        distance = robot.distance_to(workstation7)
                        if distance < min_distance:
                            min_distance = distance
                            min_robot_id = robot.rid
                    # 对最小距离的robot执行任务

                    lock_define(min_robot_id, workstation7, 1)

            # 3. 没有生产好的物品，找4，5，6原材料

            else:

                material = workstation7.material_slot
                if material is not None:
                    if not material & 0b1000000:
                        # 缺少材料6
                        # 情况0有生产好的物品6,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 6 and robot.state == True:
                                pass
                            elif robot.carry_type == 6 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots2_1.append(robot)
                        # 过程1
                        # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b1000000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                            else:
                                # 机器人没有生产好的物品
                                pass

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料6生产好但没有拿.
                        workstation6, over_produce6 = workstation7.find_latest_workstation(6)

                        if over_produce6:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation6)):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation6)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation6, 1)


                        # 情况2 原材料都没生产好，执行流程3，找材料2，3
                        # 判断原材料2
                        else:
                            # sys.stderr.write(str(over_produce6)+"\n")
                            # sys.stderr.write(str(workstation6.position) + "\n")
                            # sys.exit()
                            if judge_have(6):
                                material = workstation6.material_slot
                                if material is not None:
                                    if not material & 0b100:
                                        # 1.判断小车上是否有材料2
                                        eligible_robots3_1 = []
                                        eligible_robots3_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 2 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 2 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots3_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots3_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots3_1:
                                                distance = robot.distance_to(workstation6)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation6.material_slot & 0b100 == False:
                                                lock_define(min_robot_id, workstation6, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                        # 2. 判断材料2是否生产好但没有拿.
                                        workstation2, over_produce2 = workstation6.find_latest_workstation(2)

                                        if over_produce2:
                                            # 执行过程2，找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation2)):
                                                    eligible_robots3_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots3_2) > 0:
                                                for robot in eligible_robots3_2:
                                                    distance = robot.distance_to(workstation2)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation2, 1)

                                        # 判断原材料3
                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                    # 1.判断小车上是否有材料3
                                    if not material & 0b1000:
                                        eligible_robots4_1 = []
                                        eligible_robots4_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 3 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 3 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots4_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots4_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots4_1:
                                                distance = robot.distance_to(workstation6)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            if workstation6.material_slot & 0b1000 == False:
                                                lock_define(min_robot_id, workstation6, 0)
                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                        # 2. 判断工作台是否有生产好的材料3
                                        workstation3, over_produce3 = workstation6.find_latest_workstation(3)
                                        if over_produce3:
                                            # 找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation3)):
                                                    eligible_robots4_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots4_2) > 0:
                                                for robot in eligible_robots4_2:
                                                    distance = robot.distance_to(workstation3)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation3, 1)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8

                    if not material & 0b100000:

                        # 缺少材料5
                        # 情况0有生产好的物品5,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 5 and robot.state == True:
                                pass
                            elif robot.carry_type == 5 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots2_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass

                                # 过程1
                                # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b100000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料5生产好但没有拿.
                        workstation5, over_produce5 = workstation7.find_latest_workstation(5)
                        if over_produce5:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation5)):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation5)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation5, 1)

                        # 情况2 原材料都没生产好，执行流程3，找材料1，3
                        # 判断原材料1
                        else:
                                material = workstation5.material_slot
                                if not material & 0b10:
                                    # 1.判断小车上是否有材料1
                                    eligible_robots3_1 = []
                                    eligible_robots3_2 = []
                                    for robot in robots:
                                        if robot.carry_type == 1 and robot.state == True:
                                            pass
                                        elif robot.carry_type == 1 and robot.state == False:
                                            # 如果机器人满足条件，将其添加到列表中
                                            eligible_robots3_1.append(robot)
                                        else:
                                            # 机器人没有生产好的物品
                                            pass
                                    if len(eligible_robots3_1) > 0:
                                        min_distance = 100000
                                        min_robot_id = 0
                                        for robot in eligible_robots3_1:
                                            distance = robot.distance_to(workstation5)
                                            if distance < min_distance:
                                                min_distance = distance
                                                min_robot_id = robot.rid

                                        # 对最小距离的robot执行任务
                                        if workstation5.material_slot & 0b10 == False:
                                            lock_define(min_robot_id, workstation5, 0)

                                    #   判断是否需要继续运行
                                    if allow_run():
                                        return have_8
                                    # 2. 判断材料1是否生产好但没有拿.
                                    workstation1, over_produce1 = workstation5.find_latest_workstation(1)
                                    if over_produce1:

                                        # 执行过程2，找距离最近的机器人
                                        for robot in robots:
                                            if not robot.state and (
                                                    robot.carry_type == 0 or judge_send(robot, workstation1)):
                                                eligible_robots3_2.append(robot)
                                        min_distance = 10000
                                        if len(eligible_robots3_2) > 0:
                                            for robot in eligible_robots3_2:
                                                distance = robot.distance_to(workstation1)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            lock_define(min_robot_id, workstation1, 1)

                                    # 判断原材料3
                                    #   判断是否需要继续运行
                                    if allow_run():
                                        return have_8
                                # 1.判断小车上是否有材料3
                                if not material & 0b1000:
                                    eligible_robots4_1 = []
                                    eligible_robots4_2 = []
                                    for robot in robots:
                                        if robot.carry_type == 3 and robot.state == True:
                                            pass
                                        elif robot.carry_type == 3 and robot.state == False:
                                            # 如果机器人满足条件，将其添加到列表中
                                            eligible_robots4_1.append(robot)
                                        else:
                                            # 机器人没有生产好的物品
                                            pass
                                    if len(eligible_robots4_1) > 0:
                                        min_distance = 100000
                                        min_robot_id = 0
                                        for robot in eligible_robots4_1:
                                            distance = robot.distance_to(workstation5)
                                            if distance < min_distance:
                                                min_distance = distance
                                                min_robot_id = robot.rid

                                        # 对最小距离的robot执行任务
                                        if workstation5.material_slot & 0b1000 == False:
                                            lock_define(min_robot_id, workstation5, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                    # 2. 判断工作台是否有生产好的材料3
                                    workstation3, over_produce3 = workstation5.find_latest_workstation(3)
                                    if over_produce3:
                                        # 找距离最近的机器人
                                        for robot in robots:
                                            if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation3)):
                                                eligible_robots4_2.append(robot)
                                        min_distance = 10000
                                        if len(eligible_robots4_2) > 0:
                                            for robot in eligible_robots4_2:
                                                distance = robot.distance_to(workstation3)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid
                                            # 对最小距离的robot执行任务
                                            lock_define(min_robot_id, workstation3, 1)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8

                    if not material & 0b10000:
                        # 缺少材料4
                        # 情况0有生产好的物品4,在机器人身上
                        eligible_robots2_1 = []
                        eligible_robots2_2 = []
                        for robot in robots:
                            if robot.carry_type == 4 and robot.state == True:
                                pass
                            elif robot.carry_type == 4 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中

                                eligible_robots2_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass

                            # 过程1
                            # 寻找最小距离的robot的id
                        if len(eligible_robots2_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots2_1:
                                distance = robot.distance_to(workstation7)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation7.material_slot & 0b10000 == False:
                                lock_define(min_robot_id, workstation7, 0)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 情况1 材料4生产好但没有拿.
                        workstation4, over_produce4 = workstation7.find_latest_workstation(4)
                        if over_produce4:
                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if (not robot.state) and (robot.carry_type == 0):
                                    eligible_robots2_2.append(robot)
                            min_distance = 10000
                            min_robot_id = 0
                            if len(eligible_robots2_2) > 0:
                                for robot in eligible_robots2_2:
                                    distance = robot.distance_to(workstation4)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                if (robot.carry_type == 0):
                                    lock_define(min_robot_id, workstation4, 1)
                                else:
                                    lock_define(min_robot_id, workstation4, 0)

                        # 情况2 原材料都没生产好，执行流程3，找材料2，1
                        # 判断原材料2
                        else:
                            if judge_have(4):
                                material = workstation4.material_slot
                                if material is not None:
                                    if not material & 0b10:
                                        # 1.判断小车上是否有材料1
                                        eligible_robots4_1 = []
                                        eligible_robots4_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 1 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 1 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots4_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots4_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots4_1:
                                                distance = robot.distance_to(workstation4)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation4.material_slot & 0b10 == False:
                                                lock_define(min_robot_id, workstation4, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8

                                        # 2. 判断工作台是否有生产好的材料1
                                        workstation1, over_produce1 = workstation4.find_latest_workstation(1)
                                        if over_produce1:
                                            # 找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation1)):
                                                    eligible_robots4_2.append(robot)
                                            min_distance = 10000
                                            min_robot_id = 0
                                            if len(eligible_robots4_2) > 0:
                                                for robot in eligible_robots4_2:
                                                    distance = robot.distance_to(workstation1)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation1, 1)
                                    if allow_run():
                                        return have_8

                                    if not material & 0b100:
                                        # 1.判断小车上是否有材料2
                                        eligible_robots3_1 = []
                                        eligible_robots3_2 = []
                                        for robot in robots:
                                            if robot.carry_type == 2 and robot.state == True:
                                                pass
                                            elif robot.carry_type == 2 and robot.state == False:
                                                # 如果机器人满足条件，将其添加到列表中
                                                eligible_robots3_1.append(robot)
                                            else:
                                                # 机器人没有生产好的物品
                                                pass
                                        if len(eligible_robots3_1) > 0:
                                            min_distance = 100000
                                            min_robot_id = 0
                                            for robot in eligible_robots3_1:
                                                distance = robot.distance_to(workstation4)
                                                if distance < min_distance:
                                                    min_distance = distance
                                                    min_robot_id = robot.rid

                                            # 对最小距离的robot执行任务
                                            if workstation4.material_slot & 0b100 == False:
                                                lock_define(min_robot_id, workstation4, 0)

                                        #   判断是否需要继续运行
                                        if allow_run():
                                            return have_8
                                        # 2. 判断材料2是否生产好但没有拿.
                                        workstation2, over_produce2 = workstation4.find_latest_workstation(2)
                                        if over_produce2:

                                            # 执行过程2，找距离最近的机器人
                                            for robot in robots:
                                                if not robot.state and (
                                                        robot.carry_type == 0 or judge_send(robot, workstation2)):
                                                    eligible_robots3_2.append(robot)
                                            min_distance = 10000
                                            if len(eligible_robots3_2) > 0:
                                                for robot in eligible_robots3_2:
                                                    distance = robot.distance_to(workstation2)
                                                    if distance < min_distance:
                                                        min_distance = distance
                                                        min_robot_id = robot.rid
                                                # 对最小距离的robot执行任务
                                                lock_define(min_robot_id, workstation2, 1)
                                        # 判断原材料1
                                        #   判断是否需要继续运行


                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
        if workstation.wid == 9 and (have_8==False):
            # 缺少材料6
            # 情况0有生产好的物品6,在机器人身上
            eligible_robots2_1 = []
            eligible_robots2_2 = []
            for robot in robots:
                if robot.carry_type == 6 and robot.state == True:
                    pass
                elif robot.carry_type == 6 and robot.state == False:
                    robot_allocation_lists[robot.rid].dst_place=workstation.position
                    # lock_define(robot.rid, workstation, 0)
                    # 如果机器人满足条件，将其添加到列表中
                    # eligible_robots2_1.append(robot)
            # # 过程1
            # # 寻找最小距离的robot的id
            # if len(eligible_robots2_1) > 0:
            #     min_distance = 100000
            #     min_robot_id = 0
            #     for robot in eligible_robots2_1:
            #         distance = robot.distance_to(workstation)
            #         if distance < min_distance:
            #             min_distance = distance
            #             min_robot_id = robot.rid
                # 对最小距离的robot执行任务


            #   判断是否需要继续运行
            if allow_run():
                return have_8
            # 情况1 材料6生产好但没有拿.
            workstation6, over_produce6 = workstation.find_latest_workstation(6)

            if over_produce6:
                # 执行过程2，找距离最近的机器人
                for robot in robots:
                    if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation6)):
                        eligible_robots2_2.append(robot)
                min_distance = 10000
                min_robot_id = 0
                if len(eligible_robots2_2) > 0:
                    for robot in eligible_robots2_2:
                        distance = robot.distance_to(workstation6)
                        if distance < min_distance:
                            min_distance = distance
                            min_robot_id = robot.rid
                    # 对最小距离的robot执行任务
                    lock_define(min_robot_id, workstation6, 1)


            # 情况2 原材料都没生产好，执行流程3，找材料2，3
            # 判断原材料2
            else:
                # sys.stderr.write(str(over_produce6)+"\n")
                # sys.stderr.write(str(workstation6.position) + "\n")
                # sys.exit()
                if judge_have(6):
                    material = workstation6.material_slot
                    if material is not None:
                        if not material & 0b100:
                            # 1.判断小车上是否有材料2
                            eligible_robots3_1 = []
                            eligible_robots3_2 = []
                            for robot in robots:
                                if robot.carry_type == 2 and robot.state == True:
                                    pass
                                elif robot.carry_type == 2 and robot.state == False:
                                    # 如果机器人满足条件，将其添加到列表中
                                    eligible_robots3_1.append(robot)
                                else:
                                    # 机器人没有生产好的物品
                                    pass
                            if len(eligible_robots3_1) > 0:
                                min_distance = 100000
                                min_robot_id = 0
                                for robot in eligible_robots3_1:
                                    distance = robot.distance_to(workstation6)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid

                                # 对最小距离的robot执行任务
                                if workstation6.material_slot & 0b100 == False:
                                    lock_define(min_robot_id, workstation6, 0)

                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8
                            # 2. 判断材料2是否生产好但没有拿.
                            workstation2, over_produce2 = workstation6.find_latest_workstation(2)

                            if over_produce2:
                                # 执行过程2，找距离最近的机器人
                                for robot in robots:
                                    if not robot.state and (
                                            robot.carry_type == 0 or judge_send(robot, workstation2)):
                                        eligible_robots3_2.append(robot)
                                min_distance = 10000
                                min_robot_id = 0
                                if len(eligible_robots3_2) > 0:
                                    for robot in eligible_robots3_2:
                                        distance = robot.distance_to(workstation2)
                                        if distance < min_distance:
                                            min_distance = distance
                                            min_robot_id = robot.rid
                                    # 对最小距离的robot执行任务
                                    lock_define(min_robot_id, workstation2, 1)

                            # 判断原材料3
                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8
                        # 1.判断小车上是否有材料3
                        if not material & 0b1000:
                            eligible_robots4_1 = []
                            eligible_robots4_2 = []
                            for robot in robots:
                                if robot.carry_type == 3 and robot.state == True:
                                    pass
                                elif robot.carry_type == 3 and robot.state == False:
                                    # 如果机器人满足条件，将其添加到列表中
                                    eligible_robots4_1.append(robot)
                                else:
                                    # 机器人没有生产好的物品
                                    pass
                            if len(eligible_robots4_1) > 0:
                                min_distance = 100000
                                min_robot_id = 0
                                for robot in eligible_robots4_1:
                                    distance = robot.distance_to(workstation6)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                if workstation6.material_slot & 0b1000 == False:
                                    lock_define(min_robot_id, workstation6, 0)
                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8

                            # 2. 判断工作台是否有生产好的材料3
                            workstation3, over_produce3 = workstation6.find_latest_workstation(3)
                            if over_produce3:
                                # 找距离最近的机器人
                                for robot in robots:
                                    if not robot.state and (
                                            robot.carry_type == 0 or judge_send(robot, workstation3)):
                                        eligible_robots4_2.append(robot)
                                min_distance = 10000
                                min_robot_id = 0
                                if len(eligible_robots4_2) > 0:
                                    for robot in eligible_robots4_2:
                                        distance = robot.distance_to(workstation3)
                                        if distance < min_distance:
                                            min_distance = distance
                                            min_robot_id = robot.rid
                                    # 对最小距离的robot执行任务
                                    lock_define(min_robot_id, workstation3, 1)

            #   判断是否需要继续运行
            if allow_run():
                return have_8

            # 缺少材料5
            # 情况0有生产好的物品5,在机器人身上
            eligible_robots2_1 = []
            eligible_robots2_2 = []
            for robot in robots:
                if robot.carry_type == 5 and robot.state == True:
                    pass
                elif robot.carry_type == 5 and robot.state == False:
                    # 如果机器人满足条件，将其添加到列表中
                    robot_allocation_lists[robot.rid].dst_place = workstation.position
                else:
                    # 机器人没有生产好的物品
                    pass

                    # 过程1
                    # 寻找最小距离的robot的id
            # if len(eligible_robots2_1) > 0:
            #     min_distance = 100000
            #     min_robot_id = 0
            #     for robot in eligible_robots2_1:
            #         distance = robot.distance_to(workstation)
            #         if distance < min_distance:
            #             min_distance = distance
            #             min_robot_id = robot.rid
            #
            #     # 对最小距离的robot执行任务
            #
            #     lock_define(min_robot_id, workstation, 0)

            #   判断是否需要继续运行
            if allow_run():
                return have_8
            # 情况1 材料5生产好但没有拿.
            workstation5, over_produce5 = workstation.find_latest_workstation(5)
            if over_produce5:
                # 执行过程2，找距离最近的机器人
                for robot in robots:
                    if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation5)):
                        eligible_robots2_2.append(robot)
                min_distance = 10000
                min_robot_id = 0
                if len(eligible_robots2_2) > 0:
                    for robot in eligible_robots2_2:
                        distance = robot.distance_to(workstation5)
                        if distance < min_distance:
                            min_distance = distance
                            min_robot_id = robot.rid
                    # 对最小距离的robot执行任务
                    lock_define(min_robot_id, workstation5, 1)

            # 情况2 原材料都没生产好，执行流程3，找材料1，3
            # 判断原材料1
            else:
                    material = workstation5.material_slot
                    if not material & 0b10:
                        # 1.判断小车上是否有材料1
                        eligible_robots3_1 = []
                        eligible_robots3_2 = []
                        for robot in robots:
                            if robot.carry_type == 1 and robot.state == True:
                                pass
                            elif robot.carry_type == 1 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots3_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass
                        if len(eligible_robots3_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots3_1:
                                distance = robot.distance_to(workstation5)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation5.material_slot & 0b10 == False:
                                lock_define(min_robot_id, workstation5, 0)

                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                        # 2. 判断材料1是否生产好但没有拿.
                        workstation1, over_produce1 = workstation5.find_latest_workstation(1)
                        if over_produce1:

                            # 执行过程2，找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (
                                        robot.carry_type == 0 or judge_send(robot, workstation1)):
                                    eligible_robots3_2.append(robot)
                            min_distance = 10000
                            if len(eligible_robots3_2) > 0:
                                for robot in eligible_robots3_2:
                                    distance = robot.distance_to(workstation1)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation1, 1)

                        # 判断原材料3
                        #   判断是否需要继续运行
                        if allow_run():
                            return have_8
                    # 1.判断小车上是否有材料3
                    if not material & 0b1000:
                        eligible_robots4_1 = []
                        eligible_robots4_2 = []
                        for robot in robots:
                            if robot.carry_type == 3 and robot.state == True:
                                pass
                            elif robot.carry_type == 3 and robot.state == False:
                                # 如果机器人满足条件，将其添加到列表中
                                eligible_robots4_1.append(robot)
                            else:
                                # 机器人没有生产好的物品
                                pass
                        if len(eligible_robots4_1) > 0:
                            min_distance = 100000
                            min_robot_id = 0
                            for robot in eligible_robots4_1:
                                distance = robot.distance_to(workstation5)
                                if distance < min_distance:
                                    min_distance = distance
                                    min_robot_id = robot.rid

                            # 对最小距离的robot执行任务
                            if workstation5.material_slot & 0b1000 == False:
                                lock_define(min_robot_id, workstation5, 0)

                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8

                        # 2. 判断工作台是否有生产好的材料3
                        workstation3, over_produce3 = workstation5.find_latest_workstation(3)
                        if over_produce3:
                            # 找距离最近的机器人
                            for robot in robots:
                                if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation3)):
                                    eligible_robots4_2.append(robot)
                            min_distance = 10000
                            if len(eligible_robots4_2) > 0:
                                for robot in eligible_robots4_2:
                                    distance = robot.distance_to(workstation3)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid
                                # 对最小距离的robot执行任务
                                lock_define(min_robot_id, workstation3, 1)

            #   判断是否需要继续运行
            if allow_run():
                return have_8


            # 缺少材料4
            # 情况0有生产好的物品4,在机器人身上
            eligible_robots2_1 = []
            eligible_robots2_2 = []
            for robot in robots:
                if robot.carry_type == 4 and robot.state == True:
                    pass
                elif robot.carry_type == 4 and robot.state == False:
                    # 如果机器人满足条件，将其添加到列表中
                    robot_allocation_lists[robot.rid].dst_place = workstation.position
                    # eligible_robots2_1.append(robot)
                else:
                    # 机器人没有生产好的物品
                    pass

            #     # 过程1
            #     # 寻找最小距离的robot的id
            # if len(eligible_robots2_1) > 0:
            #     min_distance = 100000
            #     min_robot_id = 0
            #     for robot in eligible_robots2_1:
            #         distance = robot.distance_to(workstation)
            #         if distance < min_distance:
            #             min_distance = distance
            #             min_robot_id = robot.rid
            #
            #     # 对最小距离的robot执行任务
            #     lock_define(min_robot_id, workstation, 0)

            #   判断是否需要继续运行
            if allow_run():
                return have_8
            # 情况1 材料4生产好但没有拿.
            workstation4, over_produce4 = workstation.find_latest_workstation(4)
            if over_produce4:
                # 执行过程2，找距离最近的机器人
                for robot in robots:
                    if (not robot.state) and (robot.carry_type == 0):
                        eligible_robots2_2.append(robot)
                min_distance = 10000
                min_robot_id = 0
                if len(eligible_robots2_2) > 0:
                    for robot in eligible_robots2_2:
                        distance = robot.distance_to(workstation4)
                        if distance < min_distance:
                            min_distance = distance
                            min_robot_id = robot.rid
                    # 对最小距离的robot执行任务
                    if (robot.carry_type == 0):
                        lock_define(min_robot_id, workstation4, 1)
                    else:
                        lock_define(min_robot_id, workstation4, 0)

            # 情况2 原材料都没生产好，执行流程3，找材料2，1
            # 判断原材料2
            else:
                if judge_have(4):
                    material = workstation4.material_slot
                    if material is not None:
                        if not material & 0b10:
                            # 1.判断小车上是否有材料1
                            eligible_robots4_1 = []
                            eligible_robots4_2 = []
                            for robot in robots:
                                if robot.carry_type == 1 and robot.state == True:
                                    pass
                                elif robot.carry_type == 1 and robot.state == False:
                                    # 如果机器人满足条件，将其添加到列表中
                                    eligible_robots4_1.append(robot)
                                else:
                                    # 机器人没有生产好的物品
                                    pass
                            if len(eligible_robots4_1) > 0:
                                min_distance = 100000
                                min_robot_id = 0
                                for robot in eligible_robots4_1:
                                    distance = robot.distance_to(workstation4)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid

                                # 对最小距离的robot执行任务
                                if workstation4.material_slot & 0b10 == False:
                                    lock_define(min_robot_id, workstation4, 0)

                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8

                            # 2. 判断工作台是否有生产好的材料1
                            workstation1, over_produce1 = workstation4.find_latest_workstation(1)
                            if over_produce1:
                                # 找距离最近的机器人
                                for robot in robots:
                                    if not robot.state and (
                                            robot.carry_type == 0 or judge_send(robot, workstation1)):
                                        eligible_robots4_2.append(robot)
                                min_distance = 10000
                                min_robot_id = 0
                                if len(eligible_robots4_2) > 0:
                                    for robot in eligible_robots4_2:
                                        distance = robot.distance_to(workstation1)
                                        if distance < min_distance:
                                            min_distance = distance
                                            min_robot_id = robot.rid
                                    # 对最小距离的robot执行任务
                                    lock_define(min_robot_id, workstation1, 1)
                        if allow_run():
                            return have_8

                        if not material & 0b100:
                            # 1.判断小车上是否有材料2
                            eligible_robots3_1 = []
                            eligible_robots3_2 = []
                            for robot in robots:
                                if robot.carry_type == 2 and robot.state == True:
                                    pass
                                elif robot.carry_type == 2 and robot.state == False:
                                    # 如果机器人满足条件，将其添加到列表中
                                    eligible_robots3_1.append(robot)
                                else:
                                    # 机器人没有生产好的物品
                                    pass
                            if len(eligible_robots3_1) > 0:
                                min_distance = 100000
                                min_robot_id = 0
                                for robot in eligible_robots3_1:
                                    distance = robot.distance_to(workstation4)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_robot_id = robot.rid

                                # 对最小距离的robot执行任务
                                if workstation4.material_slot & 0b100 == False:
                                    lock_define(min_robot_id, workstation4, 0)

                            #   判断是否需要继续运行
                            if allow_run():
                                return have_8
                            # 2. 判断材料2是否生产好但没有拿.
                            workstation2, over_produce2 = workstation4.find_latest_workstation(2)
                            if over_produce2:

                                # 执行过程2，找距离最近的机器人
                                for robot in robots:
                                    if not robot.state and (
                                            robot.carry_type == 0 or judge_send(robot, workstation2)):
                                        eligible_robots3_2.append(robot)
                                min_distance = 10000
                                if len(eligible_robots3_2) > 0:
                                    for robot in eligible_robots3_2:
                                        distance = robot.distance_to(workstation2)
                                        if distance < min_distance:
                                            min_distance = distance
                                            min_robot_id = robot.rid
                                    # 对最小距离的robot执行任务
                                    lock_define(min_robot_id, workstation2, 1)
                            # 判断原材料1
                            #   判断是否需要继续运行


            #   判断是否需要继续运行
            if allow_run():
                return have_8
        # if workstation.wid == 9:
        #     # 缺少材料6
        #     # 情况0有生产好的物品6,在机器人身上
        #     eligible_robots2_1 = []
        #     eligible_robots2_2 = []
        #     for robot in robots:
        #         if robot.carry_type == 6 and robot.state == True:
        #             pass
        #         elif robot.carry_type == 6 and robot.state == False:
        #             # 如果机器人满足条件，将其添加到列表中
        #             eligible_robots2_1.append(robot)
        #             # 过程1
        #             # 寻找最小距离的robot的id
        #     if len(eligible_robots2_1) > 0:
        #         min_distance = 100000
        #         min_robot_id = 0
        #         for robot in eligible_robots2_1:
        #             distance = robot.distance_to(workstation)
        #             if distance < min_distance:
        #                 min_distance = distance
        #                 min_robot_id = robot.rid
        #
        #         # 对最小距离的robot执行任务
        #
        #         lock_define(min_robot_id, workstation, 0)
        #     else:
        #         # 机器人没有生产好的物品
        #         pass
        #
        #     # 情况1 材料6生产好但没有拿.
        #     workstation6, over_produce6 = workstation.find_latest_workstation(6)
        #     if over_produce6:
        #         # 执行过程2，找距离最近的机器人
        #         for robot in robots:
        #             if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation6)):
        #                 eligible_robots2_2.append(robot)
        #         min_distance = 10000
        #         if len(eligible_robots2_2) > 0:
        #             for robot in eligible_robots2_2:
        #                 distance = robot.distance_to(workstation6)
        #                 if distance < min_distance:
        #                     min_distance = distance
        #                     min_robot_id = robot.rid
        #             # 对最小距离的robot执行任务
        #             lock_define(min_robot_id, workstation6, 1)
        #
        #     # 情况2 原材料都没生产好，执行流程3，找材料2，3
        #     # 判断原材料2
        #     else:
        #
        #         material = workstation6.material_slot
        #         if not material & 0b10:
        #             # 1.判断小车上是否有材料2
        #             eligible_robots3_1 = []
        #             eligible_robots3_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 2 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 2 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots3_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots3_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots3_1:
        #                     distance = robot.distance_to(workstation6)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #
        #                 # 对最小距离的robot执行任务
        #                 if workstation6.material_slot & 0b100 == False:
        #                     lock_define(min_robot_id, workstation6, 0)
        #
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #             # 2. 判断材料2是否生产好但没有拿.
        #
        #             workstation2, over_produce2 = workstation6.find_latest_workstation(2)
        #
        #             # if over_produce2:
        #             # 执行过程2，找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation2)):
        #                     eligible_robots3_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots3_2) > 0:
        #                 for robot in eligible_robots3_2:
        #                     distance = robot.distance_to(workstation2)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation2, 1)
        #
        #         material = workstation6.material_slot
        #         if not material & 0b1000:
        #             # 判断原材料3
        #             #   判断是否需要继续运行
        #
        #             # 1.判断小车上是否有材料3
        #             eligible_robots4_1 = []
        #             eligible_robots4_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 3 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 3 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots4_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots4_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots4_1:
        #                     distance = robot.distance_to(workstation6)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 if workstation6.material_slot & 0b1000 == False:
        #                     lock_define(min_robot_id, workstation6, 0)
        #
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #
        #             # 2. 判断工作台是否有生产好的材料3
        #             workstation3, over_produce3 = workstation6.find_latest_workstation(3)
        #             # if over_produce3:
        #             # 找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation3)):
        #                     eligible_robots4_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots4_2) > 0:
        #                 for robot in eligible_robots4_2:
        #                     distance = robot.distance_to(workstation3)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation3, 1)
        #
        #
        #     # 缺少材料5
        #     # 情况0有生产好的物品5,在机器人身上
        #     eligible_robots2_1 = []
        #     eligible_robots2_2 = []
        #     for robot in robots:
        #         if robot.carry_type == 5 and robot.state == True:
        #             pass
        #         elif robot.carry_type == 5 and robot.state == False:
        #             # 如果机器人满足条件，将其添加到列表中
        #             eligible_robots2_1.append(robot)
        #         else:
        #             # 机器人没有生产好的物品
        #             pass
        #
        #             # 过程1
        #             # 寻找最小距离的robot的id
        #     if len(eligible_robots2_1) > 0:
        #         min_distance = 100000
        #         min_robot_id = 0
        #         for robot in eligible_robots2_1:
        #             distance = robot.distance_to(workstation)
        #             if distance < min_distance:
        #                 min_distance = distance
        #                 min_robot_id = robot.rid
        #
        #             # 对最小距离的robot执行任务
        #
        #         lock_define(min_robot_id, workstation, 0)
        #
        #
        #     # 情况1 材料5生产好但没有拿.
        #     workstation5, over_produce5 = workstation.find_latest_workstation(5)
        #     if over_produce5:
        #         # 执行过程2，找距离最近的机器人
        #         for robot in robots:
        #             if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation5)):
        #                 eligible_robots2_2.append(robot)
        #         min_distance = 10000
        #         if len(eligible_robots2_2) > 0:
        #             for robot in eligible_robots2_2:
        #                 distance = robot.distance_to(workstation5)
        #                 if distance < min_distance:
        #                     min_distance = distance
        #                     min_robot_id = robot.rid
        #             # 对最小距离的robot执行任务
        #             lock_define(min_robot_id, workstation5, 1)
        #     # 情况2 原材料都没生产好，执行流程3，找材料1，3
        #     # 判断原材料1
        #     else:
        #         material = workstation5.material_slot
        #         if not material & 0b10:
        #             # 1.判断小车上是否有材料1
        #             eligible_robots3_1 = []
        #             eligible_robots3_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 1 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 1 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots3_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots3_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots3_1:
        #                     distance = robot.distance_to(workstation5)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #
        #                 # 对最小距离的robot执行任务
        #                 if workstation5.material_slot & 0b10 == False:
        #                     lock_define(min_robot_id, workstation5, 0)
        #
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #             # 2. 判断材料1是否生产好但没有拿.
        #             workstation1, over_produce1 = workstation5.find_latest_workstation(1)
        #             # if over_produce1:
        #
        #             # 执行过程2，找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation6)):
        #                     eligible_robots3_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots3_2) > 0:
        #                 for robot in eligible_robots3_2:
        #                     distance = robot.distance_to(workstation1)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation1, 1)
        #
        #         material = workstation5.material_slot
        #         if not material & 0b1000:
        #             # 判断原材料3
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #             # 1.判断小车上是否有材料3
        #             eligible_robots4_1 = []
        #             eligible_robots4_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 3 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 3 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots4_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots4_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots4_1:
        #                     distance = robot.distance_to(workstation5)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #
        #                 # 对最小距离的robot执行任务
        #                 if workstation5.material_slot & 0b1000 == False:
        #                     lock_define(min_robot_id, workstation5, 0)
        #
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #
        #             # 2. 判断工作台是否有生产好的材料3
        #             workstation3, over_produce3 = workstation5.find_latest_workstation(3)
        #             # if over_produce3:
        #             # 找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation3)):
        #                     eligible_robots4_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots4_2) > 0:
        #                 for robot in eligible_robots4_2:
        #                     distance = robot.distance_to(workstation3)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation3, 1)
        #
        #
        #
        #     # 缺少材料4
        #     # 情况0有生产好的物品4,在机器人身上
        #     eligible_robots2_1 = []
        #     eligible_robots2_2 = []
        #     for robot in robots:
        #         if robot.carry_type == 4 and robot.state == True:
        #             pass
        #         elif robot.carry_type == 4 and robot.state == False:
        #             # 如果机器人满足条件，将其添加到列表中
        #             eligible_robots2_1.append(robot)
        #         else:
        #             # 机器人没有生产好的物品
        #             pass
        #
        #             # 过程1
        #             # 寻找最小距离的robot的id
        #     if len(eligible_robots2_1) > 0:
        #         min_distance = 100000
        #         min_robot_id = 0
        #         for robot in eligible_robots2_1:
        #             distance = robot.distance_to(workstation)
        #             if distance < min_distance:
        #                 min_distance = distance
        #                 min_robot_id = robot.rid
        #
        #             # 对最小距离的robot执行任务
        #
        #         lock_define(min_robot_id, workstation, 0)
        #
        #
        #     # 情况1 材料4生产好但没有拿.
        #     workstation4, over_produce4 = workstation.find_latest_workstation(4)
        #     if over_produce4:
        #         # 执行过程2，找距离最近的机器人
        #         for robot in robots:
        #             if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation4)):
        #                 eligible_robots2_2.append(robot)
        #         min_distance = 10000
        #         if len(eligible_robots2_2) > 0:
        #             for robot in eligible_robots2_2:
        #                 distance = robot.distance_to(workstation4)
        #                 if distance < min_distance:
        #                     min_distance = distance
        #                     min_robot_id = robot.rid
        #             # 对最小距离的robot执行任务
        #             lock_define(min_robot_id, workstation4, 1)
        #
        #     # 情况2 原材料都没生产好，执行流程3，找材料2，1
        #
        #     else:
        #         # 判断原材料2
        #         material = workstation4.material_slot
        #         if not material & 0b100:
        #             # 1.判断小车上是否有材料2
        #             eligible_robots3_1 = []
        #             eligible_robots3_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 2 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 2 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots3_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots3_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots3_1:
        #                     distance = robot.distance_to(workstation4)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #
        #                 # 对最小距离的robot执行任务
        #                 if workstation4.material_slot & 0b100 == False:
        #                     lock_define(min_robot_id, workstation4, 0)
        #
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #             # 2. 判断材料2是否生产好但没有拿.
        #             workstation2, over_produce2 = workstation4.find_latest_workstation(2)
        #             # if over_produce2:
        #
        #             # 执行过程2，找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot, workstation2)):
        #                     eligible_robots3_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots3_2) > 0:
        #                 for robot in eligible_robots3_2:
        #                     distance = robot.distance_to(workstation2)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation2, 1)
        #
        #         material = workstation4.material_slot
        #         if not material & 0b10:
        #             # 1.判断小车上是否有材料1
        #             eligible_robots4_1 = []
        #             eligible_robots4_2 = []
        #             for robot in robots:
        #                 if robot.carry_type == 1 and robot.state == True:
        #                     pass
        #                 elif robot.carry_type == 1 and robot.state == False:
        #                     # 如果机器人满足条件，将其添加到列表中
        #                     eligible_robots4_1.append(robot)
        #                 else:
        #                     # 机器人没有生产好的物品
        #                     pass
        #             if len(eligible_robots4_1) > 0:
        #                 min_distance = 100000
        #                 min_robot_id = 0
        #                 for robot in eligible_robots4_1:
        #                     distance = robot.distance_to(workstation4)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #
        #                 # 对最小距离的robot执行任务
        #                 if workstation4.material_slot & 0b10 == False:
        #                     lock_define(min_robot_id, workstation4, 0)
        #             #   判断是否需要继续运行
        #             if allow_run():
        #                 return
        #
        #             # 2. 判断工作台是否有生产好的材料1
        #             workstation1, over_produce1 = workstation4.find_latest_workstation(1)
        #             # if over_produce1:
        #             # 找距离最近的机器人
        #             for robot in robots:
        #                 if not robot.state and (robot.carry_type == 0 or judge_send(robot,workstation1)):
        #                     eligible_robots4_2.append(robot)
        #             min_distance = 10000
        #             if len(eligible_robots4_2) > 0:
        #                 for robot in eligible_robots4_2:
        #                     distance = robot.distance_to(workstation1)
        #                     if distance < min_distance:
        #                         min_distance = distance
        #                         min_robot_id = robot.rid
        #                 # 对最小距离的robot执行任务
        #                 lock_define(min_robot_id, workstation1, 1)

    # 流程2：判断差哪个原材料，对差的每个原材料进行遍历，对所有情况0，执行贪心过程0，对所有情况1执行贪心过程2，情况1都执行完后，对情况2执行流程3：
    # 0. 有生产好的物品,在机器人身上
    # 1. 原材料生产好但没有拿.
    # 2. 原材料都没生产好，

    # 流程3：依次判断差哪个原材料(6-4)，对差的每个原材料所需原料(1-3)进行遍历，对所有情况0，执行贪心过程0，对所有情况1执行贪心过程3，情况1都执行完后，对情况2执行贪心4：
    # 0. 有生产好的物品,在机器人身上
    # 1. 原材料生产好但没有拿.
    # 2. 原材料都没生产好，

    # 贪心过程0: 遍历判断当时所有携带生产好的物品的机器人是否工作态，如工作态则不做干扰，如空闲态则求到目标工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态

    # 贪心过程1: 对所有空闲态机器人求到目标工作台7的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态
    # 任务流程：先去取货，（然后对所有ID为8或者9的工作台计算距离）。

    # 贪心过程2:  对所有空闲态机器人求到目标工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态
    # 任务流程：先去取货，（然后返回需要材料的7号工作台）。

    # 贪心过程3:  对所有空闲态机器人求到目标工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态
    # 任务流程：先去取货，（然后返回需要材料的工作台）。

    # 贪心过程4： 对所有空闲机器人求到能生产目标原材料的工作台的距离，存入list中，取出最小距离的机器人执行任务，并将机器人状态标记为工作态
    # 任务流程：先去等待并取货，（然后返回需要材料的工作台）。
    return have_8

def find_the_actions(robots):
    robots_acts = []
    robots_dists = [[], [], [], []]
    d = [[0,0],[0,0],[0,0],[0,0]]

    for i in range(4):
        for j in range(4):
            dis = ((robot_allocation_lists[i].local_place[0] - robot_allocation_lists[j].local_place[0]) ** 2 +
                   (robot_allocation_lists[i].local_place[1] - robot_allocation_lists[j].local_place[1]) ** 2) ** 0.5
            robots_dists[i].append(dis)

    for i in range(len(robots)):
        if robots[i].state == -1:
            act = RobotCommand()
            act.robot_id = i
            act.command_type = "destroy"
            act.command_param = None
            robots_acts.append(act)
            continue

        # dist = math.dist(robot_allocation_lists[i].dst_place, robot_allocation_lists[i].local_place)
        dist = ((robot_allocation_lists[i].dst_place[0] - robot_allocation_lists[i].local_place[0]) ** 2 +
                (robot_allocation_lists[i].dst_place[1] - robot_allocation_lists[i].local_place[1]) ** 2) ** 0.5

        sys.stderr.write("-----------" + str(dist) + "\n")

        if dist <= 0.4:

            if (robots[i].carry_type == 0) and (workstations[robots[i].workstation_id].product_slot == 1):
                act = RobotCommand()
                act.robot_id = i
                act.command_type = "buy"
                act.command_param = None
                robots_acts.append(act)

            elif (robots[i].carry_type == 0) and (workstations[robots[i].workstation_id].product_slot == 0):
                act = RobotCommand()
                act.robot_id = i
                act.command_type = "forward"
                act.command_param = 0.0
                robots_acts.append(act)

                act = RobotCommand()
                act.robot_id = i
                act.command_type = 'rotate'
                act.command_param = 0.0
                robots_acts.append(act)

            elif (robots[i].carry_type != 0) and (workstations[robots[i].workstation_id].wid > 3):
                act = RobotCommand()
                act.robot_id = i
                act.command_type = "sell"
                act.command_param = None
                robots_acts.append(act)

        else:
            dst_orientation = math.atan2(
                robot_allocation_lists[i].dst_place[1] - robot_allocation_lists[i].local_place[1],
                robot_allocation_lists[i].dst_place[0] - robot_allocation_lists[i].local_place[0])
            if abs(dst_orientation - robots[i].orientation) >= 3 * math.pi / 2:

                if dst_orientation - robots[i].orientation > 0:
                    if (robot_allocation_lists[i].local_place[0] >= 49.0) or (
                            robot_allocation_lists[i].local_place[1] >= 49.0) or (
                            robot_allocation_lists[i].local_place[0] <= 1.0) or (
                            robot_allocation_lists[i].local_place[0] <= 1.0):
                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = "forward"
                        act.command_param = 6.0

                        robots_acts.append(act)

                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = 'rotate'
                        act.command_param = - math.pi
                        robots_acts.append(act)

                    else:
                        if (dist <= 2) and (abs(dst_orientation - robots[i].orientation) >= 0.06 * math.pi):
                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "forward"
                            act.command_param = 0

                            robots_acts.append(act)

                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "rotate"
                            act.command_param = - math.pi

                            robots_acts.append(act)


                        else:
                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "forward"
                            act.command_param = 6.0

                            robots_acts.append(act)

                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = 'rotate'
                            # act.command_param = (dst_orientation - robots[i].orientation - 2 * math.pi) * (
                            #         2 * math.pi - dst_orientation - robots[i].orientation) / (math.pi / 2)
                            # if abs(dst_orientation - robots[i].orientation - 2 * math.pi) >= 0.001:
                            #     robots_acts.append(act)
                            act.command_param = (dst_orientation - robots[i].orientation - 2 * math.pi) * 2
                            robots_acts.append(act)

                elif dst_orientation - robots[i].orientation < 0:
                    if (robot_allocation_lists[i].local_place[0] >= 49.0) or (
                            robot_allocation_lists[i].local_place[1] >= 49.0) or (
                            robot_allocation_lists[i].local_place[0] <= 1.0) or (
                            robot_allocation_lists[i].local_place[0] <= 1.0):
                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = "forward"
                        act.command_param = 6.0

                        robots_acts.append(act)

                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = 'rotate'
                        act.command_param = math.pi
                        robots_acts.append(act)
                    else:
                        if (dist <= 2) and (abs(dst_orientation - robots[i].orientation) >= 0.06 * math.pi):
                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "forward"
                            act.command_param = 0

                            robots_acts.append(act)

                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "rotate"
                            act.command_param = math.pi

                            robots_acts.append(act)

                        else:
                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = "forward"
                            act.command_param = 6.0

                            robots_acts.append(act)

                            act = RobotCommand()
                            act.robot_id = i
                            act.command_type = 'rotate'
                            # act.command_param = (dst_orientation - robots[i].orientation + 2 * math.pi) * (
                            #         2 * math.pi + dst_orientation - robots[i].orientation) / (math.pi / 2)
                            act.command_param = (dst_orientation - robots[i].orientation + 2 * math.pi) * 2
                            # if abs(dst_orientation - robots[i].orientation + 2 * math.pi) >= 0.001:
                            #     robots_acts.append(act)
                            robots_acts.append(act)


            elif (abs(dst_orientation - robots[i].orientation) >= math.pi) and (
                    abs(dst_orientation - robots[i].orientation) < 3 * math.pi / 2):
                act = RobotCommand()
                act.robot_id = i
                act.command_type = "forward"
                act.command_param = -2.0

                robots_acts.append(act)

                # act = RobotCommand(robot_id=i, command_type='rotate',
                #                    command_param=(dst_orientation - Robot[i].orientation + 2 * math.pi) * (
                #                                 dst_orientation - Robot[i].orientation) / math.pi)

                if dst_orientation - robots[i].orientation > 0:
                    act = RobotCommand()
                    act.robot_id = i
                    act.command_type = 'rotate'
                    act.command_param = dst_orientation - robots[i].orientation - 2 * math.pi
                    robots_acts.append(act)

                elif dst_orientation - robots[i].orientation < 0:
                    act = RobotCommand()
                    act.robot_id = i
                    act.command_type = 'rotate'
                    act.command_param = dst_orientation - robots[i].orientation + 2 * math.pi
                    robots_acts.append(act)


            elif (abs(dst_orientation - robots[i].orientation) >= math.pi / 2) and (
                    abs(dst_orientation - robots[i].orientation) < math.pi):
                act = RobotCommand()
                act.robot_id = i
                act.command_type = "forward"
                act.command_param = -2.0

                robots_acts.append(act)

                act = RobotCommand()
                act.robot_id = i
                act.command_type = 'rotate'
                act.command_param = dst_orientation - robots[i].orientation

                robots_acts.append(act)

            else:

                # act = RobotCommand(robot_id=i, command_type='rotate',
                #                    command_param=(dst_orientation - Robot[i].orientation) * (
                #                            dst_orientation - Robot[i].orientation) / math.pi)
                if (robot_allocation_lists[i].local_place[0] >= 49.0) or (
                        robot_allocation_lists[i].local_place[1] >= 49.0) or (
                        robot_allocation_lists[i].local_place[0] <= 1.0) or (
                        robot_allocation_lists[i].local_place[0] <= 1.0):
                    act = RobotCommand()
                    act.robot_id = i
                    act.command_type = "forward"
                    act.command_param = 6.0

                    robots_acts.append(act)

                    act = RobotCommand()
                    act.robot_id = i
                    act.command_type = 'rotate'
                    act.command_param = (dst_orientation - robots[i].orientation) / abs(
                        dst_orientation - robots[i].orientation) * math.pi
                    robots_acts.append(act)
                else:
                    if (dist <= 2) and (abs(dst_orientation - robots[i].orientation) >= 0.06 * math.pi):
                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = "forward"
                        act.command_param = 0

                        robots_acts.append(act)

                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = "rotate"
                        act.command_param = (dst_orientation - robots[i].orientation)/abs(dst_orientation - robots[i].orientation) * math.pi

                        robots_acts.append(act)

                    else:
                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = "forward"
                        act.command_param = 6.0

                        robots_acts.append(act)

                        act = RobotCommand()
                        act.robot_id = i
                        act.command_type = 'rotate'
                        # act.command_param = ( dst_orientation - robots[i].orientation ) * abs(
                        #                             dst_orientation - robots[i].orientation) / (math.pi/2)
                        act.command_param = (dst_orientation - robots[i].orientation) * 2

                        # if abs(dst_orientation - robots[i].orientation) >= 0.001:
                        #     robots_acts.append(act)
                        robots_acts.append(act)

    for i in range(4):
        for j in range(4):
            distj = ((robot_allocation_lists[i].dst_place[0] - robot_allocation_lists[i].local_place[0]) ** 2 +
                    (robot_allocation_lists[i].dst_place[1] - robot_allocation_lists[i].local_place[1]) ** 2) ** 0.5
            orientation1 = math.atan2(
                robot_allocation_lists[i].local_place[1] - robot_allocation_lists[j].local_place[1],
                robot_allocation_lists[i].local_place[0] - robot_allocation_lists[j].local_place[0])
            if (robots_dists[i][j] <= 8.0) and (robots_dists[i][j] > 1.0) \
                    and (abs(robots[i].orientation - robots[j].orientation) >= 0.9 * math.pi) \
                    and (abs(robots[i].orientation - robots[j].orientation) <= 1.1 * math.pi):
                if abs(orientation1 - robots[i].orientation) <= 0.1 * math.pi or abs(
                        orientation1 - robots[j].orientation) <= 0.1 * math.pi:
                    for k in range(len(robots_acts)):
                        if robots_acts[k].robot_id == j and robots_acts[k].command_type == 'rotate' and distj > 5.0:
                            robots_acts[k].command_param = math.pi

                else:
                    pass

            else:
                pass


    return robots_acts


def have_it(position):
    for robot_allocation_list in robot_allocation_lists:
        if robot_allocation_list.dst_place == position:
            return True
    return False


def all_judge():
    for robot in robots:
        if robot.state == True:
            return False
    return True


def judge_material2(dst_place, carry_type):
    # 4的原材料为 1 2
    for workstation in workstations:
        if workstation.position == dst_place:
            if carry_type == 1 or carry_type == 2 or carry_type == 3:
                if workstation.wid == 4:
                    if workstation.material_slot & carry_type == 0:
                        # 缺他 则返回True
                        return True
                # 5的原材料为 1 3
                if workstation.wid == 5:
                    if workstation.material_slot & carry_type == 0:
                        # 缺他 则返回True
                        return True
                # 6的原材料为 2 3
                if workstation.wid == 6:
                    if workstation.material_slot & carry_type == 0:
                        # 缺他 则返回True
                        return True

    return False


def judge_materials(woks, carry_type):
    for wokstation in workstations:
        if woks != wokstation:
            if judge_material(wokstation, carry_type) and lists_taget_define(wokstation.position):
                return True

    return False


def order_nearstwokstation(robot):
    if robot.carry_type != 0:
        for woks in workstations:
            # 如果缺原材料并且没人上锁
            if judge_material(woks, robot.carry_type) and lists_taget_define(woks.position):
                robot_allocation_lists[robot.rid].allocate_task(robots[robot.rid].position, woks.position)
                return
    if robot.carry_type == 0:
        for woks in workstations:
            # 如果缺原材料并且没人上锁
            if woks.product_slot == 1 and lists_taget_define(woks.position) and judge_materials(woks, woks.wid):
                robot_allocation_lists[robot.rid].allocate_task(robots[robot.rid].position, woks.position)
                return


def go_nearest(robot):
    min_woks_place = [0, 0]
    min_distance = 10000

    for workstation in workstations:
        # 如果缺此原材料 并且 且目标工作台ID必须相同 没上锁，则可去 找最近的
        if workstation.wid == work_posToid(robot_allocation_lists[robot.rid].dst_place):
            if judge_material(workstation, robot.carry_type):
                if robot.distance_to(workstation) < min_distance:
                    min_distance = robot.distance_to(workstation)
                    min_woks_place = workstation.position
    # 如果最终找到的目标工作台不是现在的目标工作台，则分布任务
    if min_woks_place != [0, 0] and robot_allocation_lists[
        robot.rid].dst_place != min_woks_place and lists_taget_define(min_woks_place):
        robot_allocation_lists[robot.rid].allocate_task(robots[robot.rid].position, min_woks_place)


def is_need_buy(robot_allocation_list):
    if work_posToid(robot_allocation_list.dst_place) == 7:
        return
    for workstation in workstations:
        if judge_material(workstation, work_posToid(robot_allocation_list.dst_place)):
            return
    robot_allocation_list.dst_place = [20, 20]
    robots[robot_allocation_list.ra_id].state = False

def free_to_take(id):
    for workstation in workstations:
        old_robot_id = -1
        if workstation.wid == id and workstation.product_slot == 1:
            for robot in robots:
                if robot_allocation_lists[robot.rid].dst_place == workstation.position:
                    old_robot_id = robot.rid
            # 在未携带物品的机器人中取最近的去（不考虑是否空闲）
            go_robots = []
            for robot in robots:
                if robot.carry_type == 0:
                    go_robots.append(robot)
            if len(go_robots) > 0:
                min_distance = 10000
                min_robot_id = go_robots[0].rid
                for go_robot in go_robots:
                    if go_robot.distance_to(workstation) < min_distance:
                        min_distance = go_robot.distance_to(workstation)
                        min_robot_id = go_robot.rid
                if old_robot_id != min_robot_id:
                    robot_allocation_lists[min_robot_id].dst_place = workstation.position
                    robots[min_robot_id].state = True
                    if robots[old_robot_id].carry_type==0:
                        robot_allocation_lists[old_robot_id].dst_place = [20, 20]
                        robots[min_robot_id].state = False


def works7_nearst():
    free_to_take(7)
    if judge_material(workstations[7], 4):
        free_to_take(4)
    if judge_material(workstations[7], 5):
        free_to_take(5)
    if judge_material(workstations[7], 6):
        free_to_take(6)


def judge_all_7lose():
    for workstation in workstations:
        if workstation.wid==7 and judge_material(workstation,6) and judge_material(workstation,5) and judge_material(workstation,4):
            return True
    return False


def go_to_one(robot):
    for workstation in workstations:
        if workstation.wid==6 and workstation.product_slot==1 and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place=workstation.position
            return
        if workstation.wid == 5 and workstation.product_slot==1  and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place = workstation.position
            return
        if workstation.wid == 4 and workstation.product_slot==1  and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place = workstation.position
            return
        if workstation.wid == 3 and workstation.product_slot==1  and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place = workstation.position
            return
        if workstation.wid == 2 and workstation.product_slot==1  and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place = workstation.position
            return
        if workstation.wid == 1 and workstation.product_slot==1  and lists_taget_define(workstation.position):
            robot_allocation_lists[robot.rid].dst_place = workstation.position
            return

    pass


if __name__ == '__main__':

    # 初始化阶段
    # 1. 判题器将地图数据输入程序，以OK结束（输入）
    # read_util_ok()
    maps, robots, workstations, robot_allocation_lists = init()

    # 2. 程序输出OK（输出）
    finish()


    while True:

        # 运行阶段，重复两个步骤： 1. 判题器给一帧场面信息，以OK结束（输入） 2.程序返回一帧控制指令，以OK结束（输出）

        # 输入的帧示例，2+1+K+4+1 行

        # 帧序号 金钱数
        # 工作台数量K
        # 工作台 坐标 剩余生产时间 原材料格状态 产品格状态 （工作台）
        # ...
        # 工作台 坐标 剩余生产时间 原材料格状态 产品格状态
        # 所处工作台ID 携带物品类型 时间价值系数 碰撞价值系数（机器人）角速度  线速度 朝向 坐标
        # ...
        # 所处工作台ID 携带物品类型 时间价值系数 碰撞价值系数 角速度  线速度 朝向 坐标
        # OK

        # 输出帧示例，1+N+1 行

        # 帧序号
        # 指令 <机器人 ID> [参数 2]
        # ...
        # 指令 <机器人 ID> [参数 2]
        # OK

        # 步骤1
        # 第一行
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')

        # 帧序号
        frame_id = int(parts[0])

        # 金钱数
        frame_money = int(parts[1])

        # 工作台数量K
        # 第二行
        workstation_number = int(sys.stdin.readline())

        # 提取工作台信息
        # 读K行
        # sys.stderr.write(str(workstations[1].position) + "\n")
        for i in range(workstation_number):
            workstation_parts = sys.stdin.readline().strip().split(' ')
            # 每次更新工作台的信息，而不是生成新的工作台存储

            wid = int(workstation_parts[0])


            workstations[i].position = [float(workstation_parts[1]), float(workstation_parts[2])]

            workstations[i].remaining_time = int(workstation_parts[3])
            workstations[i].material_slot = int(workstation_parts[4])
            workstations[i].product_slot = int(workstation_parts[5])

            # 提取机器人信息
        # sys.stderr.write(str(workstations[1].position) + "\n")
        for i in range(4):
            # 每次更新机器人的信息，而不是生成新的机器人存储
            robot_parts = sys.stdin.readline().strip().split(' ')
            rid = i
            robot = robots[rid]
            robot.workstation_id = int(robot_parts[0])
            robot.carry_type = int(robot_parts[1])
            robot.time_value = float(robot_parts[2])
            robot.collision_value = float(robot_parts[3])
            robot.angular_velocity = float(robot_parts[4])
            robot.linear_velocity = [float(robot_parts[5]), float(robot_parts[6])]
            robot.orientation = float(robot_parts[7])
            robot.position = [float(robot_parts[8]), float(robot_parts[9])]

        # 最后一位是OK
        read_util_ok()

        # for x in workstation:
        #     sys.stderr.write(x+"\n")

        # 只有15ms用来处理任务

        robot_acts = []
        st_robots_lists = robot_allocation_lists

        have_8 = False
        have_9=False
        for workstation in workstations:
            if workstation.wid==9:
                have_9=True
        have_8=greedy_algorithm_ergodic_retrieval(have_8)
        # if st_robots_lists == robot_allocation_lists and all_judge():
        #     min_distance = 1000
        #     min_position = [0, 0]
        #     allow_destroy = True
        #
        #     for robot in robots:
        #         if robot.carry_type == 0:
        #             allow_destroy = False
        #             for woks in workstations:
        #                 if woks.wid == 1 or woks.wid == 2 or woks.wid == 3:
        #                     # 如果工作台坐标不在任务list中
        #                     if not have_it(woks.position):
        #                         if min_distance > robot.distance_to(woks):
        #                             min_distance = robot.distance_to(woks)
        #                             min_position = woks.position
        #             robot_allocation_lists[robot.rid].allocate_task(robot.position, min_position)
        #     if allow_destroy == True and all_judge():
        #         for robot in robots:
        #             if robot.rid == 1 and (robot.carry_type == 1 or robot.carry_type == 2 or robot.carry_type == 3):
        #                 robot.state = -1
        #                 break

        # 。

        # 如果有7生产好 则找最近的距离7号工作台的不携带物品的机器人
        # 如果有9号工作台并且只剩最后30秒 并且场上所有7号工作台都缺所有原材料
        if have_9 and frame_id>=7500:
            if judge_all_7lose():
                for workstation in  workstations:
                    if workstation.wid==9:
                        for robot in robots:
                            if robot.carry_type!=0:
                                robot_allocation_lists[robot.rid].dst_place=workstation.position
                            #如果机器人身上没背负物品，则取依次取6 5 4 3 2 1
                            else:
                                go_to_one(robot)


        if have_8 !=False:
            works7_nearst()
            for robot_allocation_list in robot_allocation_lists:
                if robots[robot_allocation_list.ra_id].carry_type!=7:
                    if robot_allocation_list.dst_place == [20, 20] :
                        order_nearstwokstation(robots[robot_allocation_list.ra_id])
                    else:
                        # 找离自己最近的差原料的工作台
                        # if robots[robot_allocation_list.ra_id].carry_type != 0 :
                        #     go_nearest(robots[robot_allocation_list.ra_id])
                        # 如果不带东西且有目的工作台 则是去取 判断是否有必要取这个货

                        if robots[robot_allocation_list.ra_id].carry_type == 0:
                            is_need_buy(robot_allocation_list)

        # robot_allocation_lists[0] = robot_allocation_list(0, robots[0].position, workstations[0].position,True)
        # sys.stderr.write(str(workstations[0].position)+"\n")
        # robot_allocation_lists[1] = robot_allocation_list(1, robots[1].position, workstations[1].position,True)
        # sys.stderr.write(str(workstations[1].position) + "\n")
        # robot_allocation_lists[2] = robot_allocation_list(2, robots[2].position, workstations[2].position,True)
        # sys.stderr.write(str(workstations[2].position) + "\n")
        # robot_allocation_lists[3] = robot_allocation_list(3, robots[3].position, workstations[3].position,True)
        # sys.stderr.write(str(workstations[3].position) + "\n")
        # # 根据list，来求机器人动作
        for i in range(4):
            sys.stderr.write(
                "\n" + str(robot_allocation_lists[i].ra_id) + str(robot_allocation_lists[i].local_place) + str(
                    robot_allocation_lists[i].dst_place) + str(robot_allocation_lists[i].state) + "\n")
        # break
        robots_acts = find_the_actions(robots)

        # 输出动作,工作态变化等都得添加，根据现有机器人状态。
        allout_robots_acts(robots_acts)
        finish()

    # 步骤2
    # sys.stdout.write('%d\n' % frame_id)
    # line_speed, angle_speed = 3, 1.5
    # for robot_id in range(4):
    #     sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
    #     sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
    # finish()
