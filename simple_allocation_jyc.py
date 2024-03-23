import numpy as np
import sys

robot_num = 10


class Node:
    def __init__(self, point, left=None, right=None):
        self.point = point
        self.left = left
        self.right = right


class KDTree:
    def __init__(self, points, depth=0):
        # print_log = open("./printlog.txt",'a')
        # print("points",file = print_log)
        # print(points,file = print_log)
        # print_log.close()

        k = len(points[0])
        axis = depth % k

        points.sort(key=lambda x: x[axis])
        median = len(points) // 2

        self.root = Node(points[median])
        if median > 0:
            self.root.left = KDTree(points[:median], depth + 1).root
        if median + 1 < len(points):
            self.root.right = KDTree(points[median + 1:], depth + 1).root


def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def find_nearest_cargos(kd_tree, robot, num = 3):
    def search(node, depth=0, best=[]):
        if node is None:
            return best

        axis = depth % 2
        next_branch = None
        opposite_branch = None

        if robot[axis] < node.point[axis]:
            next_branch = node.left
            opposite_branch = node.right
        else:
            next_branch = node.right
            opposite_branch = node.left

        best = search(next_branch, depth + 1, best)

        # 如果当前点比当前最近的点集合里的最远点还要近，则添加该点到最近点集合里
        if len(best) < num or manhattan_distance(robot, node.point) < manhattan_distance(robot, best[-1]):
            best.append(node.point)
            best.sort(key=lambda x: manhattan_distance(robot, x))
            if len(best) > num:
                best = best[:num]

        # 如果机器人与当前节点垂直轴的距离比当前最近的点集合里的最远点还要近，则搜索另一侧子树
        if len(best) < num or abs(robot[axis] - node.point[axis]) < manhattan_distance(robot, best[-1]):
            best = search(opposite_branch, depth + 1, best)

        return best

    return search(kd_tree.root)

def find_best_nearest_cargo(kd_tree, robot):
    def search(node, depth=0, best=None):
        if node is None:
            return best

        axis = depth % 2
        next_branch = None
        opposite_branch = None

        if robot[axis] < node.point[axis]:
            next_branch = node.left
            opposite_branch = node.right
        else:
            next_branch = node.right
            opposite_branch = node.left

        best = search(next_branch, depth + 1, best)

        # 如果当前点比当前最近的点更近，则更新最近点
        if best is None or manhattan_distance(robot, node.point) < manhattan_distance(robot, best):
            best = node.point

        # 如果机器人与当前节点垂直轴的距离比当前最近的点更近，则搜索另一侧子树
        if best is None or abs(robot[axis] - node.point[axis]) < manhattan_distance(robot, best):
            best = search(opposite_branch, depth + 1, best)

        return best

    return search(kd_tree.root)



def Robot_task(robot, gbmap, berth, num, now_frame_id, boat, gds_add_num, allocation_aid):  # 读取货物位置，为机器人分配搬运(get)任务
    # 修改了robot类，加入是否完成规划的标志位
    for robot_x in robot:
        if robot_x.allocation_status == 0 and robot_x.status == 1:  # 还没有给出规划命令
            # 分类讨论，应该执行get还是pull任务
            if robot_x.goods == 0:  # 执行get任务
                if len(gbmap.gds_dict) >= 20:
                    # 找到最近的二十个货物，便于权衡，但不知道运行时间会不会很长
                    best = find_nearest_cargos(kd_tree, (robot_x.x, robot_x.y), num = 20)  # 返回格式为[(),(),()]

                    # print_log = open("./printlog.txt",'a')
                    # print("best:",best,file = print_log)
                    # print_log.close()
                    value = []
                    for best_x in best: # best_x是()格式
                        
                        if robot_x.reachable_region[best_x[0]][best_x[1]] == 0: # 如果该货物处于不可达区域
                            value.append(-1000)
                        else:
                            value.append(gbmap.gds_dict[best_x][0]/manhattan_distance((robot_x.x, robot_x.y), best_x))

                    if max(value) > 0:
                        max_index = value.index(max(value))

                        robot_x.mbx = best[max_index][0]
                        robot_x.mby = best[max_index][1]
                        # 更新分配任务的状态
                        robot_x.allocation_status = 1
                        # 分配后直接删除字典中的货物，免得重复分配
                        allocation_aid.sum_value += gbmap.gds_dict[(robot_x.mbx, robot_x.mby)][0]
                        gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)  # 删除货物字典中的货物
                        # 更新kdtree
                        update_kd_tree(gbmap)

                    else: # 最近的三个货物都不可达
                        robot_x.mbx = robot_x.x
                        robot_x.mby = robot_x.y
                        robot_x.allocation_status = 0
                        gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)  # 删除货物字典中的货物
                        # 更新kdtree
                        update_kd_tree(gbmap)

                elif len(gbmap.gds_dict) > 0:
                    # # 直接按顺序分配
                    # best = list(gbmap.gds_dict.keys())[0]
                    # if robot_x.reachable_region[best[0]][best[1]] == 1: # 如果货物处于可达区域
                    #     robot_x.mbx = best[0]
                    #     robot_x.mby = best[1]
                    #     # 更新分配任务的状态
                    #     robot_x.allocation_status = 1
                    #     # 分配后直接删除字典中的货物，免得重复分配
                    #     allocation_aid.sum_value += gbmap.gds_dict[(robot_x.mbx, robot_x.mby)][0]
                    #     gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)
                    #     # 更新kdtree
                    #     update_kd_tree(gbmap)

                    # else: # 货物不处在可达区域
                    #     robot_x.mbx = robot_x.x
                    #     robot_x.mby = robot_x.y
                    #     robot_x.allocation_status = 0
                    #     gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)  # 删除货物字典中的货物
                    #     # 更新kdtree
                    #     update_kd_tree(gbmap)

                    # 找到最近的三个货物，便于权衡，但不知道运行时间会不会很长
                    best = find_nearest_cargos(kd_tree, (robot_x.x, robot_x.y), num = len(gbmap.gds_dict))  # 返回格式为[(),(),()]

                    value = []
                    for best_x in best:
                        if robot_x.reachable_region[best_x[0]][best_x[1]] == 0: # 如果该货物处于不可达区域
                            value.append(-1000)
                        else:
                            value.append(gbmap.gds_dict[best_x][0]/manhattan_distance((robot_x.x, robot_x.y), best_x))

                    if max(value) > 0:
                        max_index = value.index(max(value))

                        robot_x.mbx = best[max_index][0]
                        robot_x.mby = best[max_index][1]
                        # 更新分配任务的状态
                        robot_x.allocation_status = 1
                        # 分配后直接删除字典中的货物，免得重复分配
                        allocation_aid.sum_value += gbmap.gds_dict[(robot_x.mbx, robot_x.mby)][0]
                        gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)  # 删除货物字典中的货物
                        # 更新kdtree
                        update_kd_tree(gbmap)

                    else: # 最近的三个货物都不可达
                        robot_x.mbx = robot_x.x
                        robot_x.mby = robot_x.y
                        robot_x.allocation_status = 0
                        gbmap.delete_gds_dict(robot_x.mbx, robot_x.mby)  # 删除货物字典中的货物
                        # 更新kdtree
                        update_kd_tree(gbmap)
                        
                else:  # 没有货物了，不分配任务
                    robot_x.mbx = robot_x.x
                    robot_x.mby = robot_x.y
                    robot_x.allocation_status = 0
            # with open('value.txt', 'a') as f:
            #     # 将列表中的每个元素转换为字符串并写入文件
            #     f.write(str(now_frame_id) + '\n')
            #     f.write(str(allocation_aid.sum_value))
            #     f.write('\n')

            if robot_x.goods == 1:  # 执行pull任务

                
                # # best = find_best_nearest_cargo(kd_tree_berth, (robot_x.x, robot_x.y))  # 找到最近的港口
                # best1 = find_nearest_cargos(kd_tree_berth, (robot_x.x, robot_x.y), num = 1)
                # best = best1[0]

                # for berth_x in berth:
                #     if berth_x.x == best[0] and berth_x.y == best[1]:
                #         berth_id = berth_x.num
                #         break

                # if berth[berth_id].park_coordinates != []:  #分配pull地点
                #     robot_x.mbx = berth[berth_id].park_coordinates[0][0]
                #     robot_x.mby = berth[berth_id].park_coordinates[0][1]
                #     berth[berth_id].park_coordinates.pop(0)
                #     robot_x.mb_berth_id = berth_id
                #     # 更新分配任务的状态
                #     robot_x.allocation_status = 1

                best4 = find_nearest_cargos(kd_tree_berth, (robot_x.x, robot_x.y), num = 4)  # 找到最近的港口，返回格式为[(),(),(),(),()]
                # best = best5[0]
                
                best = None
                for best_x in best4:
                    if robot_x.reachable_region[best_x[0]][best_x[1]] == 1:
                        best = best_x
                        break

                if best is not None:
                    for berth_x in berth:
                        if berth_x.x == best[0] and berth_x.y == best[1]:
                            berth_id = berth_x.num
                            break

                    if berth[berth_id].park_coordinates != []:  #分配pull地点
                        robot_x.mbx = berth[berth_id].park_coordinates[0][0]
                        robot_x.mby = berth[berth_id].park_coordinates[0][1]
                        berth[berth_id].park_coordinates.pop(0)
                        robot_x.mb_berth_id = berth_id
                        # 更新分配任务的状态
                        robot_x.allocation_status = 1


                # #更新分配任务的状态
                # robot_x.allocation_status = 1

                # 如果goal都为1000，即都不满足，不进行任务分配，等到满足条件时再分配

        if robot_x.allocation_status == 1:  # 已经给出规划命令
            if robot_x.goods == 0:  # 准备开始执行拿取动作
                if robot_x.x == robot_x.mbx and robot_x.y == robot_x.mby:  # 到达货物地点
                    robot_x.get_good()
                    # 完成get任务，状态重置为0,goods变为1
                    robot_x.allocation_status = 0
                    # robot_x.goods = 1
                    # #同一帧递归完成任务分配
                    # task_allocation_main(robot, gbmap, berth, boat, gds_add_num, now_frame_id)

            if robot_x.goods == 1:  # 准备开始执行卸货动作
                if robot_x.x == robot_x.mbx and robot_x.y == robot_x.mby:  # 到达货物地点
                    robot_x.pull_good()
                    # 完成pull任务，状态重置为0,goods变为0
                    robot_x.allocation_status = 0
                    # 在park_coordinates中添加停车位信息
                    berth[robot_x.mb_berth_id].park_coordinates.append((robot_x.x, robot_x.y))
                    berth[robot_x.mb_berth_id].add_gds_num(1)         #添加货物

                    # robot_x.goods = 0
                    # # 同一帧递归完成任务分配
                    # task_allocation_main(robot, gbmap, berth, boat, gds_add_num, now_frame_id)


def boat_task(berth, boat, now_frame_id, allocation_aid, boat_capacity):  # 轮船任务分配
    if now_frame_id == 1 :  # 第一帧先分配5个berth，按照运输时间短和位置分散两个因素来决定。
        # 考虑到判题器给出的泊位本身就按坐标的顺序排列的，因此简单起见，从相邻的两个泊位之间选出运输时间短的泊位。
        # 如果货物不能堆叠，感觉装载速度影响可以忽略不计，毕竟只有几帧。。。
        i = 0
        j = 0
        while i < 8:
            if (berth[i].transport_time < berth[i + 1].transport_time):
                boat[j].ship(berth[i].num, now_frame_id)
                # allocation_aid.berth_boat_stop.append(berth[i].num)  # 记录一下哪个港口即将有船
                allocation_aid.berth_used.append(berth[i].num)       # 记录使用的泊位
            else:
                boat[j].ship(berth[i + 1].num, now_frame_id)
                # allocation_aid.berth_boat_stop.append(berth[i + 1].num)  # 记录一下哪个港口即将有船
                allocation_aid.berth_used.append(berth[i + 1].num)  # 记录使用的泊位

            j = j + 1
            i = i + 2

        #手动调整一下boat的位置
        boat[j].ship(berth[9].num, now_frame_id)
        # allocation_aid.berth_boat_stop.append(berth[i + 1].num)  # 记录一下哪个港口即将有船
        allocation_aid.berth_used.append(berth[9].num)  # 记录使用的泊位


    elif now_frame_id >= 13000:  # 时间快要结束了，直接开船 # 优化一下，直接开船的时间取决于船到达虚拟点的时间(已完成)
        for boat_x in boat:
            if boat_x.status == 1 and boat_x.pos != -1:  # 处于泊位装货状态，不管满不满，直接开船
                if 15000 - now_frame_id < berth[boat_x.pos].transport_time + 20:  # 留下20帧的余量
                    # allocation_aid.berth_boat_stop.remove(boat_x.pos)  # 开走了船，直接删除该数据
                    boat_x.go()

    else: # 其他时刻，看泊位是否工作，未工作，则综合考虑泊位已有货物和运输时间
        for boat_x in boat:
            # 分类讨论
            # 首先是装载任务
            if boat_x.status == 1 and boat_x.pos != -1:  # 处于泊位装货状态
                if now_frame_id - boat_x.start_id < 1000:   #阈值内
                    if berth[allocation_aid.berth_used[boat_x.num]].now_gds_num > boat_capacity:  #如果刚进来时就能装满
                        boat_x.leaving_time = now_frame_id + boat_capacity/berth[allocation_aid.berth_used[boat_x.num]].loading_speed + 5 #留5帧的余量
                        berth[allocation_aid.berth_used[boat_x.num]].now_gds_num = berth[allocation_aid.berth_used[boat_x.num]].now_gds_num - boat_capacity

                    if boat_x.leaving_time == now_frame_id :
                        boat_x.go()
                else:  #停时间太长了直接走
                    boat_x.go()

                # elif boat_x.leaving_time == 0:
                #     if boat_x.pos in allocation_aid.berth_boat_stop:
                #         allocation_aid.berth_boat_stop.remove(boat_x.pos)  # 开走了船，直接删除该数据
                #     boat_x.go()
                # else:
                #     if boat_x.leaving_time > 0:
                #         boat_x.leaving_time = boat_x.leaving_time - 1

            elif boat_x.status == 1 and boat_x.pos == -1:  # 运输完成，已经到达虚拟点
                # 怎么开过来再怎么开回去
                boat_x.ship(allocation_aid.berth_used[boat_x.num], now_frame_id)
                allocation_aid.berth_boat_stop.append(allocation_aid.berth_used[boat_x.num])  # 船即将开进，添加数据
                boat_x.start_id = berth[allocation_aid.berth_used[boat_x.num]].transport_time + now_frame_id  # 更新start_id

    # with open('boat_status.txt', 'a') as f:
    #     # 将列表中的每个元素转换为字符串并写入文件
    #     f.write(str(now_frame_id) + '\n')
    #     for item in boat:
    #         f.write(str(item.num) + ' ' + str(item.status) + ' ' + str(item.pos) +'\n')  # 在每个元素后添加换行符
    #     f.write('\n')

def update_kd_tree(gbmap):
    goods_coordinate_list = []  # 创建货物坐标列表

    for key, value in gbmap.gds_dict.items():  # 获取当前所有货物的坐标
        goods_coordinate_list.append(key)

    # print_log = open("./printlog.txt",'a')
    # print("goods_coordinate_list",file = print_log)
    # print(goods_coordinate_list,file = print_log)
    # print_log.close()

    if goods_coordinate_list != []:
        # 创建 KD 树,全局变量
        global kd_tree
        kd_tree = KDTree(goods_coordinate_list)


def task_allocation_main(robot, gbmap, berth, boat, num, now_frame_id, allocation_aid, boat_capacity):
    # 改为分配一次后不再分配,最好保证每次循环中KDtree不被删掉，设成全局变量

    del_gds_flag = 0
    del_gds_flag = gbmap.check_gds_dict(now_frame_id)  # 删除超时的货物

    if now_frame_id == 1 or num != 0 or del_gds_flag == 1:  # 如果是第一帧或者新增了货物或者消失了货物或者货物被任务分配了，才更新KDtree
        update_kd_tree(gbmap)


    boat_task(berth, boat, now_frame_id, allocation_aid, boat_capacity)  # 分配轮船的任务

    if now_frame_id == 1 :
        berth_coordinate_list = []

        for berth_x in berth:
            if berth_x.num in allocation_aid.berth_used:
                berth_coordinate_list.append((berth_x.x,berth_x.y))

        # 创建 KD 树,全局变量
        global kd_tree_berth
        kd_tree_berth = KDTree(berth_coordinate_list)

    Robot_task(robot, gbmap, berth, num, now_frame_id, boat, num, allocation_aid)  # 给机器人分配任务

    # 打开文件以写入模式




