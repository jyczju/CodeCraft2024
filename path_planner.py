import heapq
import queue
import time

import numpy as np

import sys

# 定义节点类
class Node:
    def __init__(self, x, y, g, h, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent = parent

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h

class Path_Planner():

    def __init__(self, map_info):
        self.map_info = map_info

    # 定义启发函数
    def heuristic(self, node, goal):
        return abs(node.x - goal.x) + abs(node.y - goal.y)


    # 定义A*算法函数
    def astar(self, start_info, end_info):
        # sys.stderr.write("astar begin\n")

        start = Node(start_info[0], start_info[1], 0, 0, None)
        goal = Node(end_info[0], end_info[1], 0, 0, None)
        grid = self.map_info

        # 初始化open列表和closed列表
        open_list = []
        closed_list = set()

        # 将起点加入open列表
        heapq.heappush(open_list, start)

        # start_time = time.time()
        # 循环直到找到终点或open列表为空
        loop = 0
        while open_list:

            # current_time = time.time()
            # if current_time - start_time >= 0.1:
            #     time.sleep(0.00001)
            #     start_time = time.time()

            # loop += 1
            # if loop >= 333:# *10000*3
            #     time.sleep(0.0001) #*0.1
            #     loop = 0

            # loop += 1
            # if loop >= 10000000:
            #     time.sleep(0.00001)
            #     loop = 0

            # 从open列表中选择一个节点
            current = heapq.heappop(open_list)

            # 如果该节点是终点，则搜索结束
            if current.x == goal.x and current.y == goal.y:
                path = []
                while current.parent:
                    path.append((current.x, current.y))
                    current = current.parent
                path.append((current.x, current.y))
                return path[::-1]

            # 将该节点从open列表中删除，并将其加入closed列表
            closed_list.add((current.x, current.y))

            # 遍历该节点的所有邻居节点
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                x = current.x + dx
                y = current.y + dy

                # 如果邻居节点不在grid中，则跳过
                if x < 0 or x >= len(grid) or y < 0 or y >= len(grid[0]) or grid[x][y] == 1 or grid[x][y] == 2:
                    continue

                # 如果邻居节点已经在closed列表中，则跳过
                if (x, y) in closed_list:
                    continue

                # 计算邻居节点到终点的距离
                h = self.heuristic(Node(x, y, 0, 0, None), goal) * 1.5 # weighted a*

                # 计算邻居节点到起的距离
                g = current.g + 1

                # 如果邻居节点不在open列表中，则将其加入open列表，并记录其父节点和到起点的距离
                if (x, y) not in [(node.x, node.y) for node in open_list]:
                    heapq.heappush(open_list, Node(x, y, g, h, current))

                # 如果邻居节点已经在open列表中，则比较其到起点的距离，如果新的距离更小，则更新其父节点和到起点的距离
                else:
                    for node in open_list:
                        if node.x == x and node.y == y:
                            if g < node.g:
                                node.g = g
                                node.parent = current

        # 如果open列表为空，则搜索失败
        # return None
        return []

if __name__ == '__main__':
    # 定义迷宫
    def read_map_txt_file(file_path):
            # 打开文件并读取内容
            with open(file_path, 'r') as file:
                lines = file.readlines()

            # 初始化二维数组
            matrix = []

            # 处理每一行数据
            for line in lines:
                # 去除行末的换行符并按字符分割数据
                data = list(line.strip())
                
                # 将数据添加到二维数组中
                matrix.append(data)

            return matrix
    file_path = r'D:\OneDrive - zju.edu.cn\HWCodeCraft\WindowsRelease\maps\map8.txt'
    matrix = read_map_txt_file(file_path)

    map_info = np.zeros((200,200))

    for i in range(len(matrix)):
        for j in range(len(matrix[0])):

            if str(matrix[i])[j] == '.':
                map_info[i][j] = 0
            elif str(matrix[i])[j] == '*':
                map_info[i][j] = 1
            elif str(matrix[i])[j] == '#':
                map_info[i][j] = 2
            elif str(matrix[i])[j] == 'A':
                map_info[i][j] = 0 # 机器人的位置也是空地
            elif str(matrix[i])[j] == 'B':
                map_info[i][j] = 4

    
    
    # 定义起点和终点
    start_info = (81,32)  #定义info信息为15
    end_info = (32,67)  #定义info信息为20
    
    

    import time
    start_time = time.time()

    planner = Path_Planner(map_info)

    # 使用A*算法寻找最短路径
    path = planner.astar(start_info, end_info)
    # 打印最短路径
    print(path)


    end_time = time.time()
    execution_time = end_time - start_time
    print(f"New A*路径规划1用时为: {execution_time} 秒")

    

