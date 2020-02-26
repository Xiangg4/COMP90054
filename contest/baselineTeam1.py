# junTeam.py song2_upgrade
# -*- coding:utf8 -*-
# 在边界周旋的时候，往回走5步
from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint
from game import Actions
import copy


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,  # DefensiveReflexAgent OffensiveReflexAgent
               first='OffensiveReflexAgent', second='DefensiveAgent'):
    """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.
  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):

    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)  # 复活点
        CaptureAgent.registerInitialState(self, gameState)
        self.foodNum = len(self.getFood(gameState).asList())  # 豆子数目
        self.walls = gameState.getWalls().asList()  # 所有的墙
        self.selfBorder, self.enermyBorder , self.selfBorder5, self.enermyBorder5 = self.getBoundaries(gameState)  # 我方边界敌方边界

        self.isChased = False
        self.forceGoPoint = None
        self.myTeamIndexes = self.getTeam(gameState)  # 我方队员的序号
        self.teammateIndex = 0
        # print('Whole Team!!!!!!')
        # print(self.myTeamIndexes)
        for i in range(len(self.myTeamIndexes)):
            if (self.index != self.myTeamIndexes[i]):
                self.teammateIndex = self.myTeamIndexes[i]  # 队友
        # print('Teamate:')
        # print(self.teammateIndex)

    # TODO: Calculate the boundaries using the first 15 seconds
    def getBoundaries(self, gameState):  # 算我方的边界和敌方边界，不含城墙
        grid = gameState.getWalls()
        width = grid.width
        height = grid.height
        selfSide = self.start[0]
        leftBorder = width / 2 - 1
        rightBorder = width / 2
        leftBorder5Step = leftBorder - 5
        rightBorder5step = rightBorder + 5
        selfBorder = []
        selfBorder5 = []
        enermyBorder = []
        enermyBorder5 = []
        selfBorderX = leftBorder
        enermyBorderX = rightBorder
        selfBorderX5 = leftBorder5Step
        enermyBorderX5 = rightBorder5step
        if (selfSide > rightBorder):
            selfBorderX = rightBorder
            selfBorderX5 = rightBorder5step
            enermyBorderX = leftBorder
            enermyBorderX5 = leftBorder5Step

        # TODO: Start to calcultate no wall dot
        for i in range(height):
            if (not grid[selfBorderX][i] and not grid[enermyBorderX][i] and not grid[enermyBorderX5][i] and not grid[selfBorderX5][i]):
                selfBorder.append((selfBorderX, i))
                selfBorder5.append((selfBorderX5, i))
                enermyBorder.append((enermyBorderX, i))
                enermyBorder5.append((enermyBorderX5, i))
        # print('*******************Self Border')
        # print(selfBorder)
        # print('*******************Enermy Border')
        # print(enermyBorder)

        # print('width:')
        # print(grid.width)
        # print('height:')
        # print(grid.height)
        return selfBorder, enermyBorder, selfBorder5, enermyBorder5

    def getSuccessors(self, position):
        # TODO: Add the deadend configuration
        blockers = copy.copy(self.walls)  # 障碍，所有的墙
        #   if(isChased):
        #       blockers.extend(self.deadEndList)
        curOb = self.getCurrentObservation();
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        defenders = [ele for ele in enemies if
                     not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
        if len(defenders) > 0:
            defendersPos = [i.getPosition() for i in defenders]
            blockers.extend(defendersPos)

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = position
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if (nextx, nexty) not in blockers:  # 如果下个方向不是障碍
                nextState = (nextx, nexty)
                successors.append((nextState, action))
        return successors

    def heuristicSearch(self, goalList):
        position = self.getCurrentObservation().getAgentPosition(self.index)  # 自己的位置
        #print position,self.start

        def heuristic(p1, p2):  # 曼哈顿距离
            return util.manhattanDistance(p1, p2)

        for point in goalList:
            collection = util.PriorityQueue()
            visited = []
            #dis = heuristic(position, point)  # 每一个我到目标点的曼哈顿距离都进队
            dis = self.getMazeDistance(position, point)
            collection.push((position, []), dis)
            while not collection.isEmpty():
                tmpPoint, path = collection.pop()  # 距离我最近的点出队
                #print tmpPoint
                if (tmpPoint in visited):#处理过的点
                    continue
                visited.append(tmpPoint)
                if (point == tmpPoint):#目标点就是最近点
                    if (len(path) == 0):
                        continue
                    #print path[0]
                    return path[0]#抵达目标的最佳路径
                #print tmpPoint,point
                successors = self.getSuccessors(tmpPoint)#删掉了point
                #print successors
                for element in successors:
                    fn = len(path + [element[1]]) + self.getMazeDistance(element[0], point)#路径+下一步+下个点的到目标点的曼哈顿距离
                    collection.push((element[0], path + [element[1]]), fn)

        if len(goalList) > 0 and goalList[0] != self.start:#有目标点，第一个目标点不是复活点
            #print [self.start]
            return self.heuristicSearch([self.start])#向复活点靠近
        # if len(goalList)==0:
        #     return 'North'
        return 'Stop'

    def getNiceClosestFood(self, gameState, defendFood=False, num=-1, customPosition=None):
        position = self.getCurrentObservation().getAgentPosition(self.index)#我的位置
        if customPosition != None:
            position = customPosition
        if defendFood:#防守食物
            foodList = self.getFoodYouAreDefending(gameState).asList()
        else:#优先防守吃豆人，默认
            curOb = self.getCurrentObservation();
            position = curOb.getAgentPosition(self.index)#我的位置
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if ele.isPacman and ele.getPosition() != None]#敌方吃豆人列表
            if len(enemiesAtHome) > 0:#有敌方吃豆人
                dists = [self.getMazeDistance(position, ele.getPosition()) for ele in enemiesAtHome]#我和敌方吃豆人的距离（list)
                for i in range(0, len(enemiesAtHome)):
                    dis2Enemy = self.getMazeDistance(position, enemiesAtHome[i].getPosition())#距离赋值
                    if dis2Enemy == min(dists) and dis2Enemy <= 5:#距离近，5格之内
                        return [enemiesAtHome[i].getPosition()]#返回敌方吃豆人位置
            foodList = self.getFood(gameState).asList()#没有敌方吃豆人
        #否则吃对面食物
        collection = util.PriorityQueue()
        for food in foodList:
            dis = self.getMazeDistance(position, food)
            collection.push((food), dis)#进队
        result = []
        if (num < 0):#默认，num食物数量
            # for i in range(collection.count/4):
            for i in range(collection.count):
                result.append(collection.pop())#食物
        else:
            if (collection.count < num):
                num = collection.count
            for i in range(num):
                result.append(collection.pop())#食物
        #   print(len(result))
        return result

    def defenderBestPosition(self, gameState):#家里有敌人，吃人，家里没敌人，巡逻
        curOb = self.getCurrentObservation();
        position = curOb.getAgentPosition(self.index)#我的
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        enemiesAtHome = [ele for ele in enemies if ele.isPacman and ele.getPosition() != None]
        if len(enemiesAtHome) > 0:#家里有敌人
            dists = [self.getMazeDistance(position, ele.getPosition()) for ele in enemiesAtHome]
            for i in range(0, len(enemiesAtHome)):
                if self.getMazeDistance(position, enemiesAtHome[i].getPosition()) == min(dists):
                    return [enemiesAtHome[i].getPosition()]#敌方吃豆人位置
        else:
            defFoodList = self.getNiceClosestFood(gameState, defendFood=True, num=3)
            ranPoint = random.choice(defFoodList)
            if position == ranPoint:
                defFoodList.remove(ranPoint)
                ranPoint = random.choice(defFoodList)
            return [ranPoint]#把家里的食物巡逻一遍

    def getGoals(self, gameState, isDefender):

        if not isDefender:
            return self.getNiceClosestFood(gameState)#不是防守者
        else:
            return self.defenderBestPosition(gameState)  # 防守者

    def getClosestBorder(self, gameState):#
        position = self.getCurrentObservation().getAgentPosition(self.index)#我的位置
        collection = util.PriorityQueue()
        for border in self.selfBorder:#我方边界上每个点
            dis = self.getMazeDistance(position, border)#我到边界的距离
            collection.push((border), dis)
        if collection.count == 0:#找不到
            return [self.start]#复活点
        else:
            return [collection.pop()]#返回最近的边界的点

    def getClosestBorder5(self, gameState):  #
        position = self.getCurrentObservation().getAgentPosition(self.index)  # 我的位置
        collection = util.PriorityQueue()
        for border in self.selfBorder5:  # 我方边界上每个点
            dis = self.getMazeDistance(position, border)  # 我到边界的距离
            collection.push((border), dis)
        if collection.count == 0:  # 找不到
            return [self.start]  # 复活点
        else:
            return [collection.pop()]  # 返回最近的边界的点

    def gameTheoryCalculation1(self, gameState, pos1, pos2):
        curOb = self.getCurrentObservation()
        selfState = curOb.getAgentState(self.index)
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]#敌方幽灵
        matrix = [[0,0],[0,0]]
        if len(defenders) > 0:
            defendersPos = [i.getPosition() for i in defenders]#对面所有幽灵的位置
            matrix[0][0] = self.getMazeDistance(selfState.getPosition(), pos1)
            matrix[1][0] = self.getMazeDistance(selfState.getPosition(), pos2)
            matrix[0][1] = min([self.getMazeDistance(pos1, defender.getPosition()) for defender in defenders])
            matrix[1][1] = min([self.getMazeDistance(pos2, defender.getPosition()) for defender in defenders])
        for i in range(0,2):
            matrix[i][0] = matrix[i][0] - matrix[i][1]
        if matrix[0][0] < matrix[1][0]:
            return pos1
        else:
            return pos2





    def gameTheoryCalculation(self, gameState):
        curOb = self.getCurrentObservation()
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
        matrix = []
        if len(defenders) > 0:#对面有幽灵
            defendersPos = [i.getPosition() for i in defenders]#对面所有幽灵的位置
            for i, selfBorderPoint in enumerate(self.selfBorder):#我方边界上每一个点，i是序号
                row = []
                for j, enemyBoderPoint in enumerate(self.enermyBorder):
                    selfBorderClosestFood = \
                    self.getNiceClosestFood(gameState, defendFood=False, num=1, customPosition=selfBorderPoint)[0]#获得离我最近的1个食物的坐标
                    selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)#算这个食物到我家此边界的距离
                    enemyValue = self.getMazeDistance(enemyBoderPoint, selfBorderPoint)#算当前地方边界的点到我方边界的点的距离
                    row.append((-selfValue + enemyValue, -enemyValue))
                matrix.append(row)
        maxIndex = -1
        maxValue = -99999
        for i, value in enumerate(matrix):
            rowSum = 0
            for rowVal in matrix[i]:
                rowSum = rowSum + rowVal[0]
            if (rowSum > maxValue):
                maxValue = rowSum
                maxIndex = i
        if (maxIndex == -1):
            return None
        return self.selfBorder[maxIndex]

    def gameTheoryCalCurrentDefender(self, gameState):
        print('into gameTheoryCalCurrentDefender')
        curOb = self.getCurrentObservation()
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
        row = []
        defendersPos = [i.getPosition() for i in defenders]#敌方幽灵位置
        distribution = util.Counter()
        for i, selfBorderPoint in enumerate(self.selfBorder):#我方边界
            selfBorderClosestFood = \
            self.getNiceClosestFood(gameState, defendFood=False, num=1, customPosition=selfBorderPoint)[0]
            selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)#边界到最近食物的距离
            rowValue = 0
            if len(defenders) > 0:
                enemyValueList = [self.getMazeDistance(ele, selfBorderPoint) for ele in defendersPos]#地方幽灵到我方边界的距离
                print('enemyValueList:')
                #print(enemyValueList)
                rowValue = -selfValue + min(enemyValueList)
            else:
                rowValue = -selfValue
            row.append(rowValue)
            distribution[selfBorderPoint] = rowValue
        #   self.displayDistributionsOverPositions(distribution)
        # print(row)
        maxIndex = row.index(max(row))
        return self.selfBorder[maxIndex]


class OffensiveReflexAgent(ReflexCaptureAgent):

    def chooseAction(self, gameState):
        curOb = self.getCurrentObservation()
        selfState = curOb.getAgentState(self.index)
        teammateState = curOb.getAgentState(self.teammateIndex)
        selfPos = curOb.getAgentPosition(self.index)
        teammatePos = curOb.getAgentPosition(self.teammateIndex)
        # initializing with random border
        # if selfPos == self.start:
        #     ranBorder = random.choice(self.selfBorder)
        #     self.forceGoPoint = random.choice([None, ranBorder])
        # force go to a point
        if (self.forceGoPoint != None):#有必须去的点
            print('Force go to*********' + str(self.forceGoPoint))
            position = curOb.getAgentPosition(self.index)
            if (position == self.forceGoPoint):
                self.forceGoPoint = None
            else:
                return self.heuristicSearch([self.forceGoPoint])

        print "socre"
        print self.getScore(gameState)

        if self.getScore(gameState) > 5:
            print "!!!!!!!!!!!!!!!!!!"
            return self.heuristicSearch(self.getClosestBorder(gameState))
        # Recognize the state that it is stuck
        # if two of my ghosts are at border and are too close and
        # there is at least one very close ghost, force go somewhere(Game Theory).
        # Use random first. ran = random.choice([0, len(selfBorder)-1])
        # print(self.getTeam(gameState))

        # 没有必须去的点
        if (True):  # selfPos in self.selfBorder and teammatePos in self.selfBorder
            dis = self.getMazeDistance(selfPos, teammatePos)#我和队友的距离小于3，会触发博弈论
            if dis <= 3:
                enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
                enemiesAtHome = [ele for ele in enemies if
                                 not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
                if len(enemiesAtHome) > 0:
                    defendersPos = [i.getPosition() for i in enemiesAtHome]
                    for pos in defendersPos:
                        distance = self.getMazeDistance(pos, selfState.getPosition()) - 2
                        if distance <= 5:
                            # print('********into set forceGoPoint')
                            # self.forceGoPoint = random.choice(self.selfBorder)
                            # self.forceGoPoint = self.gameTheoryCalCurrentDefender(gameState)
                            self.forceGoPoint = random.choice(
                                [#self.gameTheoryCalculation(gameState),
                                 self.gameTheoryCalCurrentDefender(gameState),
                                 #random.choice(self.selfBorder),
                                 #random.choice(self.selfBorder)
                                 ])

                            return self.heuristicSearch([self.forceGoPoint])

        foodList = self.getFood(gameState).asList()
        foodAte = self.foodNum - len(foodList)

        #print foodAte

        # if capsule is near, eat capsule
        capsules = self.getCapsules(gameState)#找对面的药(list)
        for capsule in capsules:
            print capsule
            print "score"
            print self.getScore(gameState)
            distance = self.getMazeDistance(capsule, selfState.getPosition())
            foodpos = self.getNiceClosestFood(gameState)[0]
            print foodpos
            if (distance <= 5):#我和药的距离小于3
                #return self.heuristicSearch([capsule])#找药吃
                print "finally:",self.gameTheoryCalculation1(gameState, foodpos, capsule)
                return self.heuristicSearch([self.gameTheoryCalculation1(gameState, foodpos, capsule)])
        print "skip"
            # return self.heuristicSearch(self.getClosestBorder(gameState))
        if not selfState.isPacman:
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if
                             not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
            if len(enemiesAtHome) > 0:#有幽灵在家里
                defendersPos = [i.getPosition() for i in enemiesAtHome]
                for ele in enemiesAtHome:
                    distanceene2border = self.getMazeDistance(ele.getPosition(), self.getClosestBorder(ele)[0]) - 2
                    distanceme2border = self.getMazeDistance(selfState.getPosition(), self.getClosestBorder(gameState)[0]) - 2
                    distance = self.getMazeDistance(ele.getPosition(), selfState.getPosition()) - 2
                    if distance <= 6 and distanceene2border <= 2 and distanceme2border <= 2:#和对面幽灵距离小于5
                        return self.heuristicSearch(self.getClosestBorder5(gameState))

        # Go back to home
        if len(foodList) <= 2:
            return self.heuristicSearch(self.getClosestBorder(gameState))#全场只剩2个食物

        # already escape
        if self.isChased == True and not selfState.isPacman:#在被追，不是吃豆人
            self.isChased = False
            return self.heuristicSearch(self.getClosestBorder(gameState))

        # avoid defenders
        if selfState.isPacman:#我是吃豆人
            # get defenders position
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if
                             not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
            if len(enemiesAtHome) > 0:#有敌人在家里
                defendersPos = [i.getPosition() for i in enemiesAtHome]

                for pos in defendersPos:
                    distance = self.getMazeDistance(pos, selfState.getPosition()) - 2
                    if distance <= 2:#和对面幽灵距离小于2
                        self.isChased = True
                        return self.heuristicSearch(self.getClosestBorder(gameState))

        # # create a function for better go home
        # border = self.getClosestBorder(gameState)
        # distance = self.getMazeDistance(border[0],selfState.getPosition())
        # if distance>0 and (foodAte*2)/distance >= 1 and foodAte!=0 and selfState.isPacman:
        #     print('Go home!!!')
        #     # if food is near, eat food
        #     closestFood = self.getNiceClosestFood(gameState, defendFood=False, num=1)
        #     distance = self.getMazeDistance(closestFood[0],selfState.getPosition())
        #     # print('distance to the closest food:'+str(distance))
        #     if(distance == 1):
        #         return self.heuristicSearch(closestFood)
        #     return self.heuristicSearch(border)
        # getCapsules

        if selfState.isPacman and curOb.getAgentState(#我和队友都是吃豆人，我们距离小于1
                self.teammateIndex).isPacman and self.isChased == False and self.getMazeDistance(selfPos,
                                                                                                 teammatePos) <= 1 and selfState.numCarrying > teammateState.numCarrying:
            self.forceGoPoint = self.getClosestBorder(gameState)[0]
            return self.heuristicSearch([self.forceGoPoint])

        if not selfState.isPacman:#我不是吃豆人，在家里
            self.foodNum = len(self.getFood(gameState).asList())#对面的食物数量

        return self.heuristicSearch(self.getGoals(gameState, False))#变成进攻者，getNiceClosestFood，找最近食物的坐标


# class DefensiveReflexAgent(ReflexCaptureAgent):
#  def chooseAction(self, gameState):
#    #   return 'Stop'
#      return self.heuristicSearch(self.getGoals(gameState,True))

class DefensiveAgent(CaptureAgent):

    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.nextPosition = None
        self.PreviousFoods = None
        self.EastenFlag = False
        self.width = gameState.data.layout.width
        self.height = gameState.data.layout.height
        self.probability = {}  # (position, prob)
        self.coordinatesWithoutWall = []

        middleLine = self.width / 2

        if self.red:
            x = middleLine
        else:
            x = middleLine + 1

        for i in range(0, self.height):
            coordinate = (x, i)
            if not gameState.hasWall(x, i):
                self.coordinatesWithoutWall.append(coordinate)

        # get initial probability
        sum = 0.0
        for position in self.coordinatesWithoutWall:
            closestDis = 9999
            for food in self.getFoodYouAreDefending(gameState).asList():
                distance = self.getMazeDistance(position, food)
                if distance < closestDis:
                    if distance != 0:
                        closestDis = distance
                    else:
                        closestDis = 1;
            self.probability[position] = 1.0 / float(closestDis)
            sum += self.probability[position]

        if sum == 0:
            sum = 1

        for position in self.probability.keys():
            self.probability[position] = float(self.probability[position]) / float(sum)

    def chooseAction(self, gameState):

        myPosition = gameState.getAgentPosition(self.index)
        actions = gameState.getLegalActions(self.index)
        actions.remove('Stop')
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        enemies_State = []
        for enemy in enemies:
            if enemy.isPacman and enemy.getPosition() != None:
                enemies_State.append(enemy)

        if myPosition == self.nextPosition:
            self.nextPosition = None

        if self.PreviousFoods and len(self.PreviousFoods) != len(self.getFoodYouAreDefending(gameState).asList()):
            sum = 0.0
            for position in self.coordinatesWithoutWall:
                closestDis = 9999
                for food in self.getFoodYouAreDefending(gameState).asList():
                    distance = self.getMazeDistance(position, food)
                    if distance < closestDis:
                        if distance != 0:
                            closestDis = distance
                        else:
                            closestDis = 1;
                self.probability[position] = 1.0 / float(closestDis)
                sum += self.probability[position]

            if sum == 0:
                sum = 1

            for position in self.probability.keys():
                self.probability[position] = float(self.probability[position]) / float(sum)

        if len(enemies_State) != 0:
            enemies_position = [enemy.getPosition() for enemy in enemies_State]
            closeDis = 9999
            for position in enemies_position:
                Dis = self.getMazeDistance(myPosition, position)
                if Dis < closeDis:
                    closeDis = Dis
                    self.nextPosition = position

        if len(enemies_State) == 0 and self.PreviousFoods != None:
            eastenFood = set(self.PreviousFoods) - set(self.getFoodYouAreDefending(gameState).asList())
            if len(eastenFood) != 0:
                self.nextPosition = eastenFood.pop()
                self.EastenFlag = True

        self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()

        if self.nextPosition == None:
            rand = random.random()
            sum = 0.0
            for x in self.probability.keys():
                sum += self.probability[x]
                if rand < sum:
                    self.nextPosition = x

        values = []
        candidateActions = []
        for a in actions:
            successor = gameState.generateSuccessor(self.index, a)
            if not successor.getAgentState(self.index).isPacman:
                if self.EastenFlag == True:  # if there is a food point has been easten
                    preState = self.getPreviousObservation()

                    if preState.getAgentPosition(self.index) != successor.getAgentPosition(self.index):
                        candidateActions.append(a)
                        values.append(self.getMazeDistance(successor.getAgentPosition(self.index), self.nextPosition))
                    else:
                        reverseOperation = a  # if stuck in a deadend, we have to reverse

                if self.EastenFlag == False:  # if there are not any food point has been easten
                    values.append(self.getMazeDistance(successor.getAgentPosition(self.index), self.nextPosition))
                    candidateActions.append(a)

        if len(candidateActions) == 0:
            candidateActions.append(reverseOperation)
            values.append("0")

        minValue = min(values)
        #     bestAction = [a for v, ca in zip(values, candidateActions) if a[0] == minValue]
        bestAction = filter(lambda x: x[0] == minValue, zip(values, candidateActions))
        choiceaction = random.choice(bestAction)[1]

        return choiceaction

    def getSuccessor(self, gameState, action):

        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            return successor.generateSuccessor(self.index, action)
        else:
            return successor