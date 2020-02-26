#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from captureAgents import CaptureAgent
import random, util
from game import Directions
from util import nearestPoint
from game import Actions
import copy

#################
# Team creation #
#################


class StateModel:
    """
    This class aims to do preprocesses, including creating initial nodes and success nodes,
    calulating the path, and designing a common algorithm which can be invoked by different
    searching algorithms.
    """
    def make_root_node(self,init_position):
        """
        initialise the root node, of which the information is stored in dict
        """
        init_node = {
            "position":init_position,
            "parent":None,
            "action":None,
        }
        return init_node

    def make_succ_node(self,curr_node,succ_position,succ_action):
        """
        add information in successor nodes
        """
        succ_node = {
            "position":succ_position,
            "parent":curr_node,
            "action":succ_action,
        }
        return succ_node

    def make_actions(self,goal_node):
        """
        return a list of actions that reaches the goal.
        """
        actions = []
        while (goal_node["parent"]):
            actions.append(goal_node["action"])
            goal_node = goal_node["parent"]
        actions = actions[::-1]
        return actions

def createTeam(firstIndex, secondIndex, isRed,#DefensiveReflexAgent OffensiveReflexAgent
               first = 'OffensiveReflexAgent', second = 'DefensiveAgent'):

  return [eval(first)(firstIndex), eval(second)(secondIndex)]

####################
# Helper Functions #
####################
  
# TODO: Calculate the boundaries using the first 15 seconds
def getBoundaries(start, walls):
    width = walls.width
    height = walls.height
    selfSide = start
    leftBorder = width/2-1
    rightBorder = width/2
    selfBorder = []
    enermyBorder = []
    selfBorderX = leftBorder
    enermyBorderX = rightBorder
    if(selfSide>rightBorder):
        selfBorderX = rightBorder
        enermyBorderX = leftBorder

    # TODO: Start to calcultate no wall dot
    for i in range(height):
        if(not walls[selfBorderX][i] and not walls[enermyBorderX][i]):
            selfBorder.append((selfBorderX, i))
            enermyBorder.append((enermyBorderX, i))

    return selfBorder, enermyBorder

def manhattanHeuristic(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


#def heuristicSearch(self, goalList):
        # position = self.getCurrentObservation().getAgentPosition(self.index)
                
        # for point in goalList:
        #     collection = util.PriorityQueue()
        #     visited = []
        #     dis = manhattanHeuristic(position, point)
        #     collection.push((position, []), dis)
            
        #     while not collection.isEmpty():
        #         tmpPoint, path = collection.pop()
        #         if(tmpPoint in visited):
        #             continue
                
        #         visited.append(tmpPoint)
                
        #         # 终点判断
        #         if(point == tmpPoint):
        #             if(len(path)==0):
        #                 continue
        #             return path[0]
                
        #         successors = self.getSuccessor(tmpPoint)
                
        #         for element in successors:
        #             # AStar -> fn, path是actions的序列
        #             fn = len(path + [element[1]]) + manhattanHeuristic(element[0], point)
        #             collection.push((element[0], path+[element[1]]), fn)
        
        # # 有目标但是找不到，让它回家避免死循环
        # if len(goalList)>0 and goalList[0]!=self.start:
        #     return heuristicSearch(self,[self.start])
        
        # return 'Stop'


####################
# Offensive Agents #
####################

model = StateModel()

class OffensiveReflexAgent(CaptureAgent):


    
  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.foodNum = len(self.getFood(gameState).asList())
    self.walls = gameState.getWalls().asList()
    self.selfBorder, self.enermyBorder = getBoundaries(self.start[0],gameState.getWalls())
    self.isChased = False
    self.forceGoPoint = None
    self.myTeamIndexes = self.getTeam(gameState)
    self.teammateIndex = 0
    
    for i in range(len(self.myTeamIndexes)):
        if(self.index != self.myTeamIndexes[i]):
            self.teammateIndex = self.myTeamIndexes[i]

  #Astar heuristic search
  def aStarSearch(self,goals):
    init_state = self.getCurrentObservation().getAgentState(self.index).getPosition()
    for goal in goals: 
      init_node  = model.make_root_node(init_state)
      frontier = util.PriorityQueue()
      frontier.push(init_node,0)
      explored_set = []
      while not frontier.isEmpty():
        node = frontier.pop()
        if node["position"] not in explored_set:
          explored_set.append(node["position"])
          if goal == node["position"]:
            if node["action"]==None:
              continue
            return model.make_actions(node)[0]
          for successor in self.getSuccessor(node["position"]):
            #succ_actions = StateModel.make_actions()
            succ_node = model.make_succ_node(node,successor[0],successor[1])
            actions = model.make_actions(succ_node)
            g = len(actions)
            h = manhattanHeuristic(succ_node["position"],goal)
            f = g + h
            frontier.update(succ_node,f)
    return 'Stop'



  def getBlockers(self):
    curOb = self.getCurrentObservation()
    blockers = copy.copy(self.walls) 
    enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
    defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
    if not curOb.getAgentState(self.index).isPacman:
      if len(defenders) > 0:  # 如果敌方幽灵在家
        defendersPos = [i.getPosition() for i in defenders]
        for element in defendersPos:
          for i in range(-1, 2):
            for j in range(-1, 2):
              blockers.append((element[0] + i, element[1] + j))
    else:
      if len(defenders) > 0:
        for defender in defenders:
          blockers.append(defender.getPosition())
    return blockers

  def getSuccessor(self, position):
      # TODO: Add the deadend configuration      
      blockers = self.getBlockers()
      successors = []
      for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x,y = position
          dx, dy = Actions.directionToVector(action)
          nextx, nexty = int(x + dx), int(y + dy)
          if (nextx, nexty) not in blockers:
              nextState = (nextx, nexty)
              successors.append((nextState, action))
      return successors


  #def getCloestFood

  def getNiceClosestFood(self, gameState, defendFood=False, num=2, customPosition = None):
      position = self.getCurrentObservation().getAgentPosition(self.index)
      
      if customPosition != None:
          position = customPosition
          
      if defendFood:
          foodList = self.getFoodYouAreDefending(gameState).asList()
      else:
          curOb = self.getCurrentObservation();
          position = curOb.getAgentPosition(self.index)
          enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
          enemiesAtHome = [ele for ele in enemies if ele.isPacman and ele.getPosition() != None]
          # 有重复计算，可以修改
          if len(enemiesAtHome) > 0:
              dists = [self.getMazeDistance(position, ele.getPosition()) for ele in enemiesAtHome]
              for i in range(0, len(enemiesAtHome)):
                  dis2Enemy = self.getMazeDistance(position, enemiesAtHome[i].getPosition())
                  if dis2Enemy == min(dists) and dis2Enemy<=5:
                      return [enemiesAtHome[i].getPosition()]
          
          foodList = self.getFood(gameState).asList()
          
      collection = util.PriorityQueue()
      for food in foodList:
          dis = self.getMazeDistance(position, food)
          collection.push((food), dis)
      
      result = []
      # num控制我理想的食物的数量
      if(num<0):
          #for i in range(collection.count/4):
          for i in range(collection.count):
              result.append(collection.pop())
      else:
          if(collection.count<num):
              num = collection.count
          for i in range(num):
              result.append(collection.pop())
    #   print(len(result))
      return result

  # 家里有敌人吃人，家里没敌人巡逻，这个方法是不是可以去掉？？
  def defenderBestPosition(self, gameState):
      curOb = self.getCurrentObservation();
      position = curOb.getAgentPosition(self.index)
      enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
      enemiesAtHome = [ele for ele in enemies if ele.isPacman and ele.getPosition() != None]
      if len(enemiesAtHome) > 0:
          dists = [self.getMazeDistance(position, ele.getPosition()) for ele in enemiesAtHome]
          for i in range(0, len(enemiesAtHome)):
              if self.getMazeDistance(position, enemiesAtHome[i].getPosition()) == min(dists):
                  return [enemiesAtHome[i].getPosition()]
      else:
          defFoodList = self.getNiceClosestFood(gameState, defendFood=True, num=3)
          ranPoint = random.choice(defFoodList)
          if position ==  ranPoint:
              defFoodList.remove(ranPoint)
              ranPoint = random.choice(defFoodList)
          return [ranPoint]


  def getGoals(self, gameState, isDefender):

      if not isDefender:
          return self.getNiceClosestFood(gameState)
      else:
          return self.defenderBestPosition(gameState)#defenderBestPosition


  def getClosestBorder(self, gameState):
      position = self.getCurrentObservation().getAgentPosition(self.index)
      collection = util.PriorityQueue()
      for border in self.selfBorder:
          dis = self.getMazeDistance(position, border)
          collection.push((border), dis)
      if collection.count==0:
          return [self.start]
      else:
          return [collection.pop()]

  def gameTheoryCalculation(self, gameState):
      curOb = self.getCurrentObservation()
      enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
      defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
      matrix = []
      
      if len(defenders) > 0:
          defendersPos = [i.getPosition() for i in defenders]
          # i = 枚举index; selfBorderPoint = 坐标
          for i, selfBorderPoint in enumerate(self.selfBorder):
              row = []
              for j, enemyBoderPoint in enumerate(self.enermyBorder):
                  selfBorderClosestFood = self.getNiceClosestFood(gameState,defendFood=False, num=1,customPosition = selfBorderPoint)[0]
                  selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)
                  enemyValue = self.getMazeDistance(enemyBoderPoint, selfBorderPoint)
                  row.append( (-selfValue + enemyValue, -enemyValue) )
              matrix.append(row)
      
      maxIndex = -1
      maxValue = -99999
      for i,value in enumerate(matrix):
          rowSum = 0
          for rowVal in matrix[i]:
              rowSum = rowSum + rowVal[0]
          if(rowSum > maxValue):
              maxValue = rowSum
              maxIndex = i
      if(maxIndex == -1):
          return None
      return self.selfBorder[maxIndex]


  def gameTheoryCalCurrentDefender(self, gameState):
      print('into gameTheoryCalCurrentDefender')
      curOb = self.getCurrentObservation()
      enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
      defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
      row = []
      defendersPos = [i.getPosition() for i in defenders]
      distribution = util.Counter()
      for i, selfBorderPoint in enumerate(self.selfBorder):
          selfBorderClosestFood = self.getNiceClosestFood(gameState,defendFood=False, num=1, customPosition = selfBorderPoint)[0]
          selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)
          rowValue = 0
          if len(defenders) > 0:
              enemyValueList = [self.getMazeDistance(ele,selfBorderPoint) for ele in defendersPos]
              print('enemyValueList:')
              print(enemyValueList)
              rowValue = -selfValue + min(enemyValueList)
          else:
              rowValue = -selfValue
          row.append(rowValue)
          distribution[selfBorderPoint] = rowValue
    #   self.displayDistributionsOverPositions(distribution)
      print(row)
      maxIndex = row.index(max(row))
      return self.selfBorder[maxIndex]
  def gameTheoryCalculation1(self, pos1, pos2):
        curOb = self.getCurrentObservation()
        selfState = curOb.getAgentState(self.index)
        enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
        defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]#敌方幽灵
        matrix = [[0,0],[0,0]]
        matrix[0][0] = self.getMazeDistance(selfState.getPosition(), pos1)
        matrix[1][0] = self.getMazeDistance(selfState.getPosition(), pos2)
        if len(defenders) > 0:
            #defendersPos = [i.getPosition() for i in defenders]#对面所有幽灵的位置

            matrix[0][1] = min([self.getMazeDistance(pos1, defender.getPosition()) for defender in defenders])
            matrix[1][1] = min([self.getMazeDistance(pos2, defender.getPosition()) for defender in defenders])
        for i in range(0,2):
            matrix[i][0] = matrix[i][0] - matrix[i][1]
        if matrix[0][0] < matrix[1][0]:
            return pos1
        else:
            return pos2

  def gameTheoryCalculation2(self, pos):
      curOb = self.getCurrentObservation()
      selfState = curOb.getAgentState(self.index)
      enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
      defenders = [ele for ele in enemies if
                   not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]  # 敌方幽灵
      matrix = [[0 for i in range(2)] for i in range(len(pos))]
      #matrix = [[0, 0], [0, 0]]
      for i in range(0, len(pos)):
          matrix[i][0] = self.getMazeDistance(selfState.getPosition(), pos[i])
      if len(defenders) > 0:
         # defendersPos = [i.getPosition() for i in defenders]  # 对面所有幽灵的位置

         for i in range(0, len(pos)):
             matrix[i][1] = min([self.getMazeDistance(pos[i], defender.getPosition()) for defender in defenders])
          #matrix[1][1] = min([self.getMazeDistance(pos2, defender.getPosition()) for defender in defenders])
      for i in range(0, len(pos)):
          matrix[i][0] = matrix[i][0] - matrix[i][1]
      minvalue = matrix[0][0]
      bestpoint = pos[0]
      for i in range(0, len(pos)):
          if matrix[i][0] < minvalue:
              bestpoint = pos[i]
      return bestpoint

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
        if(self.forceGoPoint != None):
            print('Force go to*********'+str(self.forceGoPoint))
            position = curOb.getAgentPosition(self.index)
            if(position == self.forceGoPoint):
                self.forceGoPoint = None
            else:
                return self.aStarSearch([self.forceGoPoint])


        # Recognize the state that it is stuck
        # if two of my ghosts are at border and are too close and
        # there is at least one very close ghost, force go somewhere(Game Theory).
        # Use random first. ran = random.choice([0, len(selfBorder)-1])
        # print(self.getTeam(gameState))

#         if(True): #selfPos in self.selfBorder and teammatePos in self.selfBorder
#             dis = self.getMazeDistance(selfPos,teammatePos)
#             if dis <= 3:
#                 enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
#                 enemiesAtHome = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
#                 if len(enemiesAtHome) > 0:
#                     defendersPos = [i.getPosition() for i in enemiesAtHome]
#                     for pos in defendersPos:
#                         distance = self.getMazeDistance(pos,selfState.getPosition()) - 2
#                         if distance <= 2:
#                             # print('********into set forceGoPoint')
#                             # self.forceGoPoint = random.choice(self.selfBorder)
#                             self.forceGoPoint = self.gameTheoryCalCurrentDefender(gameState)
# #                            self.forceGoPoint = random.choice(
# #                            [self.gameTheoryCalculation(gameState),
# #                            self.gameTheoryCalCurrentDefender(gameState),
# #                            random.choice(self.selfBorder),
# #                            random.choice(self.selfBorder)
# #                            ])
#
#                             return self.aStarSearch([self.forceGoPoint])



        foodList = self.getFood(gameState).asList()
        foodAte = self.foodNum - len(foodList)
        print foodAte
        if not selfState.isPacman:
            self.foodNum = len(self.getFood(gameState).asList())
        if foodAte > 5:
             return self.aStarSearch(self.getClosestBorder(gameState))

        #avoid defenders


        # if selfState.getPosition() in self.selfBorder:
        #         #     foodAte = 0
        # if capsule is near, eat capsule
        capsules = self.getCapsules(gameState)
        # 可以取消循环
        # for capsule in capsules:
        #     # print(capsule)
        #     distance = self.getMazeDistance(capsule,selfState.getPosition())
        #     if(distance <= 10):
        #         return self.aStarSearch([capsule])
        if not len(capsules) == 0:
            for capsule in capsules:
                enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
                enemiesAtHome = [ele for ele in enemies if
                                 not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
                print "cap",capsule
                distance = self.getMazeDistance(capsule, selfState.getPosition())
                foodpos = self.getNiceClosestFood(gameState)[0]
                print foodpos

                if not len(enemiesAtHome) > 0:#看不到敌人幽灵时，过去尝试，看到了再判断行不行
                    return self.aStarSearch([capsule])
                if len(enemiesAtHome) > 0:#有幽灵在家里
                    if distance < min([self.getMazeDistance(ele.getPosition(),capsule) for ele in enemiesAtHome]):
                        print "decision"
                        return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])  # 找药吃
                        #return self.aStarSearch([self.gameTheoryCalculation2([foodpos, capsule])])
                        #return self.aStarSearch([capsule])
        if selfState.isPacman:
            # get defenders position
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
            if len(enemiesAtHome) > 0:
                defendersPos = [i.getPosition() for i in enemiesAtHome]

                for pos in defendersPos:
                    distance = self.getMazeDistance(pos,selfState.getPosition()) - 2
                    if distance <= 3:
                        self.isChased = True
                        return self.aStarSearch(self.getClosestBorder(gameState))
                    # else:
                    #     print "aaaaaaaaaaaaaaaaaaaaaaaaaaa"
                #return self.aStarSearch([self.gameTheoryCalculation1(self.getGoals(gameState, False)[0], self.getGoals(gameState, False)[1])])

        ############################
        if not selfState.isPacman:
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if
                             not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
            if len(enemiesAtHome) > 0:#有幽灵在家里
                defendersPos = [i.getPosition() for i in enemiesAtHome]
                for ele in enemiesAtHome:
                    closestb = self.getClosestBorder(ele)[0]
                    distanceene2border = self.getMazeDistance(ele.getPosition(), closestb) - 2
                    distanceme2border = self.getMazeDistance(selfState.getPosition(), self.getClosestBorder(gameState)[0]) - 2
                    distance = self.getMazeDistance(ele.getPosition(), selfState.getPosition()) - 2
                    if distance <= 5 and distanceene2border <= 3 and distanceme2border <= 3:#和对面幽灵距离小于2
                        self.forceGoPoint = self.gameTheoryCalCurrentDefender(gameState) # 这个地方换成了game theory，上面的部分是高翔的
                        return self.aStarSearch([self.forceGoPoint])

        #Go back to home
        if len(foodList) <= 2:
            return self.aStarSearch(self.getClosestBorder(gameState))

        #already escape
        if self.isChased == True and not selfState.isPacman:
            self.isChased = False
            return self.aStarSearch(self.getClosestBorder(gameState))






#        # 这个可以注释掉，我们的队友只会是防守者
#        if selfState.isPacman and curOb.getAgentState(self.teammateIndex).isPacman and self.isChased==False and self.getMazeDistance(selfPos,teammatePos)<=1 and selfState.numCarrying>teammateState.numCarrying:
#            self.forceGoPoint = self.getClosestBorder(gameState)[0]
#            return heuristicSearch(self, [self.forceGoPoint])
#         if selfState.isPacman:
#             # get defenders position
#             enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
#             enemiesAtHome = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
#             if len(enemiesAtHome) > 0:
#                 return self.aStarSearch([self.gameTheoryCalculation1(self.getGoals(gameState, False)[0],
#                                                                      self.getGoals(gameState, False)[1])])


        return self.aStarSearch(self.getGoals(gameState,False))

#class DefensiveReflexAgent(ReflexCaptureAgent):
#  def chooseAction(self, gameState):
#    #   return 'Stop'
#      return self.heuristicSearch(self.getGoals(gameState,True))
        
class DefensiveAgent(CaptureAgent):

  def registerInitialState(self,gameState):
      CaptureAgent.registerInitialState(self,gameState)
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
          if enemy.isPacman and enemy.getPosition()!=None:
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
          if len(eastenFood)!=0:
              self.nextPosition = eastenFood.pop()
              self.EastenFlag = True
      
      self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()

      if self.nextPosition == None:
          rand = random.random()
          sum = 0.0
          for x in self.probability.keys():
              sum+=self.probability[x]
              if rand < sum:
                  self.nextPosition = x
                      
      values = []   
      candidateActions = []
      for a in actions:
          successor = gameState.generateSuccessor(self.index, a)
          if not successor.getAgentState(self.index).isPacman:
              if self.EastenFlag == True: # if there is a food point has been easten
                 preState = self.getPreviousObservation()
                  
                 if preState.getAgentPosition(self.index) != successor.getAgentPosition(self.index):
                    candidateActions.append(a)
                    values.append(self.getMazeDistance(successor.getAgentPosition(self.index), self.nextPosition))
                 else:
                    reverseOperation = a # if stuck in a deadend, we have to reverse
                
              if self.EastenFlag == False: # if there are not any food point has been easten
                  values.append(self.getMazeDistance(successor.getAgentPosition(self.index), self.nextPosition))
                  candidateActions.append(a)

      if len(candidateActions)== 0:
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