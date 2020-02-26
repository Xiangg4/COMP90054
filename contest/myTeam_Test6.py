# junTeam.py song2_upgrade
# -*- coding:utf8 -*-
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

def createTeam(firstIndex, secondIndex, isRed,#DefensiveReflexAgent OffensiveReflexAgent
               first = 'OffensiveReflexAgent', second = 'DefensiveAgent'):
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
    self.start = gameState.getAgentPosition(self.index)#我的坐标
    CaptureAgent.registerInitialState(self, gameState)
    self.foodNum = len(self.getFood(gameState).asList())#豆子数目
    self.walls = gameState.getWalls().asList()#所有的墙
    self.selfBorder,self.enermyBorder = self.getBoundaries(gameState)#我方边界敌方边界
    self.isChased = False
    self.forceGoPoint = None
    self.myTeamIndexes = self.getTeam(gameState)#我方队员的序号
    self.teammateIndex = 0
    # print('Whole Team!!!!!!')
    # print(self.myTeamIndexes)
    for i in range(len(self.myTeamIndexes)):
        if(self.index != self.myTeamIndexes[i]):
            self.teammateIndex = self.myTeamIndexes[i]#取得第一个队员序号
    # print('Teamate:')
    # print(self.teammateIndex)


  # TODO: Calculate the boundaries using the first 15 seconds
  def getBoundaries(self, gameState):#算我方的边界和敌方边界，不含城墙
    grid = gameState.getWalls()
    width = grid.width
    height = grid.height
    selfSide = self.start[0]
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
        if(not grid[selfBorderX][i] and not grid[enermyBorderX][i]):
            selfBorder.append((selfBorderX, i))
            enermyBorder.append((enermyBorderX, i))
    # print('*******************Self Border')
    # print(selfBorder)
    # print('*******************Enermy Border')
    # print(enermyBorder)

    # print('width:')
    # print(grid.width)
    # print('height:')
    # print(grid.height)
    return selfBorder,enermyBorder

  def getSuccessors(self, position, isChased):
      # TODO: Add the deadend configuration
      blockers = copy.copy(self.walls)#障碍，所有的墙
    #   if(isChased):
    #       blockers.extend(self.deadEndList)
      curOb = self.getCurrentObservation();
      if curOb.getAgentState(self.index).isPacman:#如果我已经进入敌方变成吃豆人
          enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
          defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
          if len(defenders) > 0:#对方幽灵至少一个
              defendersPos = [i.getPosition() for i in defenders]
              blockers.extend(defendersPos)#把对方幽灵位置算进障碍里
      if not curOb.getAgentState(self.index).isPacman:#如果我还在家里，是幽灵
          enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
          defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
          if len(defenders) > 0:#如果敌方幽灵在家
              defendersPos = [i.getPosition() for i in defenders]
              expandList = copy.copy(defendersPos)#敌方幽灵位置
              for element in defendersPos:
                  eBlock = (element[0]-1, element[1])#敌方幽灵位置的东西南北8个相邻点
                  wBlock = (element[0]+1, element[1])
                  sBlock = (element[0], element[1]-1)
                  nBlock = (element[0], element[1]+1)
                  neBlock = (element[0]+1, element[1]+1)
                  nwBlock = (element[0]-1, element[1]-1)
                  seBlock = (element[0]+1, element[1]-1)
                  swBlock = (element[0]-1, element[1]+1)
                  fourDirectionList = [eBlock,wBlock,sBlock,nBlock,neBlock,nwBlock,seBlock,swBlock]
                  for b in fourDirectionList:
                      if b not in self.selfBorder:#如果敌方幽灵不贴近我方边界，就把这些相邻点也算进障碍里
                          expandList.append(b)

              blockers.extend(expandList)

      successors = []
      for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x,y = position
          dx, dy = Actions.directionToVector(action)
          nextx, nexty = int(x + dx), int(y + dy)
          if (nextx, nexty) not in blockers:#如果下个方向不是障碍
              nextState = (nextx, nexty)
              successors.append((nextState, action))
      return successors

  def heuristicSearch(self, goalList):
        position = self.getCurrentObservation().getAgentPosition(self.index)#我的位置
        def heuristic(p1,p2):#曼哈顿距离
            return util.manhattanDistance(p1, p2)
        for point in goalList:
            collection = util.PriorityQueue()
            visited = []
            dis = heuristic(position, point)#每一个我到目标点的曼哈顿距离都进队
            collection.push((position, []), dis)
            while not collection.isEmpty():
                tmpPoint, path = collection.pop()#距离最近的点出队
                if(tmpPoint in visited):
                    continue
                visited.append(tmpPoint)
                if(point == tmpPoint):
                    if(len(path)==0):
                        continue
                    return path[0]
                successors = self.getSuccessors(tmpPoint, point)
                for element in successors:
                    fn = len(path + [element[1]]) + heuristic(element[0], point)
                    collection.push((element[0], path+[element[1]]), fn)
        
        if len(goalList)>0 and goalList[0]!=self.start:
            return self.heuristicSearch([self.start])
        # if len(goalList)==0:
        #     return 'North'

        return 'Stop'

  def getNiceClosestFood(self, gameState, defendFood=False, num=-1, customPosition = None):
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
          for i, selfBorderPoint in enumerate(self.selfBorder):
              row = []
              for j, enemyBoderPoint in enumerate(self.enermyBorder):
                  selfBorderClosestFood = self.getNiceClosestFood(gameState,defendFood=False, num=1,customPosition = selfBorderPoint)[0]
                  selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)
                  enemyValue = self.getMazeDistance(enemyBoderPoint, selfBorderPoint)
                  row.append( (-selfValue + enemyValue,-enemyValue) )
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
          selfBorderClosestFood = self.getNiceClosestFood(gameState,defendFood=False, num=1,customPosition = selfBorderPoint)[0]
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
      #print(row)
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
        if(self.forceGoPoint != None):
            print('Force go to*********'+str(self.forceGoPoint))
            position = curOb.getAgentPosition(self.index)
            if(position == self.forceGoPoint):
                self.forceGoPoint = None
            else:
                return self.heuristicSearch([self.forceGoPoint])


        # Recognize the state that it is stuck
        # if two of my ghosts are at border and are too close and
        # there is at least one very close ghost, force go somewhere(Game Theory).
        # Use random first. ran = random.choice([0, len(selfBorder)-1])
        # print(self.getTeam(gameState))

        if(True):#selfPos in self.selfBorder and teammatePos in self.selfBorder
            dis = self.getMazeDistance(selfPos,teammatePos)
            if dis <= 3:
                enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
                enemiesAtHome = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
                if len(enemiesAtHome) > 0:
                    defendersPos = [i.getPosition() for i in enemiesAtHome]
                    for pos in defendersPos:
                        distance = self.getMazeDistance(pos,selfState.getPosition()) - 2
                        if distance <= 2:
                            # print('********into set forceGoPoint')
                            # self.forceGoPoint = random.choice(self.selfBorder)
                            # self.forceGoPoint = self.gameTheoryCalCurrentDefender(gameState)
                            self.forceGoPoint = random.choice(
                            [self.gameTheoryCalculation(gameState),
                            self.gameTheoryCalCurrentDefender(gameState),
                            random.choice(self.selfBorder),
                            random.choice(self.selfBorder)
                            ])

                            return self.heuristicSearch([self.forceGoPoint])



        foodList = self.getFood(gameState).asList()
        foodAte = self.foodNum - len(foodList)
        print foodAte

        # if capsule is near, eat capsule
        capsules = self.getCapsules(gameState)
        for capsule in capsules:
            # print(capsule)
            distance = self.getMazeDistance(capsule,selfState.getPosition())
            if(distance <= 3):
                return self.heuristicSearch([capsule])
            # return self.heuristicSearch(self.getClosestBorder(gameState))




        #Go back to home
        if len(foodList) <= 2:
            return self.heuristicSearch(self.getClosestBorder(gameState))

        #already escape
        if self.isChased == True and not selfState.isPacman:
            self.isChased = False
            return self.heuristicSearch(self.getClosestBorder(gameState))



        #avoid defenders
        if selfState.isPacman:
            # get defenders position
            enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
            enemiesAtHome = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
            if len(enemiesAtHome) > 0:
                defendersPos = [i.getPosition() for i in enemiesAtHome]

                for pos in defendersPos:
                    distance = self.getMazeDistance(pos,selfState.getPosition()) - 2
                    if distance <= 2:
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

        if selfState.isPacman and curOb.getAgentState(self.teammateIndex).isPacman and self.isChased==False and self.getMazeDistance(selfPos,teammatePos)<=1 and selfState.numCarrying>teammateState.numCarrying:
            self.forceGoPoint = self.getClosestBorder(gameState)[0]
            return self.heuristicSearch([self.forceGoPoint])

        if not selfState.isPacman:
            self.foodNum = len(self.getFood(gameState).asList())

        return self.heuristicSearch(self.getGoals(gameState,False))

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