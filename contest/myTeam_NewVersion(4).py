#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from captureAgents import CaptureAgent
import random, util
from util import nearestPoint
import copy

#################
# Team creation #
#################
#python capture.py -r myTeam_NewVersion(3) -b baselineTeam -l RANDOM14 打不过baseline
#python capture.py -r myTeam_NewVersion(3) -b baselineTeam -l RANDOM16 走了不该走的路
#python capture.py -r myTeam_NewVersion(3) -b baselineTeam -l RANDOM18 force to go
#python capture.py -r myTeam_NewVersion(3) -b baselineTeam -l RANDOM8 avoiding this ghost(白色的)




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



def manhattanHeuristic(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


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
    #self.selfBorder, self.enermyBorder = getBoundaries(self.start[0],gameState.getWalls())

    # whether our pacman is being chasen
    self.goBack = False
    # compulsive move
    self.mustTo = None
    # is super pacman
    self.isSuper = False
    #self.myTeamIndexes = self.getTeam(gameState)
    #
    self.teammate = [i for i in self.getTeam(gameState) if i != self.index][0]
    ############################
    self.mapWidth = gameState.getWalls().width
#    print self.mapWidth
    self.mapHeight = gameState.getWalls().height
#    print self.mapHeight    
    self.capsulesPosition = self.getCapsules(gameState)
#    position of all capsules, used to identify isSuper
    
    middle = self.mapWidth / 2 - 1
    self.ourDoor = []
    if self.red:
      for i in range(self.mapHeight):
        if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle+1][i]:
          self.ourDoor.append((middle,i))
    else:
      for i in range(self.mapHeight):
        if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle+1][i]:
          self.ourDoor.append((middle+1,i))

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


  '''
  get the set of points that our pacman should not head for
  '''
  def getAvoided(self):
    gameState = self.getCurrentObservation().getAgentState(self.index)
    avoided = copy.copy(self.walls) 

#          print ghost.getPosition()
    return avoided

  '''
  generate successors
  '''
  def getSuccessor(self, position):
      # TODO: Add the deadend configuration      
      avoided = self.getAvoided()
      successors = []
      direction = ['North','East','South','West']
      offset = [(0,1),(1,0),(0,-1),(-1,0)]
      for action in zip(direction,offset):
          point = ((position[0] + action[1][0]), (position[1] + action[1][1]))
          if point not in avoided:
              successors.append((point,action[0]))
      return successors

  
  '''
  get the closest food based on given position
  '''
  def getClosestFood(self, gameState, position):
      print "into getClosestFood"
      theirPacman = []
      enemies = []
      myState = self.getCurrentObservation().getAgentState(self.index)
      for i in self.getOpponents(self.getCurrentObservation()):
         enemies.append(self.getCurrentObservation().getAgentState(i))
      for enemy in enemies:
          if enemy.isPacman and enemy.getPosition() != None:
              theirPacman.append(enemy)
      if (not myState.isPacman) and myState.scaredTimer <= 0:  # 只有在家而且没失去战力的时候才会考虑敌人
          if len(theirPacman)>0:
              distanceToEnemy_min = 99999
              closestEnemy = None
              for pac in theirPacman:
                  dis = self.getMazeDistance(position,pac.getPosition())
                  if dis < distanceToEnemy_min:
                      distanceToEnemy_min = dis
                      closestEnemy = pac
              return closestEnemy.getPosition()
              
      foods = self.getFood(gameState).asList()
      distanceToFood_min = 9999
      closestFood = None
      for food in foods:
          dis = self.getMazeDistance(position, food)
          if dis < distanceToFood_min:
              distanceToFood_min = dis
              closestFood = food
      return closestFood


  '''
  get the food by order of distance to given position
  '''
  def getFoodByOrder(self,gameState,position):
      print "into getFoodByOrder"
      theirPacman = []
      enemies = []
      myState = self.getCurrentObservation().getAgentState(self.index)
      for i in self.getOpponents(self.getCurrentObservation()):
         enemies.append(self.getCurrentObservation().getAgentState(i))
      for enemy in enemies:
          if enemy.isPacman and enemy.getPosition() != None:
              theirPacman.append(enemy)
      if (not myState.isPacman) and myState.scaredTimer <= 0:#只有在家而且没失去战力的时候才会考虑敌人
          if len(theirPacman)>0:
              distanceToEnemy_min = 99999
              closestEnemy = None
              for pac in theirPacman:
                  dis = self.getMazeDistance(position, pac.getPosition())
                  if dis < distanceToEnemy_min:
                      distanceToEnemy_min = dis
                      closestEnemy = pac
              return [closestEnemy.getPosition()]
              
      foods = self.getFood(gameState).asList()
      
      def myCmp(x,y):
          return self.getMazeDistance(position,x) - self.getMazeDistance(position,y)
      foods.sort(cmp = myCmp)
      return foods


  def getClosestBorder(self, gameState):
      position = self.getCurrentObservation().getAgentPosition(self.index)
      collection = util.PriorityQueue()
      for border in self.ourDoor:
          dis = self.getMazeDistance(position, border)
          collection.push((border), dis)
      if collection.count==0:
          return [self.start]
      else:
          return [collection.pop()]


  def gameTheoryCalCurrentDefender(self, gameState):
      print('into gameTheoryCalCurrentDefender')
      curOb = self.getCurrentObservation()
      enemies = [curOb.getAgentState(i) for i in self.getOpponents(curOb)]
      defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
      row = []
      defendersPos = [i.getPosition() for i in defenders]
      distribution = util.Counter()
      for i, selfBorderPoint in enumerate(self.ourDoor):
          selfBorderClosestFood = self.getClosestFood(gameState, selfBorderPoint)
          selfValue = self.getMazeDistance(selfBorderClosestFood, selfBorderPoint)
          rowValue = 0
          if len(defenders) > 0:
              enemyValueList = [self.getMazeDistance(ele,selfBorderPoint) for ele in defendersPos]
#              print('enemyValueList:')
#              print(enemyValueList)
              rowValue = -selfValue + min(enemyValueList)
          else:
              rowValue = -selfValue
          row.append(rowValue)
          distribution[selfBorderPoint] = rowValue
    #   self.displayDistributionsOverPositions(distribution)
#      print(row)
      maxIndex = row.index(max(row))
      return self.ourDoor[maxIndex]
  
  ########################game calculation#######################
  
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
        # else:
        #     if self.isSuper == False:#？？？？？？？？？？？？
        #        return pos2
        #     elif self.isSuper == True:
        #        return pos1

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
        Observation = self.getCurrentObservation()
        selfState = Observation.getAgentState(self.index)
#        print foodAte
#        print "my pos:"
#        print selfState.getPosition()
#        print "cap:"
#        print capsules
        #self.mustTo = None
        self.mustTo = (gameState.data.layout.width/2,9)
        print gameState.data.layout.width

        return self.aStarSearch([self.mustTo])



    
####################
# Defensive Agents #
####################
        
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
          
      return 'Stop'


  def getSuccessor(self, gameState, action):
      
      successor = gameState.generateSuccessor(self.index, action)
      pos = successor.getAgentState(self.index).getPosition()
      if pos != nearestPoint(pos):
        return successor.generateSuccessor(self.index, action)
      else:
        return successor
