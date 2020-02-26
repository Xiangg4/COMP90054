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
    theirGhosts = []
    for i in self.getOpponents(self.getCurrentObservation()):
      enemy = self.getCurrentObservation().getAgentState(i)
      if (not enemy.isPacman) and enemy.getPosition() and enemy.scaredTimer <= 0:
        theirGhosts.append(enemy)
    if not gameState.isPacman:
      if len(theirGhosts) > 0:
        for ghost in theirGhosts:
          for i in range(-1, 2):
            avoided.append((ghost.getPosition()[0] + i, ghost.getPosition()[1] ))
            avoided.append((ghost.getPosition()[0], ghost.getPosition()[1] + i))
    else:
      if len(theirGhosts) > 0:
        for ghost in theirGhosts:
          for i in range(-1, 2):
            avoided.append((ghost.getPosition()[0] + i, ghost.getPosition()[1] ))
            avoided.append((ghost.getPosition()[0], ghost.getPosition()[1] + i))
          print "avoiding this ghost:"
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
                
        "1. must go to this point"           
        if(self.mustTo != None):
            print('Force go to*********'+str(self.mustTo))
            position = Observation.getAgentPosition(self.index)
            if(position == self.mustTo):
                self.mustTo = None
            else:
                return self.aStarSearch([self.mustTo])


        foodList = self.getFood(gameState).asList()
        foodAte = self.foodNum - len(foodList)
        
        "2. calculate the remaining foods"
        if not selfState.isPacman:
            self.foodNum = len(self.getFood(gameState).asList())

        "3. if already ate more than 5 foods, back home to get the points"
        if foodAte > 5:
            return self.aStarSearch(self.getClosestBorder(gameState))
        # 当我吃下了一个药，我现在无敌了，我去瞧瞧有没有幽灵恢复战力，直到发现有幽灵恢复战力，我就不是无敌了，我需要再吃一个药
        "4. if our pacman get a capsule"
        if selfState.getPosition() in self.capsulesPosition:#此时可以认为幽灵都没有战斗力，知道发现了具有战力的幽灵
             self.isSuper = True
             #print "Agent get Capsule!" 
             self.capsulesPosition.remove(selfState.getPosition())

        "5. if our pacman run out of the capsule"
        if self.isSuper:#我现在是无敌状态，看看有没有具有战力的幽灵
            enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
            recoveredGhost = [ele for ele in enemies if
                             not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]
            if len(recoveredGhost) != 0:#被吃掉的幽灵会恢复战斗力，但是还有没被吃的幽灵
               self.isSuper = False#知道有幽灵具有了战斗力，我不是无敌了

        #当我认为有幽灵有战斗力的时候，我寻找最近的药，然后看看我视野内有没有敌人的幽灵，没有就向药靠近，如果看见了敌人幽灵，我就算算谁离得近，我近就去吃，他近我就放弃
        if not self.isSuper:
            capsules = self.getCapsules(gameState)
            if not len(capsules) == 0:  # 找最近的药（RANDOM11)
                dis2closetcap = min([self.getMazeDistance(ele, selfState.getPosition()) for ele in capsules])
                for capsule in capsules:
                    if self.getMazeDistance(capsule, selfState.getPosition()) == dis2closetcap:
                        enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
                        enemiesAtHome = [ele for ele in enemies if
                                         not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]  # 我又改成5了
                        scaredEnemies = [ele for ele in enemies if
                                         not ele.isPacman and ele.getPosition() != None and ele.scaredTimer > 0]
                        #                print "cap",capsule
                        distance = self.getMazeDistance(capsule, selfState.getPosition())
                        foodpos = self.getClosestFood(gameState, selfState.getPosition())
                        if not len(enemiesAtHome) > 0:
                            #return self.aStarSearch([capsule])
                            return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])
                            #python capture.py -r myTeam_NewVersion(3) -b baselineTeam -l RANDOM16 不改会不吃路上东西直奔药，反而什么都吃不到

                        else:
                            if distance < min([self.getMazeDistance(ele.getPosition(), capsule) for ele in enemiesAtHome]):
                                # print "decision"
                                return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])  # 在食物和药之间找安全的吃
                                # return self.aStarSearch([self.gameTheoryCalculation2([foodpos, capsule])])
                                 #return self.aStarSearch([capsule])

        "6. if capsule is near, eat capsule "
        capsules = self.getCapsules(gameState)
        if not len(capsules) == 0:#找最近的药（RANDOM11)
            dis2closetcap = min([self.getMazeDistance(ele, selfState.getPosition()) for ele in capsules])
            for capsule in capsules:
                if self.getMazeDistance(capsule, selfState.getPosition()) == dis2closetcap:
                    enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
                    enemiesAtHome = [ele for ele in enemies if
                                     not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]#我又改成5了
                    scaredEnemies = [ele for ele in enemies if
                                     not ele.isPacman and ele.getPosition() != None and ele.scaredTimer > 0]
    #                print "cap",capsule
                    distance = self.getMazeDistance(capsule, selfState.getPosition())
                    foodpos = self.getClosestFood(gameState,selfState.getPosition())
    #                print foodpos
                    if self.isSuper == False:#现在我知道有幽灵有战力，我需要去尝试吃药，看到了再判断行不行
                        return self.aStarSearch([capsule])
                    if len(enemiesAtHome) > 0: #发现有战力的幽灵在家,如果看到了两只害怕的幽灵就不用继续吃药了，但是如果有一只不在视野内，它还是会吃新药，需要修复
                        if distance < min([self.getMazeDistance(ele.getPosition(),capsule) for ele in enemiesAtHome])-1:
    #                       print "decision"
                           return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])  # 找药吃
                            #return self.aStarSearch([self.gameTheoryCalculation2([foodpos, capsule])])
                            #return self.aStarSearch([capsule])

        "7. back home if chased"               
        if selfState.isPacman:
            # get defenders position
            enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
            theirGhost = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]
            if len(theirGhost) > 0:
                defendersPos = [i.getPosition() for i in theirGhost]
                for pos in defendersPos:
                    distance = self.getMazeDistance(pos,selfState.getPosition()) - 2
                    if distance <= 5:
                        print "BACK HOME!!"
                        self.goBack = True
                        return self.aStarSearch(self.getClosestBorder(gameState))
         
        "8. avoid looping with each other"
        if not selfState.isPacman:
            enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
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
                        self.mustTo = self.gameTheoryCalCurrentDefender(gameState) # 这个地方换成了game theory，上面的部分是高翔的
                        return self.aStarSearch([self.mustTo])

        "9. back home and finished the game"
        if len(foodList) <= 2:
            return self.aStarSearch(self.getClosestBorder(gameState))

        "10. back home safely" 
        if self.goBack == True and not selfState.isPacman:
            self.goBack = False
            return self.aStarSearch(self.getClosestBorder(gameState))
                  
        return self.aStarSearch(self.getFoodByOrder(gameState,selfState.getPosition()))
    
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
      #self.probability = {}  # (position, prob)
      #self.coordinatesWithoutWall = []

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

      
      ##############avoid location: enemy area###############
      self.avoided = []
      if self.red:
        for i in (self.width/2,self.width):
          for j in range(self.height):
            self.avoided.append((i,j))
      else:
        for i in range(self.width/2):
          for j in range(self.height):
            self.avoided.append((i,j))
      #######################################################
      
      
  ##########################A*####################
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
  ################################################

  def chooseAction(self, gameState):
      myPosition = gameState.getAgentPosition(self.index)
      # actions = gameState.getLegalActions(self.index)
      # actions.remove('Stop')
      enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
      #enemy attacker
      enemies_State = []
      for enemy in enemies:
          if enemy.isPacman and enemy.getPosition()!=None:
              enemies_State.append(enemy)
      
      # if myPosition == self.nextPosition:
      #     self.nextPosition = None
          
      #just catch the enemy attacker in sight
      if len(enemies_State) != 0:
          closestEnemy = None
          closestDis = 9999
          for enemy in enemies_State:
              Dis = self.getMazeDistance(myPosition, enemy.getPosition())
              if Dis < closestDis:
                  closestDis = Dis
                  closestEnemy = enemy
                  #self.nextPosition = position
          if closestEnemy != None:
              return self.aStarSearch([closestEnemy.getPosition()])

      #if our food has been eaten, go to that point
      if len(enemies_State) == 0 and self.PreviousFoods != None:
          eastenFood = set(self.PreviousFoods) - set(self.getFoodYouAreDefending(gameState).asList())
          if len(eastenFood)!=0:
              self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()
              return self.aStarSearch([eastenFood.pop()])
              # self.EastenFlag = True
      
      self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()


  def getSuccessor(self, position):
      
      # successor = gameState.generateSuccessor(self.index, action)
      # pos = successor.getAgentState(self.index).getPosition()
      # if pos != nearestPoint(pos):
      #   return successor.generateSuccessor(self.index, action)
      # else:
      #   return successor
      successors = []
      direction = ['North','East','South','West']
      offset = [(0,1),(1,0),(0,-1),(-1,0)]
      for action in zip(direction,offset):
          point = ((position[0] + action[1][0]), (position[1] + action[1][1]))
          if point not in self.avoided:
              successors.append((point,action[0]))
      return successors
