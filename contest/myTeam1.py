#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from captureAgents import CaptureAgent
import random, util
from util import nearestPoint
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

    def make_root_node(self, init_position):
        """
        initialise the root node, of which the information is stored in dict
        """
        init_node = {
            "position": init_position,
            "parent": None,
            "action": None,
        }
        return init_node

    def make_succ_node(self, curr_node, succ_position, succ_action):
        """
        add information in successor nodes
        """
        succ_node = {
            "position": succ_position,
            "parent": curr_node,
            "action": succ_action,
        }
        return succ_node

    def make_actions(self, goal_node):
        """
        return a list of actions that reaches the goal.
        """
        actions = []
        while (goal_node["parent"]):
            actions.append(goal_node["action"])
            goal_node = goal_node["parent"]
        actions = actions[::-1]
        return actions


def createTeam(firstIndex, secondIndex, isRed,  # DefensiveReflexAgent OffensiveReflexAgent
               first='OffensiveReflexAgent', second='DefensiveAgent'):
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
        # self.selfBorder, self.enermyBorder = getBoundaries(self.start[0],gameState.getWalls())

        # whether our pacman is being chasen
        self.goBack = False
        # compulsive move
        self.mustTo = None
        # is super pacman
        self.isSuper = False
        # self.myTeamIndexes = self.getTeam(gameState)
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
                if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle + 1][i]:
                    self.ourDoor.append((middle, i))
        else:
            for i in range(self.mapHeight):
                if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle + 1][i]:
                    self.ourDoor.append((middle + 1, i))

        #############our area && enemy area##############
        self.ourArea = []
        self.theirArea = []
        if self.red:
            for i in range(self.mapWidth / 2):
                for j in range(self.mapHeight):
                    self.ourArea.append((i, j))
            for i in range(self.mapWidth / 2, self.mapWidth):
                for j in range(self.mapHeight):
                    self.theirArea.append((i, j))
        else:
            for i in range(self.mapWidth / 2):
                for j in range(self.mapHeight):
                    self.theirArea.append((i, j))
            for i in range(self.mapWidth / 2, self.mapWidth):
                for j in range(self.mapHeight):
                    self.ourArea.append((i, j))
                    ################################################

    # Astar heuristic search
    def aStarSearch(self, goals):
        init_state = self.getCurrentObservation().getAgentState(self.index).getPosition()
        for goal in goals:
            init_node = model.make_root_node(init_state)
            frontier = util.PriorityQueue()
            frontier.push(init_node, 0)
            explored_set = []
            while not frontier.isEmpty():
                node = frontier.pop()
                if node["position"] not in explored_set:
                    explored_set.append(node["position"])
                    if goal == node["position"]:
                        if node["action"] == None:
                            continue
                        return model.make_actions(node)[0]
                    for successor in self.getSuccessor(node["position"]):
                        # succ_actions = StateModel.make_actions()
                        succ_node = model.make_succ_node(node, successor[0], successor[1])
                        actions = model.make_actions(succ_node)
                        g = len(actions)
                        h = manhattanHeuristic(succ_node["position"], goal)
                        f = g + h
                        frontier.update(succ_node, f)
        # if no action has returned(dead end)
        if len(goals) != 0:
            succ = []
            act = []
            succ = self.getSuccessor(init_state)
            # print "position",init_state
            act = [a[1] for a in succ]
            # print "act",act
            if len(act) > 0:
                return random.choice(act)
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
                        if (ghost.getPosition()[0] + i, ghost.getPosition()[1]) in self.theirArea:
                            avoided.append((ghost.getPosition()[0] + i, ghost.getPosition()[1]))
                        if (ghost.getPosition()[0], ghost.getPosition()[1] + i) in self.theirArea:
                            avoided.append((ghost.getPosition()[0], ghost.getPosition()[1] + i))
        else:
            if len(theirGhosts) > 0:
                for ghost in theirGhosts:
                    for i in range(-1, 2):
                        if (ghost.getPosition()[0] + i, ghost.getPosition()[1]) in self.theirArea:
                            avoided.append((ghost.getPosition()[0] + i, ghost.getPosition()[1]))
                        if (ghost.getPosition()[0], ghost.getPosition()[1] + i) in self.theirArea:
                            avoided.append((ghost.getPosition()[0], ghost.getPosition()[1] + i))
                    # print "avoiding this ghost:"
        #          print ghost.getPosition()
        return avoided

    '''
    generate successors
    '''

    def getSuccessor(self, position):
        # TODO: Add the deadend configuration
        avoided = self.getAvoided()
        successors = []
        direction = ['North', 'East', 'South', 'West']
        offset = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for action in zip(direction, offset):
            point = ((position[0] + action[1][0]), (position[1] + action[1][1]))
            if point not in avoided:
                successors.append((point, action[0]))
        return successors

    '''
    get the closest food based on given position
    '''

    def getClosestFood(self, gameState, position):
        # print "into getClosestFood"
        theirPacman = []
        enemies = []
        myState = self.getCurrentObservation().getAgentState(self.index)
        for i in self.getOpponents(self.getCurrentObservation()):
            enemies.append(self.getCurrentObservation().getAgentState(i))
        for enemy in enemies:
            if enemy.isPacman and enemy.getPosition() != None:
                theirPacman.append(enemy)
        if (not myState.isPacman) and myState.scaredTimer <= 0:
            if len(theirPacman) > 0:
                distanceToEnemy_min = 99999
                closestEnemy = None
                for pac in theirPacman:
                    dis = self.getMazeDistance(position, pac.getPosition())
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

    def getFoodByOrder(self, gameState, position):
        # print "into getFoodByOrder"
        theirPacman = []
        enemies = []
        myState = self.getCurrentObservation().getAgentState(self.index)
        for i in self.getOpponents(self.getCurrentObservation()):
            enemies.append(self.getCurrentObservation().getAgentState(i))
        for enemy in enemies:
            if enemy.isPacman and enemy.getPosition() != None:
                theirPacman.append(enemy)
        if (not myState.isPacman) and myState.scaredTimer <= 0:
            if len(theirPacman) > 0:
                distanceToEnemy_min = 99999
                closestEnemy = None
                for pac in theirPacman:
                    dis = self.getMazeDistance(position, pac.getPosition())
                    if dis < distanceToEnemy_min:
                        distanceToEnemy_min = dis
                        closestEnemy = pac
                return [closestEnemy.getPosition()]

        foods = self.getFood(gameState).asList()

        def myCmp(x, y):
            return self.getMazeDistance(position, x) - self.getMazeDistance(position, y)

        foods.sort(cmp=myCmp)
        # print foods
        return foods

    def getClosestBorder(self, gameState):
        position = self.getCurrentObservation().getAgentPosition(self.index)
        collection = util.PriorityQueue()
        for border in self.ourDoor:
            dis = self.getMazeDistance(position, border)
            collection.push((border), dis)
        if collection.count == 0:
            return [self.start]
        else:
            return [collection.pop()]

    #################our pacman can make a detour if staling with enemies#####################

    def gameTheoryCalCurrentDefender(self, gameState):
        # print('into gameTheoryCalCurrentDefender')
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
                enemyValueList = [self.getMazeDistance(ele, selfBorderPoint) for ele in defendersPos]
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
        defenders = [ele for ele in enemies if not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
        matrix = [[0, 0], [0, 0]]
        matrix[0][0] = self.getMazeDistance(selfState.getPosition(), pos1)
        matrix[1][0] = self.getMazeDistance(selfState.getPosition(), pos2)
        if len(defenders) > 0:
            # defendersPos = [i.getPosition() for i in defenders]
            matrix[0][1] = min([self.getMazeDistance(pos1, defender.getPosition()) for defender in defenders])
            matrix[1][1] = min([self.getMazeDistance(pos2, defender.getPosition()) for defender in defenders])
        for i in range(0, 2):
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
        # matrix = [[0, 0], [0, 0]]
        for i in range(0, len(pos)):
            matrix[i][0] = self.getMazeDistance(selfState.getPosition(), pos[i])
        if len(defenders) > 0:
            # defendersPos = [i.getPosition() for i in defenders]
            for i in range(0, len(pos)):
                matrix[i][1] = min([self.getMazeDistance(pos[i], defender.getPosition()) for defender in defenders])
            # matrix[1][1] = min([self.getMazeDistance(pos2, defender.getPosition()) for defender in defenders])
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
        # if(self.mustTo != None):
        #     print('Force go to*********'+str(self.mustTo))
        #     position = Observation.getAgentPosition(self.index)
        #     if(position == self.mustTo):
        #         self.mustTo = None
        #     else:
        #         return self.aStarSearch([self.mustTo])

        foodList = self.getFood(gameState).asList()
        foodAte = self.foodNum - len(foodList)

        "2. calculate the remaining foods"
        if not selfState.isPacman:
            self.foodNum = len(self.getFood(gameState).asList())

        "3. if already ate more than 5 foods, back home to get the points"
        if foodAte > 7:
            return self.aStarSearch(self.getClosestBorder(gameState))

        "4. if our pacman get a capsule"
        if selfState.getPosition() in self.capsulesPosition:
            self.isSuper = True
            # print "Agent get Capsule!"
            self.capsulesPosition.remove(selfState.getPosition())

        "5. if our pacman run out of the capsule"
        if self.isSuper:
            enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
            recoveredGhost = [ele for ele in enemies if
                              not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]  # 预留了5秒
            if len(recoveredGhost) != 0:
                self.isSuper = False
        "6. if capsule is near, eat capsule "

        if not self.isSuper:
            capsules = self.getCapsules(gameState)
            # capsules = [(99,99)]
            if not len(capsules) == 0:
                dis2closetcap = min([self.getMazeDistance(ele, selfState.getPosition()) for ele in capsules])
                for capsule in capsules:
                    if self.getMazeDistance(capsule, selfState.getPosition()) == dis2closetcap:
                        enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
                        enemiesAtHome = [ele for ele in enemies if
                                         not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 3]
                        scaredEnemies = [ele for ele in enemies if
                                         not ele.isPacman and ele.getPosition() != None and ele.scaredTimer > 0]
                        #                print "cap",capsule
                        distance = self.getMazeDistance(capsule, selfState.getPosition())
                        foodpos = self.getClosestFood(gameState, selfState.getPosition())
                        if not len(enemiesAtHome) > 0:
                            # return self.aStarSearch([capsule])
                            return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])
                        else:
                            if distance < min([self.getMazeDistance(ele.getPosition(), capsule) for ele in
                                               enemiesAtHome]) and len(enemies) == 2:
                                # print "decision"
                                if min([self.getMazeDistance(ele.getPosition(), selfState.getPosition()) for ele in
                                        enemiesAtHome]) <= 7:
                                    return self.aStarSearch([capsule])
                                else:
                                    return self.aStarSearch([self.gameTheoryCalculation1(foodpos, capsule)])
        # "6. if capsule is near, eat capsule "
        # if not self.isSuper:
        #     capsules = self.getCapsules(gameState)
        #     if not len(capsules) == 0:
        #         dis2closetcap = min([self.getMazeDistance(ele, selfState.getPosition()) for ele in capsules])
        #         for capsule in capsules:
        #             if self.getMazeDistance(capsule, selfState.getPosition()) == dis2closetcap:
        #                 enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
        #                 enemiesAtHome = [ele for ele in enemies if
        #                                  not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 3]
        #                 scaredEnemies = [ele for ele in enemies if
        #                                  not ele.isPacman and ele.getPosition() != None and ele.scaredTimer > 0]
        #                 #                print "cap",capsule
        #                 distance = self.getMazeDistance(capsule, selfState.getPosition())
        #                 foodpos = self.getFoodByOrder(gameState, selfState.getPosition())
        #                 foodpos.append(capsule)
        #                 #print foodpos[0],capsule
        #                 #print self.gameTheoryCalculation2(foodpos)
        #                 return self.aStarSearch([self.gameTheoryCalculation2(foodpos)])
        "7. back home if chased"
        if selfState.isPacman:
            # get defenders position
            enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
            theirGhost = [ele for ele in enemies if
                          not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 5]
            if len(theirGhost) > 0:
                defendersPos = [i.getPosition() for i in theirGhost]
                for pos in defendersPos:
                    distance = self.getMazeDistance(pos, selfState.getPosition()) - 2
                    if distance <= 5:
                        # print "BACK HOME!!"
                        self.goBack = True
                        return self.aStarSearch(self.getClosestBorder(gameState))

        "8. avoid looping with each other"
        # if not selfState.isPacman:
        #     enemies = [Observation.getAgentState(i) for i in self.getOpponents(Observation)]
        #     enemiesAtHome = [ele for ele in enemies if
        #                      not ele.isPacman and ele.getPosition() != None and ele.scaredTimer <= 0]
        #     if len(enemiesAtHome) > 0:
        #         defendersPos = [i.getPosition() for i in enemiesAtHome]
        #         for ele in enemiesAtHome:
        #             closestb = self.getClosestBorder(ele)[0]
        #             distanceene2border = self.getMazeDistance(ele.getPosition(), closestb) - 2
        #             distanceme2border = self.getMazeDistance(selfState.getPosition(), self.getClosestBorder(gameState)[0]) - 2
        #             distance = self.getMazeDistance(ele.getPosition(), selfState.getPosition()) - 2
        #             if distance <= 5 and distanceene2border <= 3 and distanceme2border <= 3:
        #                 self.mustTo = self.gameTheoryCalCurrentDefender(gameState)
        #                 return self.aStarSearch([self.mustTo])

        "9. back home and finished the game"
        if len(foodList) <= 2:
            return self.aStarSearch(self.getClosestBorder(gameState))

        "10. back home safely"
        if self.goBack == True and not selfState.isPacman:
            self.goBack = False
            return self.aStarSearch(self.getClosestBorder(gameState))

        return self.aStarSearch(self.getFoodByOrder(gameState, selfState.getPosition()))

####################
# Defensive Agents #
####################

class DefensiveAgent(CaptureAgent):

    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.nextPosition = None
        self.PreviousFoods = None
        self.EastenFlag = False
        self.width = gameState.data.layout.width
        self.height = gameState.data.layout.height
        self.guardingdoor = []
        self.walls = gameState.getWalls().asList()
        self.guardfood = []

        # self.probability = {}  # (position, prob)
        # self.coordinatesWithoutWall = []
        self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()
        middle = self.width / 2 - 1
        self.ourDoor = []
        if self.red:
            for i in range(self.height):
                if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle + 1][i]:
                    self.ourDoor.append((middle, i))
        else:
            for i in range(self.height):
                if not gameState.getWalls()[middle][i] and not gameState.getWalls()[middle + 1][i]:
                    self.ourDoor.append((middle + 1, i))

        if self.height > 10:
            for ele in self.ourDoor:
                if ele[1] >= 5 and ele[1] <= self.height - 5:
                    self.guardingdoor.append(ele)
            self.topguaringdoorx = self.ourDoor[0][0]
            self.topguaringdoory = max([ele[1] for ele in self.guardingdoor])
            self.topguaringdoor = (self.topguaringdoorx, self.topguaringdoory)
            self.butguaringdoorx = self.ourDoor[0][0]
            self.butguaringdoory = min([ele[1] for ele in self.guardingdoor])
            self.butguaringdoor = (self.butguaringdoorx, self.butguaringdoory)
            self.guardinggoal = self.topguaringdoor

        if self.height <= 10:
            for ele in self.ourDoor:
                if ele[1] >= 1 and ele[1] <= self.height - 1:
                    self.guardingdoor.append(ele)
            self.topguaringdoorx = self.ourDoor[0][0]
            self.topguaringdoory = max([ele[1] for ele in self.guardingdoor])
            self.topguaringdoor = (self.topguaringdoorx, self.topguaringdoory)
            self.butguaringdoorx = self.ourDoor[0][0]
            self.butguaringdoory = min([ele[1] for ele in self.guardingdoor])
            self.butguaringdoor = (self.butguaringdoorx, self.butguaringdoory)
            self.guardinggoal = self.topguaringdoor
        #######################################################

    ##########################A*####################
    def aStarSearch(self, goals):
        init_state = self.getCurrentObservation().getAgentState(self.index).getPosition()
        for goal in goals:
            init_node = model.make_root_node(init_state)
            frontier = util.PriorityQueue()
            frontier.push(init_node, 0)
            explored_set = []
            while not frontier.isEmpty():
                node = frontier.pop()
                if node["position"] not in explored_set:
                    explored_set.append(node["position"])
                    if goal == node["position"]:
                        if node["action"] == None:
                            continue
                        return model.make_actions(node)[0]
                    for successor in self.getSuccessor(node["position"]):
                        # succ_actions = StateModel.make_actions()
                        succ_node = model.make_succ_node(node, successor[0], successor[1])
                        actions = model.make_actions(succ_node)
                        g = len(actions)
                        h = manhattanHeuristic(succ_node["position"], goal)
                        f = g + h
                        frontier.update(succ_node, f)
        return 'Stop'

    ################################################

    def chooseAction(self, gameState):
        myPosition = gameState.getAgentPosition(self.index)
        # actions = gameState.getLegalActions(self.index)
        # actions.remove('Stop')
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        # self.guardfood = []
        # enemy attacker
        enemies_State = []
        for enemy in enemies:
            if enemy.isPacman and enemy.getPosition() != None:
                enemies_State.append(enemy)

        # if myPosition == self.nextPosition:
        #     self.nextPosition = None

        # just catch the enemy attacker in sight

        if len(enemies_State) != 0:
            self.guardfood = []
            closestEnemy = None
            closestDis = 9999
            for enemy in enemies_State:
                Dis = self.getMazeDistance(myPosition, enemy.getPosition())
                if Dis < closestDis:
                    closestDis = Dis
                    closestEnemy = enemy
                    # self.nextPosition = position
            if closestEnemy != None:
                if gameState.getAgentState(self.index).scaredTimer > 0 and closestDis <= 2:
                    return self.aStarSearch([self.guardinggoal])
                else:
                    return self.aStarSearch([closestEnemy.getPosition()])

        # if our food has been eaten, go to that point
        if len(enemies_State) == 0 and self.PreviousFoods != None:
            eastenFood = list(set(self.PreviousFoods) - set(self.getFoodYouAreDefending(gameState).asList()))
            if len(eastenFood) != 0:
                # print "aaaaaaaaaaaaa",eastenFood[0]
                self.guardfood = eastenFood
                # print "aaaaaaaaaaaaaaaaaa",self.guardfood
                self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()
                return self.aStarSearch(self.guardfood)
                # self.EastenFlag = True
        self.PreviousFoods = self.getFoodYouAreDefending(gameState).asList()

        if len(self.guardfood) == 0:
            if myPosition == self.topguaringdoor:
                self.guardinggoal = self.butguaringdoor
            if myPosition == self.butguaringdoor:
                self.guardinggoal = self.topguaringdoor
            return self.aStarSearch([self.guardinggoal])
        elif myPosition == self.guardfood[0]:
            self.guardfood = []
            return self.aStarSearch([self.guardinggoal])
        else:
            return self.aStarSearch(self.guardfood)

    def getSuccessor(self, position):

        # successor = gameState.generateSuccessor(self.index, action)
        # pos = successor.getAgentState(self.index).getPosition()
        # if pos != nearestPoint(pos):
        #   return successor.generateSuccessor(self.index, action)
        # else:
        #   return successor
        avoided = copy.copy(self.walls)
        ##############avoid location: enemy area###############

        if self.red:
            for i in (self.width / 2, self.width):
                for j in range(self.height):
                    avoided.append((i, j))
        else:
            for i in range(self.width / 2):
                for j in range(self.height):
                    avoided.append((i, j))
        myState = self.getCurrentObservation().getAgentState(self.index)
        # print "myState.getPosition()", myState.getPosition()
        if myState.scaredTimer > 0:

            for i in self.getOpponents(self.getCurrentObservation()):
                # print "len(self.getOpponents(self.getCurrentObservation()))",len(self.getOpponents(self.getCurrentObservation()))
                enemy = self.getCurrentObservation().getAgentState(i)
                # print "enemy.getPosition()",enemy.getPosition()
                if enemy.getPosition() != None:
                    if enemy.isPacman and (self.getMazeDistance(enemy.getPosition(), myState.getPosition())) <= 2:
                        avoided.append(enemy.getPosition())
        successors = []
        direction = ['North', 'East', 'South', 'West']
        offset = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for action in zip(direction, offset):
            point = ((position[0] + action[1][0]), (position[1] + action[1][1]))
            if point not in avoided:
                successors.append((point, action[0]))
        return successors
