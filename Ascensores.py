from search import *
from search import breadth_first_tree_search, depth_first_tree_search, depth_first_graph_search, breadth_first_graph_search

class Ascensores(Problem):

    def __init__(self, initial, goal, ascen): ##estado (plantas de pasajeros, (plantas de ascensores))
        self.ascen = ascen
        ##self.nombres = nombres
        Problem.__init__(self, initial, goal)

    def actions(self, state): ##[pasajeros], destino, ascensor
        possible_actions = list()
        for n in range(len(self.ascen)):
            plantaAsc = state[-1][n]
            pasajerosEnPlanta = list()
            for i in range(0, len(state) - 1):
                if state[i] == plantaAsc: pasajerosEnPlanta.append(i)
            combPasajeros = Ascensores.nonRepeatedCombinations(pasajerosEnPlanta, self.ascen[n][1])
            combPasajeros.append([])
            for j in range(len(self.ascen[n][0])):
                if self.ascen[n][0][j] != plantaAsc:
                    for i in range(len(combPasajeros)):
                        possible_actions.append((combPasajeros[i], self.ascen[n][0][j], n))
            possible_actions.append(([], plantaAsc, n))
        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """
        lstate = list(state)
        lascenPlantas = list(lstate[-1])
        destino = action[1]
        ascensorActual = action[2]
        for i in range(len(action[0])):
            lstate[action[0][i]] = destino
        lascenPlantas[ascensorActual] = destino
        lstate[-1] = tuple(lascenPlantas)
        result = tuple(lstate)
        return result

    def goal_test(self, state):
        return state[:len(state) - 1] == self.goal ##No se tiene en cuenta la posicion de los ascensores ni su turno

    def hNumPersonas(self, node):
        h = 0
        plantasActuales = node.state[:len(node.state) - 1]
        plantasObjetivo = self.goal
        for i in range(len(plantasActuales)):
            h += abs(plantasActuales[i] - plantasObjetivo[i])
        return h

    def hNumPersPlantAscensor(self, node):
        h = 0
        plantasActuales = node.state[:len(node.state) - 1]
        plantasObjetivo = self.goal
        plantasAscensores = node.state[-1]
        for i in range(len(plantasActuales)):
            ascensoresEnPlanta = 0
            ascensoresQueAlcanzanDest = 0
            for j in range(len(plantasAscensores)):
                if plantasAscensores[j] == plantasActuales[i]:
                    ascensoresEnPlanta += 1
                    if plantasObjetivo[i] in self.ascen[j][0]:
                        ascensoresQueAlcanzanDest += 1
            h += abs(plantasActuales[i] - plantasObjetivo[i]) * (len(self.ascen) - ascensoresEnPlanta + 1) * (ascensoresEnPlanta - ascensoresQueAlcanzanDest + 1)
        return h

    def h(self, node): ##estado (plantas de pasajeros, (plantas de ascensores))
        return 1

    def path_cost(self, c, state1, action, state2):
        ascensor = action[-1]
        destino = action[-2]
        origen = state1[-1][ascensor]
        return c + abs(destino - origen) * self.ascen[ascensor][2]

    @staticmethod
    def nonRepeatedCombinations(tElems, maxLength):
        #if len(tElems) < maxLength: return []
        combs = list()
        Ascensores.nonRepeatedCombinationsAux(tElems, [], maxLength, 0, combs)
        return combs

    @staticmethod
    def nonRepeatedCombinationsAux(tElems, combArr, maxLength, lastIdx, acumSol):
        for idx in range(lastIdx, len(tElems)):
            combArr.append(tElems[idx])
            acumSol.append(combArr.copy())
            if len(combArr) < maxLength: Ascensores.nonRepeatedCombinationsAux(tElems, combArr, maxLength, idx + 1, acumSol)
            combArr.pop()


    def value(self, state):
        pass

ascen = (((0,1,2,3,4), 2, 2), ((4,5,6,7,8), 2, 2), ((4,5,6,7,8), 2, 2), ((8,9,10,11,12), 2, 2), ((0,2,4,6,8,10,12), 3, 1)) ##Ascensores ((plantas validas), capacidad, tiempos)
estado = Ascensores((2, 4,1,8,1, (0,4,4,8,12)), (3,11,12,1,9), ascen)
print(greedy_best_first_graph_search(estado, estado.hNumPersonas).solution())
#print(breadth_first_graph_search(estado).solution())
#print(astar_search(estado, estado.h).solution())

#solucion con busqueda voraz y heruristica numero de personas fuera de su planta
#[([1], 8, 2), ([3], 4, 2), ([1], 11, 3), ([], 8, 3), ([], 4, 4), ([3], 0, 4), ([3], 1, 0), ([2, 4], 4, 0), ([2, 4], 8, 2), ([2], 12, 3), ([], 0, 0), ([], 4, 2), ([], 8, 3), ([4], 9, 3), ([], 8, 3), ([], 2, 0),
#([0], 3, 0)]
