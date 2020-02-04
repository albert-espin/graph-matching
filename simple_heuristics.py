#!/usr/bin/python
# -*- coding: utf-8 -*-

"""VERSIÓ D'HEURÍSTIQUES SIMPLES DE LA PRÀCTICA D'ENUMERATIUS. AQUEST NO ÉS L'ARXIU AMB HEURÍSTIQUES ELABORADES, NOMÉS SERVEIX PER CONTRASTAR L'EFICIÈNCIA AMB AQUELL. És per això que no consta de comentaris complets i no fa servir totes les estratègies de major eficiència que l'altre arxiu, "branch_bound", sí utilitza.
"""


import networkx as nx
import sys
import time
import copy



# DEFINICIÓ DE CONSTANTS (per fer més comprensible l'anàlisi del codi):

# valor enter equiparat a infinit a fer servir uniformement a les funcions del programa
INFINITY = sys.maxint



def main(entrada="grafMitja.dat"):
    '''Funció principal del programa, rep un nom de fitxer que pot ser fixat per consola que s'utilitza per llegir un graf representatiu d'un conjut de contactes relacionables sobre el qual es porta a terme un matching amb algorismes enumeratius'''
    G = llegir_graf(entrada)
    t =  time.clock()
    matching(G)
    print u"Temps de càlcul: {} segons".format(time.clock() - t)    
    
    
    
def llegir_graf(entrada):
    filename = raw_input(u"Dóna el nom del fitxer del graf,\no prem <Enter> per usar el valor de consola: ") 
    if filename == "":
        filename = entrada 
    G = nx.read_edgelist(filename, nodetype=int, data=(("rejection",float),))  
    return G



def matching(graph):

    wasSolved = False
    if isMatchingPossible(graph):
        
        initialPartialSolution = buildSolution(graph, [obtainBestInitialNode(graph)], 0.0)
        
        if initialPartialSolution != None:
        
            optimumSolution = None
            
            superiorHeight = INFINITY
    
            optimumSolution, superiorHeight = backtracking(graph, initialPartialSolution, optimumSolution, superiorHeight)

            if optimumSolution != None:
                wasSolved = True
            
                pairsStr = ""
                for pair in optimumSolution["pairs"]:
                    pairsStr += "{} <-> {}  (rebuig: {})\n".format(pair[0], pair[1], graph[pair[0]][pair[1]]["rejection"])
                print u"L'emparellament òptim involucra les parelles:\n{}Rebuig total : {}".format(pairsStr, optimumSolution["rejection"])
        
    if not wasSolved:
        print u"És impossible emparellar tots els contactes sense que cap es repeteixi, quedi sense emparellar o pugui evitar fer-ho amb rebuig infinit."
    
    
    
def isMatchingPossible(graph):
    '''Funció que comprova, donat un graf passat per paràmetre, que sigui possible obtenir un emparellament vàlid de tots els nodes sense cap repetició ni abandonament, i tampoc fent cap enllaç amb rebuig infinit'''
    # cal que el nombre de nodes sigui parell per poder emparellar
    return graph.number_of_nodes() % 2 == 0

 
 
def backtracking(graph, solution, optimumSolution, superiorHeight): 
    
    if isSolutionComplete(graph, solution):
        if isSolutionBetterThanCurrentOptimum(solution, optimumSolution):
            optimumSolution = solution
            superiorHeight = optimumSolution["inferiorHeight"]

    else:
        if isPartialSolutionViable(graph, solution, superiorHeight):
            childSolutions = sorted(obtainAllChildSolutions(graph, solution), key=lambda child: child["inferiorHeight"])
            
            for childSolution in childSolutions:
                optimumSolution, superiorHeight = backtracking(graph, childSolution, optimumSolution, superiorHeight)
            
    return optimumSolution, superiorHeight



def buildSolution(graph, pairs, rejection):
    '''Funció que a partir de diferents paràmetres constituents construeix i retorna una solució (parcial o completa) del problema; al nostre cas aquests paràmetres són una llista de parelles de nodes i un valor de rebuig; a partir d'un paràmetre addicional, el graf, calcula la cota inferior de la solució'''

    inferiorHeight = obtainInferiorHeight(graph, pairs, rejection)
    
    # si calculant la cota inferior ens trobem amb l'evidència que de la solució a construir no és completa i, com a solució parcial, és impossible que poguem derivar una solució completa vàlida, evitem crear la solució, retornem un valor buit
    if inferiorHeight == INFINITY:
        return None
        
    return {"pairs" : pairs, "rejection" : rejection, "inferiorHeight" : inferiorHeight}



def obtainInferiorHeight(graph, pairs, accumulatedRejection):
    
    return accumulatedRejection
    


def obtainAllChildSolutions(graph, partialSolution):
    '''Funció que, donada una solució parcial al problema passada per paràmetre, retorna totes les solucions que es representarien com filles a l'arbre de solucions'''
    
    childSolutions = []
    pairs = []
    newChildPairs = []
    pair = ()
    solution = {}
    rejection = 0.0
    
    if not doesSolutionHaveAnyPairs(partialSolution):
        initialNode = partialSolution["pairs"][0]
        for neighbor in graph.neighbors(initialNode):
            if initialNode > neighbor:
                pair = (neighbor, initialNode)
            else:
                pair = (initialNode, neighbor)
                
            pairs = [pair]
            rejection = graph[initialNode][neighbor]["rejection"]
            solution = buildSolution(graph, pairs, rejection)
            if solution != None:
                childSolutions.append(solution)
            
    else:
        
        usedNodes = []
        for solutionPair in partialSolution["pairs"]:
            for node in solutionPair:
                if node not in usedNodes:
                    usedNodes.append(node)

        for node in graph.nodes():
            if node not in usedNodes:
                for neighbor in graph.neighbors(node):
                    if neighbor not in usedNodes:
                        pairs = []
                        for solutionPair in partialSolution["pairs"]:
                            pairs.append(tuple(solutionPair))
                            
                        if node > neighbor:
                            pair = (neighbor, node)
                        else: 
                            pair = (node, neighbor)
                            
                        if pair not in newChildPairs:
                            pairs.append(pair)
                            newChildPairs.append(pair)
                            rejection = partialSolution["rejection"] + graph[node][neighbor]["rejection"]
                            solution = buildSolution(graph, pairs, rejection)
                            if solution != None:
                                childSolutions.append(solution) 
           
    return childSolutions
            


def isSolutionComplete(graph, solution):
    '''Funció que retorna si la solució passada per paràmetre és una solució completa al problema'''

    # Considerem una solució completa quan té tots els nodes del graf emparellats, és a dir, que hi ha tantes parelles com la meitat dels nodes. Com que els mecanismes per construir solucions ja s'encarreguen que no repetim nodes a les parelles, aquí no calen comprovacions addicionals
    
    return len(solution["pairs"]) == graph.number_of_nodes() / 2



def isSolutionBetterThanCurrentOptimum(solution, currentOptimumSolution):
    '''Funció que determina si la solució passada per paràmetre és millor que la que fins ara es considerava la millor trobada, també passada en qualitat de paràmetre'''
    
    # una solució completa qualsevol és millor que una nul·la
    if currentOptimumSolution == None:
        return True
    
    # si la solució considera òptima fins al moment existeix, la candidata serà millor només si té menor rebuig total
    return solution["rejection"] < currentOptimumSolution["rejection"]



def isPartialSolutionViable(graph, solution, superiorHeight):
    '''Funció que determina, tenint en compte la cota superior actual, si una solució parcial és viable o per contra convé descartar-la a ella i a tota la seva hipotètica descendència a l'arbre de solucions'''
    
    # una solució és viable només si la seva cota inferior és inferior a la cota superior que el programa està fent servir
    return solution["inferiorHeight"] < superiorHeight and not hasSolutionAbandonedAnyNodes(graph, solution)
    
   
   
def obtainBestInitialNode(graph):
    '''Funció que retorna el millor node per començar a buscar la solució òptima de l'algorisme; formarà la solució parcial mara de la qual derivar tot l'arbre de solucions'''
    
    # considerem el millor node per començar l'arbre de solucions aquell que té el major nombre de veïns
    bestNode = None
    bestNeighborsCount = 0
    for node in graph.nodes():
        if len(graph.neighbors(node)) > bestNeighborsCount:
            bestNode = node
            bestNeighborsCount = len(graph.neighbors(node))
    return bestNode
    
    
def doesSolutionHaveAnyPairs(solution):
    '''Funció que retorna si una solució (parcial o completa) està formada per parelles de nodes, o per el contrari té únicament un node inicial, que serveix com a punt de partida per l'algorisme'''
    return not isPairActuallyANode(solution["pairs"][0])


def isPairActuallyANode(pair):
    '''Funció que retorna si una estructura que a priori es considera una parella és en realitat únicament un node (que es fa servir a l'inici de l'algorisme, com a solució parcial mare)'''
    return type(pair) != tuple
    
 
def hasSolutionAbandonedAnyNodes(graph, solution):
    '''Funció que retorna si, amb les parelles fins ara afegides a una solució passada per paràmetre ha quedat algun node del graf aïllat de tal manera que independentment de les parelles que es facin no podrà ser emparellat (els nodes amb els quals té rebuig no infinit ja estan emparellats)'''
    
    # si cap parella no ha estat afegida encara a la solució, no té sentit considerar nodes abandonats
    if not doesSolutionHaveAnyPairs(solution):
        return False
    
    exploredNodes = []    
    for pair in solution["pairs"]:
        for node in pair:
            if node not in exploredNodes:
                exploredNodes.append(node)
    
    for node in graph.nodes():
        if node not in exploredNodes:
            abandoned = True
            for neighbor in graph.neighbors(node):
                if neighbor not in exploredNodes:
                    abandoned = False
                    break
            if abandoned: 
                return True
            
    return False


    
    
if __name__ == "__main__":
    main(*sys.argv[1:])
