#!/usr/bin/python
# -*- coding: utf-8 -*-


import networkx as nx
import sys
import time
import copy
import heapq


# DEFINICIÓ DE CONSTANTS (per fer més comprensible l'anàlisi del codi):

# valor enter equiparat a infinit a fer servir uniformement a les funcions del programa
INFINITY = sys.maxint

# valor identificat a l'índex amb què en una posició de la cua de prioritats s'accedeix al valor de prioritat associat a un element de la cua
QUEUE_PRIORITY_INDEX = 0

# valor amb què en una posició de la cua de prioritats s'accedeix a l'element contingut, no al seu valor de prioritat
QUEUE_ELEMENT_INDEX = 1



def main(entrada="exemple1.dat"):
    '''Funció principal del programa, rep un nom de fitxer que pot ser fixat per consola que s'utilitza per llegir un graf representatiu d'un conjut de contactes relacionables sobre el qual es porta a terme un matching amb algorismes enumeratius'''
    graph = extractGraphFromFile(entrada)
    startTime = time.clock()
    matching(graph)
    endTime = time.clock() - startTime
    print u"\nTemps de càlcul: {} segons\n\t\t({} milisegons)".format(endTime, endTime*1000)
    
    
    
def extractGraphFromFile(entrada):
    '''Donat un nom de fitxer per defecte, s'utilitza, a menys que l'usuari indiqui un altre, per a extreure un graf a partir de dades de nodes connectats per arestes de cert pes; es retorna el graf creat'''
    filename = raw_input(u"Dóna el nom del fitxer del graf,\no prem <Enter> per usar el valor de consola: ")
    if filename == "":
        filename = entrada  
    graph = nx.read_edgelist(filename, nodetype=int, data=(("rejection",float),)) 
    return graph   



def matching(graph):
    '''Funció que comprova certes condicions per determinar si en principi es podrà realitzar l'aparellament dels nodes del graf, i si troba que a priori és possible condueix a realitzar aquest aparellament per ramificació i poda, amb la peculiaritat que abans de fer-ho divideix el graf en components connexes i realitza l'aparellament per separat a cada una d'elles, la qual cosa aporta un guany d'eficiència extraordinari en grafs amb molts nodes i amb diversos components connexos'''
    
    # perquè es pugui realitzar l'aparellament cal un nombre parell de nodes
    impossibleToSolve = not isGraphNumberOfNodesEven(graph)
    
    if not impossibleToSolve:
    
        nodeCount = graph.number_of_nodes()
        edgeCount = graph.number_of_edges()
        
        # llista de tàndems parella-rebuig, tractada com una cua en afegir i extreure elements simplement per poder mostrar després les parelles en ordre (i tenint en compte que la cerca en una cua implementada com a heap és inferior a una cerca lineal)
        pairAndRejectionDicts = []
        
        totalRejection = 0
        
        # comencem obtenint l'aparellament dels nodes amb grau 1, que només tenen una opció d'aparellament, i afegim aquestes dades a la llista general de diccionaris de parelles i valors de rebuigs
        dicts = matchSinglePairNodes(graph)
        for dict in dicts:
            pair = dict["pair"]
            rejection = dict["rejection"]
            heapq.heappush(pairAndRejectionDicts, (pair[0], {"pair" : pair, "rejection" : rejection}))
            totalRejection += rejection
            
        # obtenim els components connexos del graf inicial com a subgrafs
        connectedComponents = obtainGraphConnectedComponentsAsGraphs(graph)
        
        # realitzem l'aparellament per ramificació i poda a cada component connex per separat
        for connectedComponent in connectedComponents:
            # comprovem que el component connex tingui un nombre parell de nodes, o no podrà haver aparellament complet
            if not isGraphNumberOfNodesEven(connectedComponent):
                impossibleToSolve = True
        
        if not impossibleToSolve:
            
            pairsStr = ""
            
            for connectedComponent in connectedComponents:

                optimumSolution = branchAndBound(connectedComponent)
                
                # si s'ha trobat un conjunt de parelles com a aparellament òptim pel component connex, es guarden a la llista de solucions referides al graf inicial sencer
                if optimumSolution != None:
                    for pair in optimumSolution["pairs"]:
                        rejection = connectedComponent[pair[0]][pair[1]]["rejection"]
                        heapq.heappush(pairAndRejectionDicts, (pair[0], {"pair" : pair, "rejection" : rejection}))
                    totalRejection += optimumSolution["rejection"]
                    
                # si no s'ha trobat cap solució vol dir que l'aparellament de tot el graf no és possible
                else:
                    impossibleToSolve = True
                    break
              
            # si la naturalesa del graf ha fet possible un aparellament complet, es mostren les parelles realitzades, el rebuig de cada una i el rebuig total
            if not impossibleToSolve:
                print u"\nEl graf escollit té {} nodes i {} arestes.".format(nodeCount, edgeCount)
                
                for i in range(len(pairAndRejectionDicts)):
                    dict = heapq.heappop(pairAndRejectionDicts)
                    pair = dict[1]["pair"]
                    rejection = dict[1]["rejection"]
                    pairsStr += "{} <-> {}  (rebuig: {})\n".format(pair[0], pair[1], rejection) 
                print u"L'aparellament òptim involucra {} parelles:\n{}Rebuig total : {}".format(nodeCount/2, pairsStr, totalRejection)
        
    # si el graf no permet un aparellament complet, s'indica aquest fet per pantalla
    if impossibleToSolve:
        print u"És impossible emparellar tots els contactes sense que cap es repeteixi, quedi sense emparellar o pugui evitar fer-ho amb rebuig infinit."
        
        
        
def matchSinglePairNodes(graph):
    '''Funció que aparella els nodes que només tenen una aresta i elimina del graf tots els nodes aparellats, cosa que permet reduir posteriorment l'espai de solucions possibles; retorna les parelles realitzades i el seu rebuig'''
      
    singlePairNodes = []
    pairAndRejectionDicts = []
    rejection = 0
    
    # localitza els nodes de grau 1
    for node in graph.nodes():
        if len(graph.neighbors(node)) == 1:
            singlePairNodes.append(node)
     
    # per cada node amb un sol veí, si aquest veí no ha estat emparellat amb un altre node, s'aparella el node amb el seu veí, es construeix una solució amb la parella i s'eliminen els nodes del graf així com les arestes a les quals participen 
    for node in singlePairNodes:
        if node in graph.nodes() and graph.neighbors(node):
            neighbor = graph.neighbors(node)[0]
            if neighbor in graph.nodes():
                if neighbor < node:
                    node, neighbor = neighbor, node
                pair = (node, neighbor)
                rejection = graph[node][neighbor]["rejection"]
                pairAndRejectionDicts.append({"pair" : pair, "rejection" : rejection})
                graph.remove_node(node)
                graph.remove_node(neighbor)
        
    # retorn dels tàndems parella-rebuig realitzats en aparellar
    return pairAndRejectionDicts
        
        
        
def obtainGraphConnectedComponentsAsGraphs(graph):
    '''Funció que donat un graf passat per paràmetre retorna, com a llista de grafs independents, els components connexes que el formen''' 
    
    # trobem els nodes que formen part de cada component connex, cada grup en una llista
    componentsNodes = depthFirstSearch(graph)

    # si només s'ha trobat una llista de nodes que formen un component connex vol dir que tot el graf inicial és un sol component connex i no té sentit cap descomposició
    if len(componentsNodes) == 1:
        return [graph]
    
    connectedComponents = []
    
    # si per contra hi ha diferents components connexos, creem un subgraf per a cada un d'ells, amb els nodes i arestes corresponents al component
    for nodeList in componentsNodes:
        componentGraph = nx.Graph()
        for node in nodeList:
            for neighbor in graph.neighbors(node):
                componentGraph.add_edge(node, neighbor, rejection=graph[node][neighbor]["rejection"])
            graph.remove_node(node)
        connectedComponents.append(componentGraph)

    # retornem el conjunt de components connexos com una llista de subgrafs
    return connectedComponents



def depthFirstSearch(graph):
    '''Funció que realitza una cerca en profunditat sobre el graf, amb la particularitat que registra els diferents components connexos del mateix, tot retornant una llista que conté les llistes de nodes que conformen cada component connex'''
    
    connectedComponentsNodes = []
    visitedNodes = []
    
    # cada crida a explorar sobre un node aquí retornarà una llista de nodes que formen un component connex, així com una versió actualitzada de la llista de nodes ja visitats
    for node in graph.nodes():
        if node not in visitedNodes:
            connectedComponentNodes, visitedNodes =  exploreGraphConnectivityFromNode(graph, node, visitedNodes)
            connectedComponentsNodes.append(connectedComponentNodes)
    
    return connectedComponentsNodes
    
   
    
def exploreGraphConnectivityFromNode(graph, node, visitedNodes):
    '''Funció que donat un graf, un node del mateix i una llista de nodes ja visitats, explora els diferents nodes encara no visitats als quals es pot arribar des del node passat per paràmetre, i retorna tant la llista de nodes explorats com la llista de nodes visitats actualitzada'''
    
    exploredNodes = [node]
    visitedNodes.append(node)
    
    for neighbor in graph.neighbors(node):
        if neighbor not in visitedNodes:
            nodes, visitedNodes = exploreGraphConnectivityFromNode(graph, neighbor, visitedNodes)
            
            for exploredNode in nodes:
                exploredNodes.append(exploredNode)
            
    return exploredNodes, visitedNodes
    
    
    
def isGraphNumberOfNodesEven(graph):
    '''Funció que comprova que el nombre de nodes d'un graf sigui parell'''
    
    return graph.number_of_nodes() % 2 == 0



def branchAndBound(graph):
    '''Funció central de l'exploració, ramificació i poda de les solucions de l'arbre de solucions: donat un graf, construeix una solució candidata inicial i a partir d'ella va generant filles i aplicant heurístiques per obtenir la solució òptima al problema de l'aparellament complet dels nodes del graf'''
    
    # la solució més òptima fins al moment comença sent nul·la
    optimumSolution = None
    
    # la cota superior parteix d'un valor infinit i s'actualitza com la cota inferior d'una solució òptima fins al moment un cop trobada
    superiorHeight = INFINITY
    
    # la solució candidata inicial, arrel de l'arbre de solucions, consisteix en un node, el que les heurístiques internes pronostiquen millor per a començar explorant
    initialPartialSolution = buildSolution(graph, [obtainBestInitialNode(graph)], 0, superiorHeight)
    
    # si es dóna la situació excepcional que es jutja la solució inicial com a inviable, es retorna un valor nul com a solució òptima
    if not isPartialSolutionViable(graph, initialPartialSolution, superiorHeight):
        return None
    
    # implementem una cua de prioritats que conté les solucions candidates com un heap binari, que inicialment conté la solució candidata inicial
    priorityQueue = []
    heapq.heappush(priorityQueue, (initialPartialSolution["inferiorHeight"], initialPartialSolution))
    
    # mentre la cua de prioritats no sigui buida, s'extreu el valor més prioritari (amb menor cota inferior) per ser analitzat primer
    while priorityQueue:
        partialSolution = heapq.heappop(priorityQueue)[QUEUE_ELEMENT_INDEX]
        
        # obtenim la llista de solucions filles d'una solució, tot examinant-les per ordre ascendent de cota inferior
        childSolutions = sorted(obtainAllChildSolutions(graph, partialSolution, superiorHeight), key = lambda child: child["inferiorHeight"])
           
        # per a cada solució filla 
        for childSolution in childSolutions:
            # si és completa i a més és millor que la solució òptima fins al moment, passa a ser la nova solució òptima fins al moment, i la seva cota inferior la nova cota superior
            if isSolutionComplete(graph, childSolution):
                if isSolutionBetterThanCurrentOptimum(childSolution, optimumSolution):
                    optimumSolution = childSolution
                    superiorHeight = optimumSolution["inferiorHeight"]
                    # quan s'actualitza la cota superior, s'elimina de la cua de prioritats tota solució candidata amb una cota inferior superior o igual a la cota superior
                    priorityQueue = obtainQueueWithOnlyViableSolutions(graph, priorityQueue, superiorHeight)
            
            # si la solució filla candidata no és completa, s'avalua si pot ser completada en una solució viable (entre d'altres coses si la seva cota inferior és menor a la superior): si és així, s'afegeix a la cua de prioritats perquè, quan arribi el moment, es puguin examinar les seves solucions filles        
            else:
                if isPartialSolutionViable(graph, childSolution, superiorHeight):
                    addSolutionToQueue(childSolution, priorityQueue)
      
    # retorn de la millor solució trobada, és a dir, de la solució òptima                   
    return optimumSolution



def obtainBestInitialNode(graph):
    '''Funció que retorna el millor node per començar a buscar la solució òptima de l'algorisme; formarà la solució parcial mare de la qual derivar tot l'arbre de solucions'''
    
    # considerem el millor node per començar l'arbre de solucions aquell que té el menor nombre de veïns
    return obtainNodeWithLessNeighbors(graph)
            
    
            
def obtainNodeWithLessNeighbors(graph, discardedNodes=[]):
    '''Funció que troba, d'entre tots els nodes del graf passat per paràmetre, quin dels que no pertanyen a la llista de nodes a ignorar passada per paràmetre és el que té menor grau, és a dir, menys veïns, i el retorna'''
    
    bestNode = None
    greatestRejection = 0
    bestNeighborCount = INFINITY
    
    for node in graph.nodes():
        if node not in discardedNodes:
            count = len(graph.neighbors(node))
            if count < bestNeighborCount:
                bestNode = node
                bestNeighborCount = count
                
    return bestNode



def buildSolution(graph, pairs, rejection, superiorHeight):
    '''Funció que a partir de diferents paràmetres constituents construeix i retorna una solució (parcial o completa) del problema; al nostre cas aquests paràmetres són una llista de parelles de nodes i un valor de rebuig; a partir d'un paràmetre addicional, el graf, calcula la cota inferior de la solució'''
    
    # han d'existir parelles (o almenys un node d'inici) per poder crear una solució
    if not pairs:
        return None

    # calculem la cota inferior de la solució que volem crear a partir dels seus paràmetres constitutius, les parelles explorades i el rebuig total acumulat per aquestes
    inferiorHeight = obtainInferiorHeight(graph, pairs, rejection)
    
    # si calculant la cota inferior ens trobem amb l'evidència que de la solució a construir no és completa i, com a solució parcial, és impossible que poguem derivar una solució completa vàlida, evitem crear la solució, retornem un valor buit; fem el mateix si la cota inferior de la solució a crear és major o igual a la cota inferior: volem evitar crear solucions candidates inútils que després s'haurien d'eliminar
    if inferiorHeight == INFINITY or inferiorHeight >= superiorHeight:
        return None
      
    # retorn de la solució candidata construïda  
    return {"pairs" : pairs, "rejection" : rejection, "inferiorHeight" : inferiorHeight}



def obtainInferiorHeight(graph, pairs, accumulatedRejection):
    '''Funció que, a partir de diferents paràmetres constituents d'una solució, calcula i retorna la cota inferior que ha de correspondre a una solució (parcial o completa) amb els paràmetres passats'''
    
     # Donat un conjunt de parelles de nodes i un rebuig total acumulat per elles, definim la cota inferior per a una solució que consti d'aquestes parelles i aquest rebuig com aquest rebuig acumulat més el rebuig de les n parelles amb menor rebuig d'entre totes les que són possibles al graf que no s'han utilitzat encara i que no contenen nodes de les ja explorades, sent n el nombre de parelles que falta afegir a la solució perquè pugui ser completa
    
    inferiorHeight = accumulatedRejection
    exploredNodes = []
    pairsRejections = []
    
    numberOfPairsToAdd = 0
    numberOfGraphNodes = graph.number_of_nodes()
    
    # esbrinem el nombre de parelles que falta afegir a la solució perquè sigui completa
    
    # cas en què la solució porta fins ara únicament un node, cap parella; cal trobar tantes parelles com la meitat de nodes del graf
    if isPairActuallyANode(pairs[0]):
        numberOfPairsToAdd = numberOfGraphNodes / 2     
       
    # situació a la qual sí que tenim parelles acumulades, no un sol node 
    else:
        # cas especial en què detectem que la solució ja és completa (té tots els nodes aparellats) i retornem com a cota inferior simplement el rebuig acumulat
        if (len(pairs) == numberOfGraphNodes / 2):
            return inferiorHeight
        
        # cal afegir tantes parelles com la meitat de nodes del graf menys les parelles ja comptabilitzades
        else:
            numberOfPairsToAdd = (numberOfGraphNodes / 2) - len(pairs)
            
            # registrem els nodes presents a les parelles ja explorades
            for pair in pairs:
                for node in pair:
                    if node not in exploredNodes:
                        exploredNodes.append(node)
            
    # per cada parella del graf no explorada encara i amb nodes no pertanyents a les ja explorades, guardem el seu rebuig
    for pair in graph.edges():
        if pair not in pairs:
            if pair[0] not in exploredNodes and pair[1] not in exploredNodes:
                pairsRejections.append(graph[pair[0]][pair[1]]["rejection"])
    
    # si només podem tenir en compte el rebuig d'un nombre de parelles inferior a les quals eventualment caldria afegir a les passades per paràmetre perquè pugui ser completada una solució, significa que de la solució parcial que es pot formar amb les parelles passades per paràmetre no es podria obtenir cap de completa: per descartar la creació d'aquesta solució parcial retornem un valor infinit com a cota inferior
    if len(pairsRejections) < numberOfPairsToAdd:
        return INFINITY
    
    else:
        # ordenem els rebuigs en ordre ascendent
        pairsRejections.sort()
    
        # afegim al rebuig inicial els menors rebuigs d'entre aquells que hem trobat de parelles no explorades: tants com parelles falta per tenir un conjunt complet de parelles del graf
        for i in range (0, numberOfPairsToAdd):
            inferiorHeight += pairsRejections[i]
            
        return inferiorHeight

    

def obtainAllChildSolutions(graph, partialSolution, superiorHeight):
    '''Funció que, donada una solució parcial al problema passada per paràmetre, retorna totes les solucions que es representarien com filles a l'arbre de solucions'''
    
    childSolutions = []
    pairs = []
    newChildPairs = []
    pair = ()
    solution = {}
    rejection = 0.0
    
    # si no portem explorada cap parella, només tenim un node d'inici, les solucions filles de l'actual consistiran en aquest node emparellat amb cadascun dels seus veïns (un veí per filla)
    if not doesSolutionHaveAnyPairs(partialSolution):
        
        initialNode = partialSolution["pairs"][0]
        
        if not initialNode:
            return []
        
        for neighbor in graph.neighbors(initialNode):
            if initialNode > neighbor:
                pair = (neighbor, initialNode)
            else:
                pair = (initialNode, neighbor)
                
            pairs = [pair]
            rejection = graph[initialNode][neighbor]["rejection"]
            solution = buildSolution(graph, pairs, rejection, superiorHeight)
            if solution != None:
                childSolutions.append(solution)
    
    # si la solució actual està formada per parelles, les solucions filles són el conjunt de parelles acumulat més una parella addicional, que consisteix en la parella del següent node a emparellar del graf amb un dels seus veïns (amb un veí diferent a cada solució filla)
    else:
        usedNodes = []
        for solutionPair in partialSolution["pairs"]:
            for node in solutionPair:
                if node not in usedNodes:
                    usedNodes.append(node)         
             
        # per raons d'eficiència (volem les mínimes ramificacions) el següent node a analitzar és aquell amb menor nombre de veïns dels que falta aparellar       
        node = obtainNodeWithLessNeighbors(graph, usedNodes)
                
        if node != None:
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
                            solution = buildSolution(graph, pairs, rejection, superiorHeight)
                            if solution != None:
                                childSolutions.append(solution) 
           
    return childSolutions



def isSolutionComplete(graph, solution):
    '''Funció que retorna si la solució passada per paràmetre és una solució completa al problema'''

    # Considerem una solució completa quan té tots els nodes del graf emparellats, és a dir, que hi ha tantes parelles com la meitat dels nodes. Com que els mecanismes per construir solucions ja s'encarreguen que no repetim nodes a les parelles, aquí no calen comprovacions addicionals
    
    return len(solution["pairs"]) == graph.number_of_nodes() / 2



def isSolutionBetterThanCurrentOptimum(solution, currentOptimumSolution):
    '''Funció que determina si la solució passada per paràmetre és millor que la que fins ara es considerava la millor trobada, passada també en qualitat de paràmetre'''
    
    # una solució completa qualsevol és millor que una entitat nul·la
    if currentOptimumSolution == None:
        return True
    
    # si la solució considera òptima fins al moment existeix, la candidata serà millor només si té menor rebuig total
    return solution["rejection"] < currentOptimumSolution["rejection"]



def isPartialSolutionViable(graph, solution, superiorHeight):
    '''Funció que determina, tenint en compte la cota superior actual, si una solució parcial és viable o per contra convé descartar-la a ella i a tota la seva hipotètica descendència a l'arbre de solucions'''
    
    # una solució és viable només si la seva cota inferior és inferior a la cota superior que el programa està fent servir
    return solution["inferiorHeight"] < superiorHeight and not hasSolutionAbandonedAnyNodes(graph, solution)
    
    
    
def doesSolutionHaveAnyPairs(solution):
    '''Funció que retorna si una solució (parcial o completa) està formada per parelles de nodes, o contràriament té únicament un node inicial, que serveix com a punt de partida per l'algorisme'''
    return not isPairActuallyANode(solution["pairs"][0])



def isPairActuallyANode(pair):
    '''Funció que retorna si una estructura que a priori es considera una parella és en realitat únicament un node (que es fa servir a l'inici de l'algorisme, com a solució parcial mare)'''
    return type(pair) != tuple
    
    
 
def hasSolutionAbandonedAnyNodes(graph, solution):
    '''Funció que retorna si, amb les parelles fins ara afegides a una solució passada per paràmetre ha quedat algun node del graf aïllat de tal manera que independentment de les parelles que es facin no podrà ser aparellat (els nodes amb els quals té rebuig no infinit ja estan aparellats)'''
    
    # si cap parella no ha estat afegida encara a la solució, no té sentit considerar nodes abandonats
    if not doesSolutionHaveAnyPairs(solution):
        return False
    
    # registrem els nodes ja aparellats
    exploredNodes = []    
    for pair in solution["pairs"]:
        for node in pair:
            if node not in exploredNodes:
                exploredNodes.append(node)
    
    # si un node que no ha estat aparellat no té veïns no aparellats, és que ha estat abandonat
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



def addSolutionToQueue(solution, priorityQueue):
    '''Funció que afegeix a la cua de prioritats una solució, tot col·locant-la al lloc que li pertoca d'acord a la seva prioritat'''
    
    # en aquest problema, la prioritat es regeix pel valor de rebuig acumulat de les solucions; com menor és per a una solució, més endavant està ubicada dins la cua
    
    heapq.heappush(priorityQueue, (solution["inferiorHeight"], solution))



def obtainQueueWithOnlyViableSolutions(graph, priorityQueue, superiorHeight):
    '''Funció que analitza les solucions la cua de prioritats i retorna una versió de la mateixa que conserva només les solucions viables'''

    newQueue = []
    
    index = -1
    
    # si tots els elements de la cua tenen cotes que no són inferiors a la superior, haurem de buidar la cua en la seva totalitat
    if priorityQueue[0][QUEUE_ELEMENT_INDEX]["inferiorHeight"] >= superiorHeight:
        index = 0
       
    # altrament, ens aproximem amb cerca binària a l'índex a partir del qual els elements de la cua tenen cotes inferiors que no són inferiors a la superior i per tant cal descartar
    else:
        min = 0
        max = len(priorityQueue) -1
        while index == -1:
            midpoint = (min + max) / 2
            height = priorityQueue[midpoint][QUEUE_ELEMENT_INDEX]["inferiorHeight"]
            if height == superiorHeight or abs(max-min) <= 1:
                index = midpoint
            elif superiorHeight < height:
                max = midpoint
            else:
                min = midpoint
         
    # a partir de l'índex aproximat, trobem el punt exacte a partir del qual les cotes inferiors de les solucions candidates de la cua no són inferiors a la cota superior  
    found = False
    while index < len(priorityQueue) and not found:
        if priorityQueue[index][QUEUE_PRIORITY_INDEX] >= superiorHeight:
            found = True
        else:
            index += 1
     
    # obtenim una cua només amb les solucions considerades viables, i la retornem 
    newQueue = priorityQueue[:index]
    
    return newQueue


    
    
if __name__ == "__main__":
    main(*sys.argv[1:])
