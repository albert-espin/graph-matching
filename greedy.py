#!/usr/bin/python
# -*- coding: utf-8 -*-


"""
Comentari general: l'algorisme de matching d'aquest fitxer prioritza que tots els nodes siguin connectats al fet d'obtenir una suma de pesos òptima, fins al punt que sempre que un algorisme qualsevol (exhaustiu, per exemple) pugui aconseguir emparellar tots els nodes (que no hi hagi un nombre senar de nodes ni nodes que són l'única connexió possible de més d'un node de grau 1), aquest algorisme també ho assolirà, tot i que el pes total de les arestes no sigui necessàriament òptim, tenint en compte el plantejament voraç de l'algorisme.

Nota general sobre l'anàlisi d'eficiència: V es refereix al nombre de vèrtexs o nodes del graf G sobre el que treballa la major part del programa, i E equival al seu nombre d'arestes.
"""


import networkx as nx
import sys


# DEFINICIÓ DE CONSTANTS (per fer més comprensible l'anàlisi del codi):

# valor enter equiparat a infinit que és emprat uniformement a les diferents funcions del programa
INFINITE = sys.maxint

# índex per accedir al node d'inici d'una aresta d'un graf de NetworkX
START_NODE_INDEX = 0

# índex per accedir al node final d'una aresta d'un graf de NetworkX
END_NODE_INDEX = 1

# índex per accedir al pes d'una aresta amb pesos d'un graf de NetworkX
WEIGHT_INDEX = 2


def main(entrada="exemple1.dat"): # Θ(V^2 + E) 
    '''Funció principal del programa, rep un nom de fitxer que pot ser fixat per consola que s'utilitza per llegir un graf representatiu d'un conjut de contactes relacionables sobre el qual es porta a terme un matching voraç'''
    G = llegir_graf(entrada) # Θ(V + E)
    agencia_de_contactes(G) # Θ(V^2) 
    
    
def llegir_graf(entrada): # Θ(V + E)
    filename = raw_input(u"Dóna el nom del fitxer del graf,\no prem <Enter> per usar el valor de consola: ") # Θ(1)  
    if filename == "": # Θ(1) +
        filename = entrada # ( Θ(1) )  
    G = nx.read_edgelist(filename, nodetype=int, data=(("weight",float),)) # Θ(V + E)  
    return G # Θ(1)   


def agencia_de_contactes(G): # Θ(V^2) 
    '''Funció que rep per paràmetre un graf els nodes del qual interpreta com a contactes, les arestes com a possibles connexions entre ells amb un índex de rebuig com a pes; duu a terme un matching voraç del qual s'obté un conjunt disjunt de parelles de contactes, que es mostren seguint l'ordre ascendent dels nombres associats als nodes de les parelles
    
    Comentari sobre l'eficiència: fixem-nos que aquest algorisme té complexitat Θ(V^2), tot i que executa V vegades la funció "obtainPriorityEdge", que ella mateixa té una complexitat base de Θ(V^2). Com s'explica llavors que aquest mètode no tingui Θ(V^3)? Per la senzilla raó que a la primera execució de "obtainPriorityEdge", el pitjor cas serà efectivament Θ(V^2), però com anem eliminant nodes a cada iteració de "agencia_de_contactes", el pitjor cas (tots els nodes connectats amb tots) deixa d'existir: com anem eliminant nodes, no podem tenir tots els nodes inicials connectats amb tots a cada iteració, de manera que la complexitat mitja a "agencia_de_contactes" de "obtainPriorityEdge" és Θ(V*log(V)), raó per la qual l'algorisme "agencia_de_contactes" té complexitat Θ((V^2)*log(V)), que simplifiquem com Θ(V^2).'''
    contactEdges = list() # Θ(1)
    
    # tantes vegades com la meitat truncada dels nodes del graf obtenim l'aresta més prioritària segons l'heurística del programa, i l'afegim voraçment al conjunt de solucions, a més d'eliminar del graf els nodes de l'aresta (i per extensió l'aresta), de manera que a la següent iteració l'aresta prioritària serà una altra
    for i in range(len(G.nodes()) / 2): # Θ(V/2) * [
        priorityEdge = obtainPriorityEdge(G) # # Θ(V*log(V))  
        if not priorityEdge: # Θ(1) +
            break # ( Θ(1) ) +
        contactEdges.append([priorityEdge[START_NODE_INDEX], priorityEdge[END_NODE_INDEX]]) # Θ(1) + 
        G.remove_node(priorityEdge[START_NODE_INDEX]) # Θ(1) +
        G.remove_node(priorityEdge[END_NODE_INDEX]) # Θ(1) ] 
      
    # mostra ordenada de la solució al problema, les parelles obtingudes  
    print "\n{} parelles de contactes: {}".format(len(contactEdges), sorted(contactEdges)) # Θ(V*log(V))   
    
    
def obtainPriorityEdge(G): # Θ(V^2)
    '''Donat un graf, retorna l'aresta d'aquest considerada més prioritària seguint el criteri següent: si hi ha al graf nodes de grau 1, l'aresta prioritària serà la de menor pes dels nodes amb una sola aresta; si no hi ha nodes de grau 1, l'aresta prioritària serà aquella que tingui menor pes de tot el graf'''
    priorityNode = None # Θ(1)
    priorityEdge = None # Θ(1)
    
    # analitzem cada node del graf
    for node in G.nodes(): # Θ(V) * [
        edge = None # Θ(1) +
        
        # si el node és aïllat, sense cap connexió, no ens interessa: és evident que no ens permet trobar una aresta prioritària
        if G.neighbors(node) != 0: # Θ(1) +
        
            # a la primera iteració, establim com a prioritària l'aresta de menor pes del primer node, i aquest com a node prioritari
            if not priorityNode: # ( Θ(1) +
                priorityNode = node  # ( Θ(1) +
                priorityEdge = obtainShortestEdge(G, node) # Θ(V) ) OR
            
            # si el node prioritari actual té un grau major que 1
            elif len(G.neighbors(priorityNode)) > 1:  # Θ(1) +
                # trobem l'aresta de menor pes del node de la iteració actual
                edge = obtainShortestEdge(G, node) # ( Θ(V) ) +
                
                # si el node de la iteració actual té grau 1, com el prioritari no en tenia el node de la iteració actual passa a ser el prioritari, i la seva aresta de menor pes l'aresta prioritària
                if len(G.neighbors(node)) == 1:  # Θ(1) +
                    priorityNode = node # ( Θ(1) +
                    priorityEdge = edge # Θ(1) ) OR
                
                # si per contra el node de la iteració actual té un grau major que 1, com el prioritari actual també només passa a ser node prioritari si la seva aresta de menor pes és menor en pes que l'aresta prioritària actual, cas en què passa a ser l'aresta prioritària
                elif len(G.neighbors(node)) > 1: # Θ(1) +
                    if edge[WEIGHT_INDEX] < priorityEdge[WEIGHT_INDEX]: # ( Θ(1) +
                        priorityNode = node # ( Θ(1) +
                        priorityEdge = edge # Θ(1) ) ) OR              
            
            # si el node prioritari actual té grau 1, només pot ser substituït per el node de la iteració actual si aquest també té grau 1 i la seva aresta de menor pes és menor en pes que l'aresta prioritària actual, cas en què passa a ser l'aresta prioritària        
            elif len(G.neighbors(priorityNode)) == 1: # Θ(1) +
                if len(G.neighbors(node)) == 1: # ( Θ(1) +
                    edge = obtainShortestEdge(G, node) # ( Θ(1) +
                    if edge[WEIGHT_INDEX] < priorityEdge[WEIGHT_INDEX]: # Θ(1) +
                        priorityNode = node # ( Θ(1) +
                        priorityEdge = edge #  # Θ(1) ) ) ) ) ]

    return priorityEdge # Θ(1)


def obtainShortestEdge(G, node): # Θ(V)
    '''Donat un graf i un node que li pertany, retorna l'aresta de menor pes d'entre totes les que connecten el node'''
    shortestNeighbor = None # Θ(1)
    shortestEdge = None # Θ(1)
    startNode = INFINITE # Θ(1)
    endNode = INFINITE # Θ(1)
    
    for neighbor in G.neighbors(node): # Θ(V) * [
        if shortestEdge == None or G[node][neighbor]["weight"] < shortestEdge[WEIGHT_INDEX]:  # Θ(2) +
            shortestNeighbor = neighbor # ( Θ(1) +
            if node < shortestNeighbor:  # Θ(1) +
                startNode = node # ( Θ(1) +
                endNode = shortestNeighbor # Θ(1) ) OR
            else: # Θ(1) +
                startNode = shortestNeighbor # ( Θ(1) +
                endNode = node   # Θ(1) ) +
            shortestEdge = (startNode, endNode, G[startNode][endNode]) # Θ(1) ) )
           
    return shortestEdge # Θ(1)



# Aquest troç és l'encarregat de cridar la funció main amb els paràmetres introduïts per consola.
if __name__ == '__main__':
    main(*sys.argv[1:])
