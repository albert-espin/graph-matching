# Optimal Weighted Graph Matching

Program to find optimal pair matches in graphs, i.e. the set of pairs that minimizes the sum of edge costs, while all the nodes get exclusive connections. Files are described in ".dat" files (specifying all node connections and their costs), with some examples included. The problem is solved using branch and bound with a series of optimality-preserving heuristics; run "branch_bound.py" to try it. Additionally, a set of simpler heuristics is included in "simple_heuristics.py", while a faster greedy solver (that cannot guarantee optimality) can be found in "greedy.py".


| | | |
|-|-|-|
| **Programming language**  | Python 2 |
| **Language**   | English (variables and functions), Catalan (comments) |
| **Author** | Albert Espín |
| **Date**  | Q4 2015  |
| **License**  | MIT |
