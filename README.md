# ADMM
With the help of the code of the paper "An Alternating Direction Method of Multiplier Based Problem Decomposition Scheme for Iteratively Improving Primal and Dual Solution Quality in Vehicle Routing Problem", it is applied to the VRPTW problem, joint optimization of honey peach picking and distribution.

## ADMM
 ADMM can be used to optimize a K-vehicles VRP problem from an infeasible solution to a feasible solution through iterations. It decomposes the whole large-scale problem into K independent VRP which can be easily solved by using K FDP.
 
 ## CalculateCostForVehicle
 This function is used to calculate primal cost for each vehicle in a single iteration. Input visitedSeq should be a [K, s] matrix. Output visitedNode is a [K, s+1] matrix. The (s+1)th column stores the total primal cost.
 
 ## FDP
 FDP is used to generate single vehicle least cost routing.
 
 ## FeasibleSolution
 Generate final visited sequence and time sequence for each vehicle. Suggest to use this function after program has been converged. When the result of the program is unfeasible, make it feasible.
 
 ## GeneralizedCostMatrix
 According to cost matrix C and other parameters, use objective function 2 to generate generalized cost matrix GC.
 
 ## GenerateFreshness
 According to the final visited sequence and corresponding time sequence, generate variable freshness for each vehicle. And calculate its freshness deviation.
 
 ## GenerateRandomTime
 Generate random delivery time in timeRange for each customers. Output matrix B is a (space nodes\*space nodes) matrix, including beginning node and end node.
