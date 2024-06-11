# Solutions to Travelling Salesperson Problem

Given an input of vertices, [drone.cpp](https://github.com/micdwill/TSP-Heuristic-and-Optimization/blob/master/drone.cpp) contains an optimal 
solution based on Euclidean distance for an MST, a heursitic for a TSP, and an optimal solution for a TSP. Input file format is as follows:

```markdown
[# of vertices]  
[x component integer] [y component integer]  
[x component integer] [y component integer]  
.  
.  
.  
[x component integer] [y component integer]
```

Then the command line must include -m [mode] or --mode [mode] with valid modes being MST for the minimum spanning tree solution, FASTTSP for the travelling salesperson heuristic solution, and OPTTSP for the optimal travelling salesperson solution.

## Minimum Spanning Tree Mode

This MST has been implemented with the rule that a line can not pass from the 3rd quadrant to the 1st, 2nd, or 3rd quadrant without first going through the border (i.e. (0, -z) or (-z, 0) where z is a non-negative integer). This solution uses [Prim's Algorithm](https://en.wikipedia.org/wiki/Prim%27s_algorithm).

## Fast Travelling Salesperson Problem Mode

This heuristic uses [arbitrary insertion](https://www2.isye.gatech.edu/~mgoetsch/cali/VEHICLE/TSP/TSP013__.HTM) followed by [2-opt](https://en.wikipedia.org/wiki/2-opt#:~:text=The%20main%20idea%20behind%20it,well%20as%20many%20related%20problems.) for an n-squared implementation. It should be noted that the second for loop in the 2-opt does not iterate through every vertex, only the first 30. This largely improves complexity.

## Optimal Travelling Salesperson Problem Mode

This is a branch and bound implementation. First, the above heuristic is used to find a lower bound. Then as we iterate through all n! permutations, for a given permutation of n - k vertices, an MST of the remaning K vertices is calculated. If the path so far + the size of this MST is not strictly less than our best path, we prune this branch.
