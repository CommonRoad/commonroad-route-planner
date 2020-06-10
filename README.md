# commonroad-route-planner

This repository hosts the code base of a commonly used route planner.
It plans a route on the lanelets going in the same direction.
It also works on scenarios, where no goal region is specified.

Required modules (TODO):
- commonroad-io
- commonroad-collision-checker - for visualization purposes
- PyYaml
- matplotlib
- fnmatch
- networkx

Two available backends:
- networkx
- priority queue with graph based A*

The second one is a slightly faster but needs further analysis.

Possible improvements:
- use redefine the cost of a lanelet using time approach (eg.: time_cost = lanelet_length / max_velocity)
