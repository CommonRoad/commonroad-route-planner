# CommonRoad Route Planner

The basic functionality of this package is to find sequences of lanelets (also referred to as routes) that lead from the initial lanelet(s)  to the goal lanelet(s) of a given planning problem. It also works with survival scenarios (where no goal region is specified). In addition, the package generates a reference path for each of the planned route, which can be used to construct a curvilinear coordinate system at a later stage.

### Supported Backends
The planner supports different backends to search for the shortest route in the scenario:
1. NETWORKX: uses built-in functions from the networkx package, tends to change lanes later
2. NETWORKX_REVERSED: uses built-in functions from the networkx package, tends to change lanes earlier
3. PRIORITY_QUEUE: uses A-star search to find routes, lane change maneuver depends on the heuristic cost
## Installation

To use this module, run the setup script in the root folder:

```bash
pip install .
```
Or simply install the dependencies listed in `requirements.txt` and add this repository to your python path.
## Minimal Example
A tutorial notebook and an example script can be found under the `tutorial/` folder.
