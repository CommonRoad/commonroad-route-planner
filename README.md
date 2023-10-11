# CommonRoad Route Planner

The basic functionality of this package is to twofold:
- find sequences of lanelets (also referred to as routes) that lead from the initial lanelet(s) to the goal lanelet(s) of a given planning problem. It also works with survival scenarios (where no goal region is specified).
- generate a reference path as a high-level behavior for each of the planned route, which can be used to construct a curvilinear coordinate system at a later stage.

### Supported Backends

The planner supports different backends to search for the shortest route in the scenario:

1. NETWORKX: uses built-in functions from the networkx package, tends to change lanes later

## Installation

This module uses python-poetry for installation. After installing poetry via pip-install, run in the root folder:

```bash
poetry install .
```
You can configure your python poetry environment in PyCharm similar to anaconda etc.

**Note that poetry uses your system python version number as default**

## Minimal Example

We provide a basic [tutorial](/test/test_example.py)
