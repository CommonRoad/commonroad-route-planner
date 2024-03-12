# CommonRoad Route Planner

The basic functionality of this package is to twofold:
- find sequences of lanelets (also referred to as routes) that lead from the initial lanelet(s) to the goal lanelet(s) of a given planning problem. It also works with survival scenarios (where no goal region is specified).
- generate a reference path as a high-level behavior for each of the planned route, which can be used to construct a curvilinear coordinate system at a later stage.

### Supported Backends

The planner supports different backends to search for the shortest route in the scenario:

1. NETWORKX: uses built-in functions from the networkx package, tends to change lanes later


## Using the scenario files with git.lsf
This repo contains a number of scenario xml files in [/scenarios](/scenarios/). They are stored with git large-file-system (git lfs).
If you want to download these scenario files, before cloning this repo, do the following:

1. install git-lsf via `sudo apt-get install -y git-lfs` and `git lfs install`
2. use `git config --global credential.helper store`
3. clone repo


## Installation


#### Poetry
This module uses python-poetry for installation. After installing poetry via pip-install, run in the root folder:

```bash
poetry install .
```
You can configure your python poetry environment in PyCharm similar to anaconda etc.

#### Conda + Poetry
Alternatively, you can just create a conda environment, install poetry in it and use the install command from above inside the conda environment.
This way, you don't have to deal with conflicting python versions. However, contrary to just using plain poetry, you have to chose the conda
environment as your interpreter in your IDE.



**Note that poetry uses your system python version number as default**

## Minimal Example

We provide a basic [tutorial](/test/test_example.py)
