# CommonRoad Route Planner
The CommonRoad Route Planner aims to give an extendable, light-weight route- and reference path planner for the CommonRoad project
This tool is built for and Ubuntu 22.04 and Python3 ^3.8.

## Documentation
The documentation can be found [here](https://cps.pages.gitlab.lrz.de/commonroad-route-planner/)


## Project status
This tool is still under development but has already been successfully deployed in both simulation and on real-world 
autonomous vehicles.

We highly welcome your contribution.

## Installation and Usage
We recommend to use PyCharm (Professional) as IDE.

### Usage in other projects
We provide an PyPI package which can be installed with the following command
```shell
pip install commonroad-route-planner
```

### Development
It is recommended to use [poetry](https://python-poetry.org/) as an environment manager.
Clone the repository and install it with poetry. Currently, only the internal repository can be cloned.
This repo contains a number of scenario xml files in /scenarios. They are stored with git large-file-system (git lfs).
If you want to download these scenario files, before cloning this repo, do the following:
```shell
sudo apt-get install -y git-lfs and git lfs install
git config --global credential.helper store
git clone https://gitlab.lrz.de/cps/commonroad-route-planner.git
conda activate ENVIRONMENT
pip install .
```

### Examples
We recommend to use PyCharm (Professional) as IDE.
Examples can be found on our CommonRoad website


## Documentation
You can generate the documentation within your activated Poetry environment using.
```bash
conda activate ENVIRONMENT
mkdocs build
```
The documentation will be located under site, where you can open `index.html` in your browser to view it.
For updating the documentation you can also use the live preview:
```bash
conda activate ENVIRONMENT
mkdocs serve
```

## Authors
Responsible: Tobias Mascetta, tobias.mascetta[at]tum.de




