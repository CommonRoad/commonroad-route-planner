# Changelog

## [2025.0.1]
### Changed
- Rename route-generator to path_generator
- Interface segregation of path_generator

### Deleted
- route_candidate_holder legacy interface


## [2024.2.1]
### Changed
- bugfix update_planning problem to empty previous planning problem initial state and goal


## [2024.2.0]
### Changed
- Route planner now uses lanelet_network instead of scenario as argument.
- Moved scenario as optional argument to be deprecated in future releases.
- Changed some logger warnings to debug



## [2024.1.1]
### Fixed
- CR Drivability Checker dependency set to major


## [2024.1.0]
### Changed
- Total refactor



## [2022.11]

### Changed

- Add support for commonroad-io 2022.3

## [2022.8]

### Changed

- In addition to Scenario, accepts LaneletNetwork as the road network.
- In addition to PlanningProblem, accepts State and GoalRegion as the route planning problem.

## [2022.1]

- First release of the toolbox.
