# Route Generation
This section explains how a route dataclass is generated using the strategy pattern.


## Route Generator
The RouteGenerator takes an ordered list of lanelet ids from start to goal as input and generates route instances for them.
The generator uses a route generation strategy using the strategy design pattern.
You can add a new strategy by inheriting from base strategy and choosing it as parameter when instantiating the route planner (or the route generator).

:::commonroad_route_planner.route_generator


***
## Route Generation Strategy Pattern
The route planner uses the strategy design pattern to allow a dynamic change (include own implementations) of the
route generation.
For this, inherite from the abstract BaseGenerationStrategy and use your own strategy class as an input argument
for either the RoutePlanner or the RouteGenerator.

### Base Generation Strategy
This is the abstract class to inherite from.
::: commonroad_route_planner.route_generation_strategies.base_generation_strategy


***
### Default Generation Strategy
::: commonroad_route_planner.route_generation_strategies.default_generation_strategy
