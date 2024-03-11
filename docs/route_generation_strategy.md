## Summary
The route planner uses the strategy design pattern to allow a dynamic change (include own implementations) of the 
route generation. 
For this, inherite from the abstract BaseGenerationStrategy and use your own strategy class as an input argument
for either the RoutePlanner or the RouteGenerator

::: commonroad_route_planner.route_generation_strategies.base_generation_strategy
::: commonroad_route_planner.route_generation_strategies.default_generation_strategy

