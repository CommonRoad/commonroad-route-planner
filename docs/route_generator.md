## Summary
The RouteGenerator takes an ordered list of lanelet ids from start to goal as input and generates route instances for them.
The generator uses a route generation strategy using the strategy design pattern.
You can add a new strategy by inheriting from base strategy and choosing it as parameter when instantiating the route planner (or the route generator).

::: commonroad_route_planner.route_generator
