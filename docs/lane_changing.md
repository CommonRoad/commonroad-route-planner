## Summary
A bit similar to the route generation strategy, there are also different strategies for lane changing.
The LaneChangeHandler computes the reference path during a lane change, the LaneChangePositionHandler handles where
to do the lane change and the LaneChangeInstruction saves the instruction (needed e.g. for chaikin's corner cutting).
The MethodInterface defines the interface to these lane change methods


::: commonroad_route_planner.lane_changing.lane_change_handler
::: commonroad_route_planner.lane_changing.change_position
::: commonroad_route_planner.lane_changing.change_instruction
::: commonroad_route_planner.lane_changing.lane_change_methods.method_interface

