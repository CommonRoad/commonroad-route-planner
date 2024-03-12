# Lane Changing
This section explains the lane changing via the different handlers and lane changing methods.

## Known Issues for Future Releases
For future releases, we aim to provide a sampling based lane change with
polynoms and given cost functions so that lane changes over long lanelets do not become to long

## Lane Changing Handlers
A bit similar to the route generation strategy, there are also different strategies for lane changing.
The LaneChangeHandler computes the reference path during a lane change, the LaneChangePositionHandler handles where
to do the lane change and the LaneChangeInstruction saves the instruction.

***
### Lane Change Handler
::: commonroad_route_planner.lane_changing.lane_change_handler

***
### Lane Change Position Handler
::: commonroad_route_planner.lane_changing.change_position

***
### Lane Change Instruction
::: commonroad_route_planner.lane_changing.change_instruction


***
## Lane Changing Methods
A bit similar to the route generation strategy, there are also different strategies for lane changing.
The MethodInterface defines the interface to these lane change methods. To get at least C3-continuity, we currently
provide cubig and quintic splines.


***
### Lane Change Method Interface
::: commonroad_route_planner.lane_changing.lane_change_methods.method_interface

***
### Lane Change Methods
Currently, the following lane change methods are supported.

***
### Polynomial Interpolation
:::commonroad_route_planner.lane_changing.lane_change_methods.polynomial_change
