# Integration Check

`pp_core` is used as the fixed-lookahead baseline in the shared benchmark workflow.

Key interface assumptions:

- subscribes to `/ego_racecar/odom`
- publishes `/drive`
- consumes a raceline CSV with waypoint coordinates and target speed
