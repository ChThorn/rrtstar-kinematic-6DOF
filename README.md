## FK and IK test using Google Test

This repos is testing the FK and IK for manipulator 6DOF.

## RRTSTar algorithm

The implemenation is also considered test the RRT* algorithm to deal the path planning and later on will be check.

`Testing seperately of FK with IK` and `Testing the RRT* algorithm processing`.

The combination will be implemented later...

There are two implementation in the source codes for RRT*.

# Performance Comparison

The table below compares the performance characteristics between the original and modified code implementations.

| Aspect | Original Code | Modified Code |
|--------|--------------|--------------|
| Path Smoothness | Basic cubic splines | Quintic splines + velocity constraints |
| Dynamic Feasibility | No velocity/jerk limits | Explicit velocity/jerk constraints |
| IK Accuracy | Placeholder (2-link planar arm) | Numerical IK with damped least squares |
| Collision Checking | Simple interpolated checks | FK-based checks at key points |
| Cost Function | Distance-only | Distance + angular change + jerk |
| Memory Safety | Potential leaks during optimization | Ownership checks in `optimizePath` |
| Computational Load | Lightweight | Higher (due to splines and numerical IK) |

## Implementation Notes

The modified code introduces several improvements over the original implementation, focusing on trajectory quality and runtime safety. While the computational load has increased, the benefits in path smoothness, dynamic feasibility, and collision avoidance justify the additional processing requirements.

## Usage Guidelines

When using the modified implementation, be aware of the higher computational demands, especially for real-time applications. The enhanced features are particularly beneficial for robotic systems that require precise motion control and safety guarantees.