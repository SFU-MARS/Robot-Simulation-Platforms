"""
ROS-free navigation core shared by every robot and simulator.

Layering: robots (profiles, units) <- mapping (occupancy grid, boundary edges)
<- nominal (performance controllers).  HJ values, dynamics and safety filters
live in the FaST-BRT submodule; this package never imports rclpy.
"""
