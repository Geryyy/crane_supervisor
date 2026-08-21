# crane_supervisor

First-slice ROS-free shell for W08. It installs no C++ headers, libraries, or
runtime nodes; the mode/fault fixture is private to the package tests and is
not a safety implementation. ROS services, controller-manager switching,
timing/staleness policy, working-cell geometry, and hardware stop integration
are non-goals in this shell. It has no dependency on vendor hardware or
retained controllers.
