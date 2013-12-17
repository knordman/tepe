tepe
====

Please see the documentation here: http://knordman.github.com/tepe

TEPE is a lightweight physics engine. It particularly has a compile-time defined and somewhat minimal memory usage. It was designed to run as a GPU kernel, enabling numerous concurrent simulator instances to be set up (by for example using CUDA from Nvidia). TEPE is completely designed as a collection of inline functions in a set of header files. It uses the well known Projected Gauss-Seidel iterative solver. This ensures small memory requirements, and can be implemented so that GPU thread divergence is avoided, even when simulation worlds contain different amounts of constraints.

In its current state it is most suitable for simulating mobile walking robots. Currently the only supported constraint is the hinge joint which can have a motor attached. There is no general collision detection, instead there is a simple "foot collision", where certain bodies tagged as "feet" are collided against a plane.
