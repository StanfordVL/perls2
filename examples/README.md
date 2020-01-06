# perls2 Examples
PErception and Robotic Learning System v2

The following examples are provided to introduce the perls2 framework and highlight its capabilities. Examples in simulation may be demonstrated using the run_<\example\>.py. Examples for real robots require a little more instruction.

**Please consult the ReadME.MD in the corresponding folder.**



## Available Examples
### Reach task with joint position: simple_reach

	A sample reach task using an analytical policy. Observations consist of the delta ee position from the goal object. Actions are interpreted as delta from the current end-effector position. Camera parameters and object parameters are randomized on reset.

### Simple Reach switch between simulation and real.
	An example to show to to share logic between simulation and real. The simulation reaches for an apple. The real task reaches for a randomly generated position.
	This example allows for switching between a Bullet Sawyer and Real Sawyer

