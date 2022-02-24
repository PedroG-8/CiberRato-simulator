# RMI assignment 2 - CiberRato Simulator
## Program to command a robotic agent through an unknown maze, print the maze and the shortest path through all beacons in a file

- Localization: The agent needs to navigate and localize itself in an unknown maze, using
the movement model, the distance to obstacles and the compass. GPS is not available.
Motors, distance sensors and compass are noisy. Collisions with walls will be penalized.

- Mapping: The agent needs to explore an unknown maze in order to completely map
its navigable cells. At the same time, the agent needs to localize target spots placed in
the maze. The number of target spots may change between mazes. After completing the
mapping task, the agent should return to the starting spot.

- Planning: The agent needs to compute a closed path with minimal cost that allows to
visit all target spots, starting and ending in the starting spot.

Final grade: 16.9/20

Developed with:

- [Pedro Silva](https://github.com/pedromsilva99)
