CSC 4444: Artificial Intelligence

Implementing A* Search (tree search) in python 3.10 given a scenario. 

___________________________________________________________________________________________________________________________________________________________________________________
Here we have a vacuum-cleaning agent that can sense the environment and perform actions to move around and vacuum/clean dirty squares.
- We assume a 5x5 grid world known to the agent. The environment is fully observable: the percepts give complete information about the dirty/clean status of every square and the agent's location.
- The environment is deterministic: A clean square remains clean and a dirty square remains dirty unless the agent cleans it up.
- The actions available for the agent are: Left, Right, Down, Up, Suck. Each action takes place in one time "step".
- Each of the actions will incur a step of 1.
- At the end of each time step, each remaining dirty square will incur a cost of 2.
- ___________________________________________________________________________________________________________________________________________________________________________________

Christian Cox
