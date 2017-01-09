# Startup

**Terminal 1 (yarp)** 

```
yarpserver --write
```


**Terminal 2 (gazebo)**

```
gazebo Code/bayesian-task-optimization/gazebo_worlds/balancing.world
```


**Terminal 3 (controller-server)**

```
ocra-icub-server --floatingBase --taskSet Code/bayesian-task-optimization/reaching-task-sets/icubGazeboSim/TaskOptimizationTaskSet.xml --absolutePath
```

**Terminal 4 (controller-client)**

*original*
```
reach-client --rightHandWptFile Code/bayesian-task-optimization/rightHandWaypoints.txt --comWptFile Code/bayesian-task-optimization/comWaypoints.txt --home
```

*optimal*
```
reach-client --rightHandWptFile Code/bayesian-task-optimization/rightHandWaypoints.txt --comWptFile Code/bayesian-task-optimization/comWaypoints_optimal.txt --home
```

*NOTE: Add `--save` to save client data during execution.*



