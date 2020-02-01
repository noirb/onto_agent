# Onto Agent

This project contains an ontology-based action learning and recognition system.

### Dependencies:

* [ROS](http://wiki.ros.org/) (tested & working well on [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) and Kinetic)
* [KnowROB](http://www.knowrob.org/)
* [gencs](https://github.com/noirb/gencs)
* [ontology_msgs](../ontology_msgs)
* [ontology_svcs](../ontology_svcs)
* [household-scenarios](https://github.com/noirb/HouseholdScenarios)
* [OPTIONAL] [onto-vis](https://github.com/noirb/onto-vis)

### Usage:

Once the above dependencies are all set up in your ROS workspace (with the exception of `household-scenarios` and `onto-vis`, which do not require ROS), you should be able to build everything with `catkin_make`.

#### To run the learning agent:

```bash
roslaunch onto_agent onto_agent.prereqs.launch
rosrun onto_agent onto_agent
```

#### Other targets:

```bash
rosrun onto_agent knowrob_graph_test_suite
```

Executes a collection of automated test cases to verify correct operation of the graph in `Graph.hpp`

```bash
rosrun onto_agent knowrob_graph_test_updates
```

Executes a test for publishing changes to a Graph via ROS messages

