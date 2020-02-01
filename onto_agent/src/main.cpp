#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include <curses.h>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include <ontology_svcs/GetAllClassInstancesWithPoses.h>
#include <ontology_msgs/PropertyChanged.h>
#include <ontology_msgs/ObjUpdate.h>

#include "utils.hpp"
#include "Agent.hpp"

using namespace json_prolog;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "onto_agent_node");

  try {
    if (argc > 1)
    {
      Agent agent(parseActivities(argv[1]));
      agent.run();
    }
    else
    {
      Agent agent;
      agent.run();
    }
  }
  catch (std::exception& e)
  {
    std::cerr << std::endl << "\tFatal Error: " << std::endl
              << "\t\t" << e.what() << std::endl
              << "\tShutting Down" << std::endl;
  }
  catch (...)
  {
    std::cerr << "\tFatal Error. Shutting Down..." << std::endl;
  }

  return 0;
}
