#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include <curses.h>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include <ontology_svcs/GetAllClassInstancesWithPoses.h>
#include <ontology_msgs/Graph.h>
#include <ontology_msgs/GraphUpdate.h>
#include <ontology_msgs/PropertyChanged.h>
#include <ontology_msgs/ObjUpdate.h>

#include "Ontology.hpp"
#include "Graph.hpp"

using namespace json_prolog;

template<typename T>
void printEdge(typename GraphEdge<T>::Ptr edge)
{
  std::cout << "\t" + std::to_string(edge->source->data) + " --> " <<
                      std::to_string(edge->destination->data)      <<
               " (w: " << edge->weight << ")"                      << std::endl;
}

template<typename T>
void printTree(std::vector<typename GraphNode<T>::Ptr> tree)
{
  for (size_t i = 0; i < tree.size(); i++)
  {
    std::cout << tree[i]->data;
    if (i < tree.size() - 1)
      std::cout << " -> ";
    else
      std::cout << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KnowRob_GraphUpdateTest");

  Graph<std::string> graph;
  auto root = graph.add_node("#start");
  auto last = root;

  ROS_INFO("Advertising graph topic...");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ontology_msgs::Graph>("/sgv/onto/graph", 1, true);
  ros::Publisher updpub = nh.advertise<ontology_msgs::GraphUpdate>("/sgv/onto/graph/updates", 5);

  ontology_msgs::Graph graph_msg = graph.toMessage();

  ROS_INFO("Publishing graph initial state...");
  ros::spinOnce();
  pub.publish(graph_msg);

  int i = 0;
  ros::Rate loop_rate(0.8);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    
    ROS_INFO("Adding Node to Graph");
    auto curr = graph.add_node("#" + std::to_string(++i));
    graph.add_edge(last, curr);
    graph.add_edge(curr, root);

    ROS_INFO("Sending update for new node addition...");
    ontology_msgs::GraphUpdate addMsg;
    addMsg.operation = ontology_msgs::GraphUpdate::ADD;
    addMsg.node.name = curr->data;
    for (auto l : curr->links)
    {
      addMsg.node.neighbors.push_back(l->destination->data);
    }
    updpub.publish(addMsg);

    ROS_INFO("Sending update for new edge to new node...");
    ontology_msgs::GraphUpdate updMsg;
    updMsg.operation = ontology_msgs::GraphUpdate::CHANGE;
    updMsg.node.name = last->data;
    updMsg.node.neighbors.clear();
    for (auto l : last->links)
    {
      updMsg.node.neighbors.push_back(l->destination->data);
    }
    updpub.publish(updMsg);

    last = curr;
  }


  /*Graph<std::string> g;
  printTree<int>(g.depth_first_tree(n1));
  printTree<int>(g.depth_first_tree(n2));
  printTree<int>(g.depth_first_tree(n3));

  dijk = g.dijkstra(n3);
  std::cout << "Distances from n3:" << std::endl;
  for (auto e : dijk)
  {
    std::cout << "\t" << std::to_string(e.first->data) << " : " << e.second << std::endl;
  }
*/
  return 0;
}
