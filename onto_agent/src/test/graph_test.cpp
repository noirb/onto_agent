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

GraphNode<std::string>::Ptr fillGraph(Graph<std::string>& g, std::string rootClass, Ontology& onto)
{
  auto root_node = g.find(rootClass);
  if (root_node == nullptr)
   root_node = g.add_node(rootClass);

  auto subclasses = onto.getDirectSubclassesOfClass(rootClass);

  for (auto& s : subclasses)
  {
    auto node = g.find(s);
    if (node == nullptr)
    {
      node = fillGraph(g, s, onto);
    }
    g.add_edge(node, root_node);
  }
  return root_node;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KnowRob_GraphTest");

  Ontology onto;
  Graph<std::string> graph;
  auto root = fillGraph(graph, "#SpatialThing", onto);

  printTree<std::string>(graph.depth_first_tree(root));

  ROS_INFO("Advertising graph topic...");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ontology_msgs::Graph>("/sgv/onto/graph", 5);
  ontology_msgs::Graph graph_msg = graph.toMessage();

  ros::Rate loop_rate(0.1);
  while (ros::ok())
  {
    ROS_INFO("Publishing graph...");
    pub.publish(graph_msg);
    ros::spinOnce();
    loop_rate.sleep();
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
