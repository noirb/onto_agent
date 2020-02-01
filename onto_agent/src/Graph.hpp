#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <algorithm>
#include <queue>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ontology_msgs/Graph.h>
#include <ontology_msgs/GraphNode.h>
#include <ontology_msgs/GraphUpdate.h>

#include "Ontology.hpp"
#include "Activity.hpp"

// forward declarations
template<typename T>
class GraphNode;

template<typename T>
class GraphEdge;

template<typename T>
class Graph;

template<typename T>
class GraphNode
{
public:
  T data;
  std::vector<std::shared_ptr<GraphEdge<T> > > links;

  typedef std::shared_ptr<GraphNode<T> > Ptr;

  GraphNode(T val) : data(val)
  {}

  size_t outdegree() const
  {
    return links.size();
  }

};

// all edges are one-way
// bidirectional edges are represented with two symmetric edges
template<typename T>
class GraphEdge
{
public:
  double weight = 1.0;
  typename GraphNode<T>::Ptr source;
  typename GraphNode<T>::Ptr destination;

  typedef std::shared_ptr<GraphEdge<T> > Ptr;

  GraphEdge(typename GraphNode<T>::Ptr Source, typename GraphNode<T>::Ptr Destination)
  {
    this->source = Source;
    this->destination = Destination;
  }

  GraphEdge(typename GraphNode<T>::Ptr Source, typename GraphNode<T>::Ptr Destination, double Weight)
  {
    this->source = Source;
    this->destination = Destination;
    this->weight = Weight;
  }
};

template<typename T>
static ontology_msgs::GraphNode nodeMessage(typename GraphNode<T>::Ptr node)
{
  ontology_msgs::GraphNode msg;
  msg.name = "Unprintable Node Type";
  return msg;
}

template<>
ontology_msgs::GraphNode nodeMessage<std::string>(typename GraphNode<std::string>::Ptr node)
{
  ontology_msgs::GraphNode msg;
  msg.name = node->data;
  for (auto l : node->links)
  {
    msg.neighbors.push_back(l->destination->data);
    msg.weights.push_back(l->weight);
  }
  return msg;
}

template<>
ontology_msgs::GraphNode nodeMessage<int>(typename GraphNode<int>::Ptr node)
{
  ontology_msgs::GraphNode msg;
  msg.name = std::to_string(node->data);
  for (auto l : node->links)
  {
    msg.neighbors.push_back(std::to_string(l->destination->data));
    msg.weights.push_back(l->weight);
  }
  return msg;
}

template<>
ontology_msgs::GraphNode nodeMessage<Activity&>(typename GraphNode<Activity&>::Ptr node)
{
  ontology_msgs::GraphNode msg;
  msg.name = node->data.name;
  for (auto l : node->links)
  {
    msg.neighbors.push_back(l->destination->data.name);
    msg.weights.push_back(l->weight);
  }
  return msg;
}

template<typename T>
class Graph
{
  std::vector<typename GraphNode<T>::Ptr> graph;
  ros::Publisher* update_pub = nullptr;

public:
  Graph() {}
  Graph(ros::Publisher* updater) : update_pub(updater) {}

  size_t size() const
  {
    return graph.size();
  }

  bool adjacent(typename GraphNode<T>::Ptr n1, typename GraphNode<T>::Ptr n2) const
  {
    return false;
  }

  std::vector<typename GraphNode<T>::Ptr> neighbors(typename GraphNode<T>::Ptr n) const
  {
    std::vector<typename GraphNode<T>::Ptr> result;
    for (auto l : n->links)
    {
      if (l->source == n)
        result.push_back(l->destination);
    }
    return result;
  }
  
  void add_node(typename GraphNode<T>::Ptr n)
  {
    graph.push_back(n);

    publish_update(n, ontology_msgs::GraphUpdate::ADD);
  }

  typename GraphNode<T>::Ptr add_node(T value)
  {
    typename GraphNode<T>::Ptr node(new GraphNode<T>(value));
    add_node(node);
    return node;
  }

  void remove_node(typename GraphNode<T>::Ptr n)
  {
    auto node = std::find(graph.begin(), graph.end(), n);
    if (node != graph.end())
      graph.erase(node);

    publish_update(n, ontology_msgs::GraphUpdate::REMOVE);
  }

  void add_edge(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination)
  {
    // only add new edge if it doesn't already exist
    if (has_edge(source, destination))
      return;

    source->links.push_back(typename GraphEdge<T>::Ptr(new GraphEdge<T>(source, destination)));

    publish_update(source, ontology_msgs::GraphUpdate::CHANGE);
  }

  void add_edge(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination, double weight)
  {
    // only add new edge if it doesn't already exist
    if (has_edge(source, destination))
      return;
    source->links.push_back(typename GraphEdge<T>::Ptr(new GraphEdge<T>(source, destination, weight)));

    publish_update(source, ontology_msgs::GraphUpdate::CHANGE);
  }

  void remove_edge(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination)
  {
    for (size_t i = 0; i < source->links.size(); i++)
    {
      if (source->links[i]->destination == destination)
      {
        source->links.erase(source->links.begin() + i);
        break;
      }
    }

    publish_update(source, ontology_msgs::GraphUpdate::CHANGE);
  }

  bool has_edge(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination)
  {
    for (auto edge : source->links)
    {
      if (edge->destination == destination)
        return true;
    }
    return false;
  }

  void set_edge_weight(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination, double weight)
  {
    for (auto l : source->links)
    {
      if (l->destination == destination)
        l->weight = weight;
    }

    publish_update(source, ontology_msgs::GraphUpdate::CHANGE);
  }
  
  double get_edge_weight(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr destination)
  {
    for (auto l : source->links)
    {
      if (l->destination == destination)
        return l->weight;
    }
    return 0;
  }

  typename GraphNode<T>::Ptr dfs_search(typename GraphNode<T>::Ptr root, std::vector<typename GraphNode<T>::Ptr>& explored, T target)
  {
    if (root->data == target)
      return root;

    typename GraphNode<T>::Ptr res = nullptr;
    explored.push_back(root);

    for (auto l : root->links)
    {
      auto w = l->destination;
      auto n = std::find(explored.begin(), explored.end(), w);
      if (n != explored.end()) // we've already explored the node l points to
        continue;

      res = dfs_search(w, explored, target);
      if (res != nullptr)
        return res;
    }

    return res;
  }

  typename GraphNode<T>::Ptr find(T value) const
  {
    for (auto n : graph)
    {
      if (n->data == value)
        return n;
    }
    return nullptr;
    /*
    std::vector<typename GraphNode<T>::Ptr> explored;
    return dfs_search(graph[0], explored, value);
    */

  }

  bool contains(T value) const
  {
    if (find(value) != nullptr)
      return true;
    else
      return false;
  }

  // merges source node into dest & updates all incoming edges
  void merge(typename GraphNode<T>::Ptr source, typename GraphNode<T>::Ptr dest)
  {
    // update all edges pointing to source
    for (auto& n : graph)
    {
      typename GraphEdge<T>::Ptr toSource = nullptr;
      typename GraphEdge<T>::Ptr toDest = nullptr;
      for (auto& l : n->links)
      {
        if (l->destination == source)
          toSource = l;
        else if (l->destination == dest)
          toDest = l;
        if (toSource != nullptr && toDest != nullptr)
          break;
      }

      // if we only point to source, just retarget the edge
      if (toSource != nullptr && toDest == nullptr)
      {
        toSource->destination = dest;
      }
      // if we have edges to both, combine their weights & remove toSource
      else if (toSource != nullptr && toDest != nullptr)
      {
        toDest->weight = toSource->weight + toDest->weight;
        remove_edge(n, source);
      }
    }

    // update all edges pointing out of source
    for (auto& l : source->links)
    {
      // if both source & dest point to the same place, combine weights
      if (has_edge(dest, l->destination))
      {
        set_edge_weight(dest, l->destination, get_edge_weight(dest, l->destination) + l->weight);
      }
      // otherwise create a new edge for it
      else
      {
        add_edge(dest, l->destination, l->weight);
      }
    }

    // remove source from graph
    remove_node(source);
  }

  std::vector<typename GraphNode<T>::Ptr> dfs_impl(typename GraphNode<T>::Ptr root, std::vector<typename GraphNode<T>::Ptr>& explored)
  {
    explored.push_back(root);

    for (auto l : root->links)
    {
      auto w = l->destination;
      auto n = std::find(explored.begin(), explored.end(), w);
      if (n != explored.end()) // we've already explored the node l points to
        continue;

      auto next_level = dfs_impl(w, explored);
    }

    return explored;
  }

  std::vector<typename GraphNode<T>::Ptr> depth_first_tree(typename GraphNode<T>::Ptr root)
  {
    std::vector<typename GraphNode<T>::Ptr> explored;
    return (dfs_impl(root, explored));
  }

  std::map<typename GraphNode<T>::Ptr, size_t> dijkstra(typename GraphNode<T>::Ptr root)
  {
    std::map<typename GraphNode<T>::Ptr, size_t> distances;
    std::map<typename GraphNode<T>::Ptr, typename GraphNode<T>::Ptr> predecessors;
    std::vector<typename GraphNode<T>::Ptr> visited;
    std::vector<typename GraphNode<T>::Ptr> Q;
 
    distances[root] = 0;

    // init
    for (auto v : graph)
    {
      if (v != root)
      {
        distances[v] = std::numeric_limits<size_t>::max();
      }
      Q.push_back(v);
    }

    while (!Q.empty())
    {
      // get minimum dist entry
      typename GraphNode<T>::Ptr next = nullptr;
      size_t min_dist = std::numeric_limits<size_t>::max();
      size_t min_idx = 0;
      for (size_t i = 0; i < Q.size(); i++)
      {
        if (distances[Q[i]] < min_dist)
        {
          min_dist = distances[Q[i]];
          min_idx = i;
          next = Q[i];
        }
      }

      // no more nodes can be reached from here
      if (next == nullptr)
        break;

      Q.erase(Q.begin() + min_idx); // remove next from Q
      visited.push_back(next);

      for (auto v : next->links)
      {
        double vd = distances[next] + v->weight; // curr dist is dist to next + weight of next->v
        if (vd < distances[v->destination])
        {
          distances[v->destination] = vd;
          predecessors[v->destination] = next;
        }
      }
    }

    return distances;
  }

  // gets all ancestors of the given node
  std::vector<typename GraphNode<T>::Ptr> ancestors(typename GraphNode<T>::Ptr node)
  {
    std::vector<typename GraphNode<T>::Ptr> result;

    for (auto l : node->links)
    {
      result.push_back(l->destination);
      auto next = ancestors(l->destination);
      for (auto n : next)
      {
        if (std::find(result.begin(), result.end(), n) == result.end())
          result.push_back(n);
      }
    }

    return result;
  }

  // gets the immediate parent(s) of the given node
  std::vector<typename GraphNode<T>::Ptr> parents(typename GraphNode<T>::Ptr node) const
  {
    std::vector<typename GraphNode<T>::Ptr> result;

    for (auto l : node->links)
    {
      result.push_back(l->destination);
    }

    return result;
  }

  // finds (a) lowest common ancestor of nodes n1 and n2
  // Assumes the graph is rooted, acyclic, and edges point toward parents
  typename GraphNode<T>::Ptr LCA(typename GraphNode<T>::Ptr n1, typename GraphNode<T>::Ptr n2)
  {
    // get distance from root to all nodes in graph
    std::vector<typename GraphNode<T>::Ptr> n1_parents = ancestors(n1);
    std::vector<typename GraphNode<T>::Ptr> n2_parents = ancestors(n2);
    std::vector<typename GraphNode<T>::Ptr> shared_parents; // ancestors of both n1 and n2

    for (auto n : n1_parents)
    {
      if (std::find(n2_parents.begin(), n2_parents.end(), n) != n2_parents.end())
      {
        shared_parents.push_back(n);
      }
    }

    // find common parent which is furthest from the root
    double max_dist = -1;
    typename GraphNode<T>::Ptr max_node;
    for (auto n : shared_parents)
    {
      auto distances = dijkstra(n);
      if (distances[graph[0]] > max_dist)
      {
        max_dist = distances[graph[0]];
        max_node = n;
      }
    }

    return max_node;
  }

  double similarity(typename GraphNode<T>::Ptr n1, typename GraphNode<T>::Ptr n2)
  {
    double sim = 0;

    auto lca = LCA(n1, n2);
    auto d1 = dijkstra(n1);
    auto d2 = dijkstra(n2);
    auto d3 = dijkstra(lca);

    // numerator:   2 * D_ij(a_ij)
    double num = 2.0 * d3[graph[0]];
    double den = d1[lca] + d2[lca] + 2 * d3[graph[0]];

    sim = num / den;

    // denomenator: D_i(a_i) + D_j(a_j) + 2 * D_ij(a_ij)
    // a_ij     : LCA of n1, n2
    // a_i      : n1
    // a_j      : n2
    // D_i(a_i) : d1[ LCA ]
    // D_j(a_j) : d2[ LCA ]
    // D_ij(a_ij) : d3[ root ]

    return sim;
  }

  static GraphNode<std::string>::Ptr fillGraph(Graph<std::string>& g, std::string rootClass, Ontology& onto)
  {
    auto root_node = g.find(rootClass);
    if (root_node == nullptr)
     root_node = g.add_node(onto.getShortName(rootClass));

    auto subclasses = onto.getDirectSubclassesOfClass(rootClass);

    for (auto& s : subclasses)
    {
      auto node = g.find(onto.getShortName(s));
      if (node == nullptr)
      {
        node = fillGraph(g, s, onto);
      }
      g.add_edge(node, root_node);
    }
    return root_node;
  }

  static Graph FromOntology(std::string rootClass, Ontology& onto)
  {
    Graph g;
    fillGraph(g, rootClass, onto);
    return g;
  }

  static void FromOntology(std::string rootClass, Ontology& onto, Graph& g)
  {
    fillGraph(g, rootClass, onto);
  }

  bool operator==(const Graph<T>& g) const
  {
    if (this->size() != g.size())
      return false;

    for (auto node : graph)
    {
      // check to ensure all of our nodes also exist in the other graph
      auto other = g.find(node->data);
      if (other == nullptr)
        return false;

      // check to ensure all edges on each node also match
      if (node->links.size() != other->links.size())
        return false;
      for (auto l : node->links)
      {
        bool found = false;
        for (auto ll : other->links)
        {
          if (l->destination->data == ll->destination->data &&
              l->weight == ll->weight)
          {
            found = true;
            break;
          }
        }
        if (!found)
          return false;
      }
    }

    return true;
  }

  bool operator!=(const Graph<T>& g) const
  {
    return !(*this == g);
  }

  void publish_update(typename GraphNode<T>::Ptr node, int operation)
  {
    if (update_pub == nullptr)
      return;

    ontology_msgs::GraphUpdate msg;
    msg.operation = operation;
    msg.node = nodeMessage<T>(node);

    update_pub->publish(msg);
  }

  ontology_msgs::Graph toMessage()
  {
    ontology_msgs::Graph msg;
    for (auto node : graph)
    {
      ontology_msgs::GraphNode n;
      n.name = node->data;
      for (auto link : node->links)
      {
        n.neighbors.push_back(link->destination->data);
        n.weights.push_back(link->weight);
      }

      msg.nodes.push_back(n);
    }
    return msg;
  }

  void toFile(std::string filename, bool ascii=false)
  {
    if (ascii)
    {
      std::ofstream out;
      out.open(filename);
      out << this->toMessage();
      out.close();
    }
    else
    {
      rosbag::Bag bag;
      bag.open(filename, rosbag::bagmode::Write);

      bag.write("graph", ros::TIME_MIN, this->toMessage());
      bag.close();
    }
  }

  static Graph<T> fromFile(std::string filename)
  {
    Graph<T> g;
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery("graph"));
    
    for (auto m : view)
    {
      ontology_msgs::Graph::ConstPtr gmsg = m.instantiate<ontology_msgs::Graph>();
      if (gmsg != NULL)
      {
        for (auto n : gmsg->nodes)
        {
          g.add_node(n.name);
        }

        for (auto n : gmsg->nodes)
        {
          for (size_t i = 0; i < n.neighbors.size(); i++)
          {
            g.add_edge(g.find(n.name), g.find(n.neighbors[i]), n.weights[i]);
          }
        }

      }
    }

    bag.close();
    return g;
  }
};

#endif // _GRAPH_H_
