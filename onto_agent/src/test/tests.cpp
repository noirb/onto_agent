#include <Graph.hpp>
#include <lest.hpp>

using lest::approx;

const lest::test specification[] =
{
  CASE( "Graph Node Insertion" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    EXPECT( (n1 != nullptr && n2 != nullptr && n3 != nullptr) );
    EXPECT( g.contains(1) );
    EXPECT( g.contains(2) );
    EXPECT( g.contains(3) );
  },

  CASE( "Inserted nodes can be retrieved" )
  {
    Graph<int> g;
    g.add_node(1);
    g.add_node(2);
    g.add_node(3);

    auto n1 = g.find(1);
    auto n2 = g.find(2);
    auto n3 = g.find(3);

    EXPECT( (n1 != nullptr && n1->data == 1) );
    EXPECT( (n2 != nullptr && n2->data == 2) );
    EXPECT( (n3 != nullptr && n3->data == 3) );
  },

  CASE( "Edge insertion" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n1, n2);
    g.add_edge(n2, n3);
    g.add_edge(n3, n1);

    EXPECT( n1->links.size() > 0 );
    EXPECT( n1->links[0]->destination == n2 );
    EXPECT( n2->links.size() > 0 );
    EXPECT( n2->links[0]->destination == n3 );
    EXPECT( n3->links.size() > 0 );
    EXPECT( n3->links[0]->destination == n1 );
  },

  CASE( "Multiple Edges per node" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n1, n2);
    g.add_edge(n1, n3);

    g.add_edge(n2, n3);
    g.add_edge(n2, n1);

    EXPECT( n1->links.size() == 2 );
    EXPECT( n2->links.size() == 2 );
    EXPECT( n3->links.size() == 0 );

    EXPECT( n1->links[0]->destination != n1->links[1]->destination );
  },

  CASE( "Node OutDegree (empty)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);

    EXPECT( n1->outdegree() == 0 );
  },

  CASE( "Node OutDegree (add edges)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);

    g.add_edge(n1, n2);
    EXPECT( n1->outdegree() == 1 );
    EXPECT( n2->outdegree() == 0 );

    g.add_edge(n1, n1);
    EXPECT( n1->outdegree() == 2 );
  },

  CASE( "Node OutDegree (add + remove edges)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n1, n2);
    g.add_edge(n1, n3);
    EXPECT( n1->outdegree() == 2 );

    g.remove_edge(n1, n2);
    EXPECT( n1->outdegree() == 1 );

    g.remove_edge(n1, n3);
    EXPECT( n1->outdegree() == 0 );

    g.add_edge(n1, n2);
    EXPECT( n1->outdegree() == 1 );
  },

  CASE( "Graph Equivalence (empty)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    EXPECT( g1 == g2 );
  },

  CASE( "Graph Equivalence (1 node -- matching)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    g1.add_node(42);
    g2.add_node(42);

    EXPECT( g1 == g2 );
  },

  CASE( "Graph Equivalence (1 node -- different)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    g1.add_node(42);
    g2.add_node(24);

    EXPECT( g1 != g2 );
  },

  CASE( "Graph Equivalence (multiple nodes -- matching)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    for (int i = 0; i < 5; i++)
    {
      g1.add_node(i);
      g2.add_node(i);
    }

    EXPECT( g1 == g2 );
  },

  CASE( "Graph Equivalence (multiple nodes -- different)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    for (int i = 0; i < 5; i++)
    {
      g1.add_node(i);
      g2.add_node(i*10);
    }

    EXPECT( g1 != g2 );
  },

  CASE( "Graph Equivalence (different sizes)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    for (int i = 0; i < 5; i++)
    {
      g1.add_node(i);
      g2.add_node(i);
    }
    g2.add_node(42);

    EXPECT( g1 != g2 );
  },

  CASE( "Graph Equivalence (1 edge -- matching)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    auto n1 = g1.add_node(1);
    auto n2 = g1.add_node(2);
    g1.add_edge(n1, n2);

    n1 = g2.add_node(1);
    n2 = g2.add_node(2);
    g2.add_edge(n1, n2);

    EXPECT( g1 == g2 );
  },
  
  CASE( "Graph Equivalence (1 edge -- different" )
  {
    Graph<int> g1;
    Graph<int> g2;

    auto n1 = g1.add_node(1);
    auto n2 = g1.add_node(2);
    g1.add_edge(n1, n2);

    n1 = g2.add_node(1);
    n2 = g2.add_node(2);
    g2.add_edge(n2, n1);

    EXPECT( g1 != g2 );
  },

  CASE( "Graph Equivalence (multiple edges -- matching)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    for (int i = 0; i < 10; i+=2)
    {
      auto n1 = g1.add_node(i);
      auto n2 = g1.add_node(i+1);
      g1.add_edge(n1, n2);
      g1.add_edge(n2, n1);

      n1 = g2.add_node(i);
      n2 = g2.add_node(i+1);
      g2.add_edge(n1, n2);
      g2.add_edge(n2, n1);
    }

    EXPECT( g1 == g2 );
  },

  CASE( "Graph Equivalence (multiple edges -- different)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    auto root1 = g1.add_node(42);
    auto root2 = g2.add_node(42);

    for (int i = 0; i < 10; i+=2)
    {
      auto n1 = g1.add_node(i);
      auto n2 = g1.add_node(i+1);
      g1.add_edge(n1, n2);
      g1.add_edge(n2, n1);
      g1.add_edge(root1, n1);
      g1.add_edge(root1, n2);

      n1 = g2.add_node(i);
      n2 = g2.add_node(i+1);
      g2.add_edge(n1, n2);
      g2.add_edge(n2, n1);
      g2.add_edge(n1, root2);
      g2.add_edge(n2, root2);
    }

    EXPECT( g1 != g2 );
  },

  CASE( "Graph Equivalence (multiple edges -- different count)" )
  {
    Graph<int> g1;
    Graph<int> g2;

    auto root1 = g1.add_node(42);
    auto root2 = g2.add_node(42);

    for (int i = 0; i < 10; i+=2)
    {
      auto n1 = g1.add_node(i);
      auto n2 = g1.add_node(i+1);
      g1.add_edge(n1, n2);
      g1.add_edge(n2, n1);

      n1 = g2.add_node(i);
      n2 = g2.add_node(i+1);
      g2.add_edge(n1, n2);
      g2.add_edge(n2, n1);
      g2.add_edge(n1, root2);
    }

    EXPECT( g1 != g2 );
  },

  CASE( "Merge Nodes (disjoint 1)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);

    g.merge(n1, n2);

    EXPECT( g.find(1) == nullptr ); // n1 should no longer exist in the graph
    EXPECT( g.find(2) != nullptr ); // n2 should still be there
  },

  CASE( "Merge Nodes (disjoint 2)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n1, n2, 1); //   n1
    g.add_edge(n2, n3, 1); //  /
                           // n2--n3
    g.merge(n3, n1); // n1<-->n2

    EXPECT( g.has_edge(n1, n2) ); // original edge 1->2 should still exist
    EXPECT( g.has_edge(n2, n1) ); // edge 2->3 should now be 2->1
    EXPECT( g.get_edge_weight(n2, n1) == 1 ); // edge weight is unchanged
    EXPECT( n1->outdegree() == 1 ); // n1 still only has one outgoing edge
    EXPECT( n2->outdegree() == 1 ); // n2 still only has one outgoing edge
  },

  CASE( "Merge Nodes (disjoint 3)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n2, n1, 1);
    g.add_edge(n2, n3, 1);
    EXPECT( n2->outdegree() == 2 );

    g.merge(n1, n3);

    EXPECT( n2->outdegree() == 1 ); // n1 no longer exists & we don't want to edges to n3
    EXPECT( g.has_edge(n2, n3) );
    EXPECT( !g.has_edge(n3, n2) );
    EXPECT( n3->outdegree() == 0 );
  },

  CASE( "Merge Nodes (connected)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);

    g.add_edge(n1, n2, 1); //   n1
    g.add_edge(n2, n3, 1); //  /  \.
    g.add_edge(n3, n1, 1); // n2--n3
    g.add_edge(n2, n1, 1);
    g.add_edge(n1, n3, 1);
    g.add_edge(n3, n2, 1);

    // first merge:
    //    n1<-->n2<-
    g.merge(n3, n2);
    EXPECT( g.find(3) == nullptr );
    EXPECT( n1->outdegree() == 1 ); // n1->n2 still exists
    EXPECT( n2->outdegree() == 2 ); // n2->n1 PLUS n2->n2 exist (inherited from n3)
    EXPECT( !g.has_edge(n1, n1) );
    EXPECT( g.has_edge(n1, n2) );
    EXPECT( g.has_edge(n2, n2) );
    EXPECT( g.has_edge(n2, n1) );

    // edge weights should all be merged
    EXPECT( g.get_edge_weight(n1, n2) == 2 ); // n1->n2 + n1->n3
    EXPECT( g.get_edge_weight(n2, n1) == 2 ); // n2->n1 + n3->n1
    EXPECT( g.get_edge_weight(n2, n2) == 2 ); // n2->n3 + n3->n2
    
    // second merge:
    //     ->n1<-
    g.merge(n2, n1);
    EXPECT( g.find(2) == nullptr );
    EXPECT( n1->outdegree() == 1 ); // should only point to itself
    EXPECT( g.has_edge(n1, n1) );
    EXPECT( g.get_edge_weight(n1, n1) == 6 );

  },

  CASE( "Ancestors (simple tree)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);
    auto n4 = g.add_node(4);
                        //    n1
    g.add_edge(n2, n1); //   /  \. 
    g.add_edge(n3, n1); // n2    n3
    g.add_edge(n4, n2); //  |
                        // n4
    std::vector<typename GraphNode<int>::Ptr> n4_expected_ancestors = {
      n2, n1
    };
    std::vector<typename GraphNode<int>::Ptr> n3_expected_ancestors = {
      n1
    };

    EXPECT( g.ancestors(n3) == n3_expected_ancestors );
    EXPECT( g.ancestors(n4) == n4_expected_ancestors );
  },

  CASE( "Ancestors (multiple parents)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);
    auto n4 = g.add_node(4);
                        //    n1
    g.add_edge(n2, n1); //   /  \. 
    g.add_edge(n3, n1); //  n2  n3
    g.add_edge(n4, n2); //   \  /
    g.add_edge(n4, n3); //    n4

    std::vector<typename GraphNode<int>::Ptr> n4_expected_ancestors = {
      n2, n3, n1
    };

    auto n4_ancestors = g.ancestors(n4);

    // must have same number of entries
    EXPECT( n4_ancestors.size() == n4_expected_ancestors.size() );
    // must contain same entries (but not necessarily same order
    for (auto n : n4_expected_ancestors)
    {
      EXPECT( std::find(n4_ancestors.begin(), n4_ancestors.end(), n) != n4_ancestors.end() );
    }
  },

  CASE( "LCA (simple tree)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);
    auto n4 = g.add_node(4);
    auto n5 = g.add_node(5);

    g.add_edge(n2, n1);
    g.add_edge(n3, n1);
    g.add_edge(n4, n3);
    g.add_edge(n5, n3);

    EXPECT( g.LCA(n4, n5) == n3 );
    EXPECT( g.LCA(n4, n2) == n1 );
  },

  CASE( "Semantic Similarity (Ilya's Example Tree)" )
  {
    Graph<std::string> g;
    auto thing                      = g.add_node("Thing");
    auto temporalThing              = g.add_node("TemporalThing");
    auto intrinsicStateChangeEvent  = g.add_node("IntrinsicStateChangedEvent");
    auto property                   = g.add_node("Property");
    auto stateTransition            = g.add_node("StateTransition");
    auto generalProperty            = g.add_node("GeneralProperty");
    auto isInHand                   = g.add_node("IsInHand");
    auto userDefinedProperty        = g.add_node("UserDefinedProperty");
    auto hasDestination             = g.add_node("HasDestination");
    auto hasLocation                = g.add_node("HasLocation");
    auto event                      = g.add_node("Event");
    auto action                     = g.add_node("Action");
    auto removeSomething            = g.add_node("RemoveSomething");
    auto clean                      = g.add_node("Clean");
    auto unwrap                     = g.add_node("Unwrap");
    auto sweep                      = g.add_node("Sweep");
    auto wipe                       = g.add_node("Wipe");

    g.add_edge(temporalThing,              thing);
    g.add_edge(intrinsicStateChangeEvent,  temporalThing);
    g.add_edge(event,                      temporalThing);
    g.add_edge(property,                   intrinsicStateChangeEvent);
    g.add_edge(stateTransition,            intrinsicStateChangeEvent);
    g.add_edge(generalProperty,            property);
    g.add_edge(userDefinedProperty,        property);
    g.add_edge(isInHand,                   generalProperty);
    g.add_edge(hasDestination,             userDefinedProperty);
    g.add_edge(hasLocation,                userDefinedProperty);
    g.add_edge(action,                     event);
    g.add_edge(removeSomething,            action);
    g.add_edge(clean,                      removeSomething);
    g.add_edge(unwrap,                     removeSomething);
    g.add_edge(sweep,                      clean);
    g.add_edge(wipe,                       clean);

    lest::approx approximator = lest::approx::custom().epsilon(0.001);
    EXPECT( g.similarity(unwrap, wipe) == approximator(0.727) );
    EXPECT( g.similarity(sweep, wipe)  == approximator(0.833) );
  },

  CASE( "LCA (multiple parents" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);
    auto n4 = g.add_node(4);
    auto n5 = g.add_node(5);

    g.add_edge(n2, n1);
    g.add_edge(n3, n1);
    g.add_edge(n4, n2);
    g.add_edge(n4, n1);
    g.add_edge(n5, n2);

    EXPECT( g.LCA(n4, n5) == n2 );
  },

  CASE( "Semantic Similarity (simple)" )
  {
    Graph<int> g;
    auto n1 = g.add_node(1);
    auto n2 = g.add_node(2);
    auto n3 = g.add_node(3);
    auto n4 = g.add_node(4);
    auto n5 = g.add_node(5);

    g.add_edge(n2, n1);
    g.add_edge(n3, n1);
    g.add_edge(n4, n2);
    g.add_edge(n5, n2);

    EXPECT( g.similarity(n4, n5) > 0 );
  },

  CASE( "File output 1" )
  {
    Graph<std::string> g;
    Graph<std::string> res;

    std::string out_file = "graph_test_output.1.txt";

    auto n1 = g.add_node("root");
    auto n2 = g.add_node("2");
    auto n3 = g.add_node("3");
    auto n4 = g.add_node("4");
    auto n5 = g.add_node("5");

    g.add_edge(n2, n1);
    g.add_edge(n3, n1);
    g.add_edge(n4, n2);
    g.add_edge(n5, n2);

    EXPECT( (g.toFile(out_file), true) );

    EXPECT( (res = Graph<std::string>::fromFile(out_file), true) );

    EXPECT( g == res );

    EXPECT( (res.toFile(out_file, true), true) );
  },

  CASE( "File output 2" )
  {
    Graph<std::string> g;
    Graph<std::string> res;

    std::string out_file = "graph_test_output.2.txt";

    auto n1 = g.add_node("root");
    auto n2 = g.add_node("2");
    auto n3 = g.add_node("3");
    auto n4 = g.add_node("4");
    auto n5 = g.add_node("5");

    g.add_edge(n2, n1);
    g.add_edge(n3, n1);
    g.add_edge(n4, n1);
    g.add_edge(n5, n1);

    EXPECT( (g.toFile(out_file), true) );
    EXPECT( (res = Graph<std::string>::fromFile(out_file), true) );
    EXPECT( g == res );
    EXPECT( (res.toFile(out_file, true), true) );
  },

  CASE( "File output 3" )
  {
    Graph<std::string> g;
    Graph<std::string> res;

    std::string out_file = "graph_test_output.3.txt";

    auto n1 = g.add_node("root");
    auto n2 = g.add_node("2");
    auto n3 = g.add_node("3");
    auto n4 = g.add_node("4");
    auto n5 = g.add_node("5");

    g.add_edge(n2, n1);
    g.add_edge(n3, n2);
    g.add_edge(n3, n1);
    g.add_edge(n4, n1);
    g.add_edge(n4, n2);
    g.add_edge(n4, n3);
    g.add_edge(n5, n1);
    g.add_edge(n5, n2);
    g.add_edge(n5, n3);
    g.add_edge(n5, n4);

    EXPECT( (g.toFile(out_file), true) );
    EXPECT( (res = Graph<std::string>::fromFile(out_file), true) );
    EXPECT( g == res );
    EXPECT( (res.toFile(out_file, true), true) );
  }
};

int main(int argc, char** argv)
{
  return lest::run( specification, argc, argv, std::cout );
}
