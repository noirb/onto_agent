#ifndef _ONTOTEST_H_
#define _ONTOTEST_H_

#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <curses.h>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ontology_svcs/GetAllClassInstancesWithPoses.h>
#include <ontology_svcs/InstructBehavior.h>
#include <ontology_svcs/GetActivity.h>
#include <ontology_svcs/GetActivities.h>
#include <ontology_svcs/GetActivityGraph.h>
#include <ontology_svcs/RelabelActivity.h>
#include <ontology_svcs/RelabelObject.h>
#include <ontology_svcs/AssertClass.h>
#include <ontology_svcs/SaveLog.h>

#include <ontology_msgs/SimulationStateChange.h>
#include <ontology_msgs/PropertyChanged.h>
#include <ontology_msgs/ObjUpdate.h>
#include <ontology_msgs/HandUpdate.h>
#include <ontology_msgs/DisplayLabel.h>
#include <ontology_msgs/Graph.h>
#include <ontology_msgs/GraphUpdate.h>

#include "Graph.hpp"
#include "Ontology.hpp"
#include "Activity.hpp"
#include "MindState.hpp"
#include "Constants.hpp"

#define MOVE_MIN_VEL 0.04f
#define ACT_MIN_DIST 0.0225f // squared distance
#define MIN_ACTIVITY_DURATION 0.05

class Colors
{
  public:
  int White;
  int Grey;
  int Yellow;
  int Green;
  int GreenHighlight;
  int GreyHighlight;
};

struct EntityInstance
{
  std::string base_class;    // ontology class this instance was created from
  std::string instance_name; // instance name given from KnowROB
  std::map<std::string, std::string> properties; // properties applied to this instance (e.g. 'Dirtiness' == 'Clean')
  EntityInstance() {}

  EntityInstance(std::string instanceName, std::string baseClass) :
    base_class(baseClass),
    instance_name(instanceName)
  {}

  EntityInstance(std::string instanceName, std::string baseClass, std::map<std::string, std::string> instanceProperties) :
    base_class(baseClass),
    instance_name(instanceName),
    properties(instanceProperties)
  {}

  bool HasProperty(std::string property)
  {
    return properties.find(property) != properties.end();
  }

  std::string GetProperty(std::string property)
  {
    if (!HasProperty(property))
      throw std::runtime_error("Entity Instance " + instance_name + " has no property: " + property);
    
    return properties[property];
  }

  void SetProperty(std::string property, std::string value)
  {
    properties[property] = value;
  }

  bool operator==(const EntityInstance& other) const
  {
    return instance_name == other.instance_name;
  }

  bool operator!=(const EntityInstance& other) const
  {
    return !(*this == other);
  }
};

struct MotionSegment
{
  std::string effectorName = "NONE";
  double start_time = 0;
  double end_time = 0;
  std::string obj_actedOn = "NONE";
  std::map<std::string, std::string> obj_actedOn_properties;
  std::string obj_inHand = "NONE";
  std::map<std::string, std::string> obj_inHand_properties;
  Eigen::Vector3d velocity;
  bool moving = false;
  bool toolUse = false;
  std::string classification = UNKNOWN_ACTIVITY;
  bool fresh = false;

  double Duration()
  {
    return end_time - start_time;
  }

  bool Matches(MotionSegment& m)
  {
    if (this->obj_actedOn  == m.obj_actedOn &&
        this->obj_inHand   == m.obj_inHand  &&
        this->moving       == m.moving      &&
        this->toolUse      == m.toolUse     &&
        this->effectorName == m.effectorName
       )
    {
      for (auto p : obj_inHand_properties)
      {
        if (m.obj_inHand_properties.find(p.first) == m.obj_inHand_properties.end())
        {
          return false;
        }
        if (m.obj_inHand_properties[p.first] != p.second)
        {
          return false;
        }
      }
      for (auto p : obj_actedOn_properties)
      {
        if (m.obj_actedOn_properties.find(p.first) == m.obj_actedOn_properties.end())
        {
          return false;
        }
        if (m.obj_actedOn_properties[p.first] != p.second)
        {
          return false;
        }
      }
      return true;
    }
    else
    {
      return false;
    }
  }
};


class Agent
{
  Activity unknownActivity = Activity("UnknownActivity", "#UnknownActivity");
  std::vector<Activity> knownActivities;
  std::map<std::string, EntityInstance> entityInstances;
  std::map<std::string, Eigen::Vector3d> entityPositions;
  std::vector<MotionSegment> motions_L;
  std::vector<MotionSegment> motions_R;
  Colors colors;

  Ontology ontology;
  Graph<std::string> ontograph;
  Graph<Activity> taskGraph;

  std::string log_dir;

  ros::Publisher leftLabel_pub;
  ros::Publisher rightLabel_pub;
  ros::Publisher ontograph_pub;
  ros::Publisher taskGraph_pub;

  ros::ServiceServer getactivity_srv, getactivities_srv, getactivitygraph_srv,
                     relabelactivity_srv, relabelobject_srv, assertnewclass_srv,
                     savelog_srv;
  ros::ServiceClient ontolist, relabelobject;

  MindState mindState = MindState::Idle;
  std::string targetActivity;

  double baseTimeStamp = 0; // in case we observe several scenarios we must track a time offset between them
public:
  Agent(){};
  Agent(std::vector<Activity> activities) : knownActivities(activities){};
  ~Agent();

  void run();

private:
  bool hasProperty(ActivityPropertyType type, std::vector<std::string> range, EntityInstance entity);
  bool hasProperty(ActivityProperty prop, EntityInstance entity);

  void writeMotionLog(std::vector<MotionSegment> motions, std::string filename);
  void writeActivities(std::string filename);

  std::string classify(MotionSegment m);
  std::string classifyGranular(MotionSegment m);

  std::string makeID()
  {
    return to_string(boost::uuids::random_generator()());
  }

  std::string makeID(std::string prefix)
  {
    return prefix + makeID();
  }

  double getOffsetTime(ros::Time time, double offset)
  {
    return offset + ((double)time.sec + ((double)time.nsec * 0.00000001));
  }

  Activity& processUnknownActivity(MotionSegment& m);
  Activity& getActivityByName(std::string name);

  // Updates the Task Graph with a transition between two activities
  // Will either add a new edge or increase the count on an existing edge
  void addTaskTransition(Activity from, Activity to);

  Graph<std::string> getActivityGraph(Activity& activity);
  void simStateChangedCB(const ontology_msgs::SimulationStateChange::ConstPtr& msg);
  void propertyChangedCB(const ontology_msgs::PropertyChanged::ConstPtr& msg);

  void objPoseUpdateCB(const ontology_msgs::ObjUpdate::ConstPtr& msg);
  void leftHandUpdate(const ontology_msgs::HandUpdate::ConstPtr& msg);
  void rightHandUpdate(const ontology_msgs::HandUpdate::ConstPtr& msg);

  bool instructBehaviorCB(ontology_svcs::InstructBehaviorRequest& req, ontology_svcs::InstructBehaviorResponse& res);
  bool getActivityCB(ontology_svcs::GetActivityRequest& req,
                     ontology_svcs::GetActivityResponse& res);
  bool getActivitiesCB(ontology_svcs::GetActivitiesRequest& req,
                       ontology_svcs::GetActivitiesResponse& res);
  bool getActivityGraphCB(ontology_svcs::GetActivityGraphRequest& req,
                          ontology_svcs::GetActivityGraphResponse& res);
  bool relabelActivityCB(ontology_svcs::RelabelActivityRequest& req,
                         ontology_svcs::RelabelActivityResponse& res);
  bool relabelObjectCB(ontology_svcs::RelabelObjectRequest& req,
                       ontology_svcs::RelabelObjectResponse& res);
  bool assertNewClassCB(ontology_svcs::AssertClassRequest& req,
                        ontology_svcs::AssertClassResponse& res);
  bool saveLogCB(ontology_svcs::SaveLogRequest& req,
                 ontology_svcs::SaveLogResponse& res);

  void appendToMotionLog(MotionSegment& m, std::vector<MotionSegment>& log);
  void checkMoving(MotionSegment& m);
  void checkActedOn(MotionSegment& m, Eigen::Vector3d currPos);
  void checkToolUse(MotionSegment& m);
  void checkObjProperties(MotionSegment& m);

  std::vector<std::string> bubbleActivityProperty(std::string parent, std::vector<std::string> childset);
  std::vector<std::string> bubbleActivityProperty(ActivityProperty& activityProperty, std::string newClass);
  void bubbleActivityProperty(Activity& activity, std::string newClass);
  void bubbleActivityProperty(Activity& activity);

  void updateUI();

  void write(WINDOW* wnd, std::string str);
  void write(WINDOW* wnd, int y, int x, std::string str);

  void writeToggle(WINDOW* wnd, std::string str, bool enabled);
  void writeToggle(WINDOW* wnd, int y, int x, std::string str, bool enabled);

  void init_curses();

};

#endif // _ONTOTEST_H
