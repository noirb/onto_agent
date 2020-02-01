#include "Agent.hpp"
#include "utils.hpp"

#include <algorithm>

Agent::~Agent()
{
  endwin();

  std::cout << "Shutting down..." << std::endl;
  std::cout << "Writing log data to directory: " << log_dir << std::endl;
  if (motions_L.size() > 0)
  {
    std::cout << "Writing motion log for left hand..." << std::endl;;
    writeMotionLog(motions_L, log_dir + "/motion_log_left.txt");
  }
  else
  {
    std::cout << "No motion data for left hand. Skipping." << std::endl;
  }

  if (motions_R.size() > 0)
  {
    std::cout << "Writing motion log for right hand..." << std::endl;;
    writeMotionLog(motions_R, log_dir + "/motion_log_right.txt");
  }
  else
  {
    std::cout << "No motion data for right hand. Skipping." << std::endl;
  }

  if (taskGraph.size() > 0)
  {
    std::cout << "Writing task graph..." << std::endl;
    taskGraph.toFile(log_dir + "/task_graph.bag", false);
    taskGraph.toFile(log_dir + "/task_graph.txt", true);
  }
  else
  {
    std::cout << "Task graph is empty. Skipping." << std::endl;
  }

  std::cout << "Writing activities..." << std::endl;
  writeActivities(log_dir + "/activities.txt");
}

void Agent::writeMotionLog(std::vector<MotionSegment> motions, std::string filename)
{
  std::ofstream outfile;
  outfile.open(filename);

  outfile << "# Motion segmentation log (v0.1)" << std::endl;
  outfile << "effector_name, start_time, end_time, moving, tool_use, obj_acted_on, obj_in_hand, activity" << std::endl;
  for (auto& m : motions)
  {
    outfile << m.effectorName                           << ", "
            << m.start_time                             << ", "
            << m.end_time                               << ", "
            << (m.moving  ?   "moving" : "stopped")     << ", "
            << (m.toolUse ? "tool_use" : "no_tool_use") << ", "
            << m.obj_actedOn                            << ", "
            << m.obj_inHand                             << ", "
            << m.classification
            << std::endl;
  }
  outfile.close();
}

void Agent::writeActivities(std::string filename)
{
  std::ofstream outfile;
  outfile.open(filename);

  for (auto act : knownActivities)
  {
    outfile << std::string(act) << std::endl;
  }
}

std::string Agent::classify(MotionSegment m)
{
  if (m.moving == false &&
      m.obj_actedOn == NONE &&
      m.obj_inHand  == NONE)
    return "IdleMotion";

  if (m.moving == true &&
      m.obj_actedOn != NONE &&
      m.obj_inHand  == NONE)
    return "Reach";

  if (m.moving == false &&
      m.obj_actedOn == NONE &&
      m.obj_inHand  != NONE)
    return "Take";

  if (m.moving == true &&
      m.obj_actedOn == NONE &&
      m.obj_inHand  == NONE)
    return "Release";

  if (m.moving == true &&
      m.obj_actedOn == NONE &&
      m.obj_inHand  != NONE)
    return "PutSomethingSomewhere";

  if (m.toolUse == true &&
      m.obj_actedOn != NONE &&
      m.obj_inHand  != NONE)
    return classifyGranular(m);

  return UNKNOWN_ACTIVITY;
}

bool Agent::hasProperty(ActivityPropertyType type, std::vector<std::string> range, EntityInstance entity)
{
  switch(type)
  {
    case ActivityPropertyType::IsSubclassOf:
    {
      auto classes = ontology.getSubclassesOfClass(entity.base_class);
      for (auto c : range)
      {
        if (ontology.isSubclassOf(entity.instance_name, c))
          return true;
      }
      break;
    }
    case ActivityPropertyType::IsClass:
    {
      return std::find(range.begin(), range.end(), entity.base_class) != range.end();
    }
    case ActivityPropertyType::IsNotSubclassOf:
    {
      auto classes = ontology.getSubclassesOfClass(entity.base_class);
      for (auto& c : classes)
      {
        if (std::find(range.begin(), range.end(), c) != range.end())
        {
          return false;
        }
      }
      return true;
    }
    case ActivityPropertyType::IsNotClass:
    {
      return std::find(range.begin(), range.end(), entity.base_class) == range.end();
    }
    case ActivityPropertyType::HasValue:
    {
      auto properties = ontology.getProperties(entity.instance_name);
      // every two entries in range are a property->value pair
      for (size_t i = 0; i < range.size(); i += 2)
      {
        auto it = properties.begin();
        if ((it = properties.find(range[i])) == properties.end()) // property does not exist
          return false;
        if ((*it).second != range[i+1]) // property value does not match
          return false;
      }
      return true;
    }
    case ActivityPropertyType::DoesNotHaveValue:
    {
      auto properties = ontology.getProperties(entity.instance_name);
      for (size_t i = 0; i < range.size(); i += 2)
      {
        auto it = properties.begin();
        if ((it = properties.find(range[i])) == properties.end()) // property does not exist
          return false;
        if ((*it).second != range[i+1]) // property values do not match
          return false;
      }
      return true;
      if (properties.find(range[0]) == properties.end())
        return false;
    }
  }
  return false;
}

bool Agent::hasProperty(ActivityProperty prop, EntityInstance entity)
{
  for (auto constraint : prop.values)
  {
    if (!hasProperty(constraint.first, constraint.second, entity))
        return false;
  }
  return true;
}

std::string Agent::classifyGranular(MotionSegment m)
{
  for (auto a : knownActivities)
  {
    if (a.HasProperty(S_INHAND))
    {
      auto prop = a.GetProperty(S_INHAND);
      if (!hasProperty(prop, this->entityInstances[m.obj_inHand]))
        continue; // this activity is not satisfied, try next
    }
    else
    {
      continue; // this activity is not granular
    }

    if (a.HasProperty(S_ACTEDON))
    {
      auto prop = a.GetProperty(S_ACTEDON);
      if (!hasProperty(prop, this->entityInstances[m.obj_actedOn]))
        continue; // this activity is not satisfied, try next
    }
    else
    {
      continue; // this activity is not granular
    }

    return a.name;
  }

  return UNKNOWN_GRANULAR_ACTIVITY;
}

bool Agent::instructBehaviorCB(ontology_svcs::InstructBehaviorRequest& req, ontology_svcs::InstructBehaviorResponse& res)
{
  std::cout << "New goal: " << req.targetBehavior << " : " << req.params << std::endl;

  return true;
}

bool Agent::getActivitiesCB(ontology_svcs::GetActivitiesRequest& req, ontology_svcs::GetActivitiesResponse& res)
{
  for (auto a : knownActivities)
  {
    ontology_msgs::Activity act;
    act.name = a.name;
    act.className = a.ontologyClass;
    if (a.HasProperty(S_INHAND))
    {
      auto inhand = a.GetProperty(S_INHAND);
      for (auto i : inhand.values)
      {
        std::string str = ActivityPropertyTypeName(i.first) + "(";
        for (auto v : i.second)
          str += ontology.getShortName(v) + ", ";
        str += ")";
        act.inHand.push_back(str);
      }
    }
    if (a.HasProperty(S_ACTEDON))
    {
      auto actedon = a.GetProperty(S_ACTEDON);
      for (auto i : actedon.values)
      {
        std::string str = ActivityPropertyTypeName(i.first) + "(";
        for (auto v : i.second)
          str += ontology.getShortName(v) + ", ";
        str += ")";
        act.actedOn.push_back(str);
      }
    }

    res.activities.push_back(act);
  }
  return true;
}

bool Agent::getActivityCB(ontology_svcs::GetActivityRequest& req, ontology_svcs::GetActivityResponse& res)
{
  for (auto a : knownActivities)
  {
    if (a.name == req.name)
    {
      res.activity.name = a.name;
      res.activity.className = a.ontologyClass;

      if (a.HasProperty(S_INHAND))
      {
        auto inhand = a.GetProperty(S_INHAND);
        for (auto i : inhand.values)
        {
          std::string str = ActivityPropertyTypeName(i.first) + "(";
          for (auto v : i.second)
            str += v + ", ";
          str += ")";
          res.activity.inHand.push_back(str);
        }
      }
      if (a.HasProperty(S_ACTEDON))
      {
        auto actedon = a.GetProperty(S_ACTEDON);
        for (auto i : actedon.values)
        {
          std::string str = ActivityPropertyTypeName(i.first) + "(";
          for (auto v : i.second)
            str += v + ", ";
          str += ")";
          res.activity.actedOn.push_back(str);
        }
      }

      return true;
    }
  }
  return false; // requested activity does not exist
}

bool Agent::getActivityGraphCB(ontology_svcs::GetActivityGraphRequest& req, ontology_svcs::GetActivityGraphResponse& res)
{
  for (auto a : knownActivities)
  {
    if (a.name == req.name)
    {
      res.graph = getActivityGraph(a).toMessage();
      return true;
    }
  }
  return false; // requested activity does not exist
}

bool Agent::relabelActivityCB(ontology_svcs::RelabelActivityRequest& req, ontology_svcs::RelabelActivityResponse& res)
{
  res.success = false;

  if (req.oldName.length() < 1 || req.newName.length() < 1 ||
      req.oldName == req.newName)
  {
    res.reason = "Invalid arguments: '" + req.oldName + "', '" + req.newName + "'";
    return true;
  }

  Activity* oldActivity = nullptr;
  Activity* newActivity = nullptr;
  for (auto& activity : knownActivities)
  {
    if (activity.name == req.oldName)
    {
      oldActivity = &activity;
    }
    else if (activity.name == req.newName)
    {
      newActivity = &activity;
    }
    if (oldActivity != nullptr && newActivity != nullptr)
    {
      break;
    }
  }

  // bail if we didn't find a match
  if (oldActivity == nullptr)
  {
    res.reason = "Could not find activity: '" + req.oldName + "'";
    return true;
  }

  // if newActivity exists, merge in the properties from oldActivity
  if (newActivity != nullptr)
  {
    for (auto& prop : oldActivity->properties)
    {
      if (newActivity->HasProperty(prop.first))
      {
        auto& newProperty = newActivity->GetProperty(prop.first);
        for (auto& p : prop.second.values)
        {
          for (auto& v : p.second)
          {
            if (!newProperty.has(p.first, v))
            {
             newProperty.append(p.first, v);
            }
          }
        }
      }
      else // add all properties of this type to newActivity
      {
        for (auto& p : prop.second.values)
        {
          newActivity->SetProperty(prop.first, p.first, p.second);
        }
      }
    }

    // merge task graph entries for old & new activity
    taskGraph.merge(taskGraph.find(*oldActivity), taskGraph.find(*newActivity));

    // remove entry for oldActivity
    knownActivities.erase(std::find(std::begin(knownActivities), std::end(knownActivities), *oldActivity));
  }
  else
  {
    auto oldNode = taskGraph.find(*oldActivity);
    oldActivity->name = req.newName;
    oldActivity->ontologyClass = "#" + req.newName;
    oldNode->data = *oldActivity;
  }

  // update task graph & motion log entries for oldActivity to match newActivity
  for (auto& m : motions_L)
  {
    if (m.classification == req.oldName)
      m.classification = req.newName;
  }
  for (auto& m : motions_R)
  {
    if (m.classification == req.oldName)
      m.classification = req.newName;
  }

  res.success = true;
  return true;
}

bool Agent::relabelObjectCB(ontology_svcs::RelabelObjectRequest& req, ontology_svcs::RelabelObjectResponse& res)
{
  if (entityInstances.find(req.objName) != entityInstances.end())
  {
    // relabel instance
    entityInstances[req.objName].base_class = req.objClass;

    // get existing properties
    auto properties = ontology.getProperties(entityInstances[req.objName].instance_name);

    // reclassify in ontology
    entityInstances[req.objName].instance_name = ontology.newInstance(req.objClass);
    // populate old properties

    for (auto property : properties)
    {
      ontology.assertProperty(entityInstances[req.objName].instance_name, property.first, property.second);
    } 

    // update sim side
    ontology_svcs::RelabelObject srv;
    srv.request.objName = req.objName;
    srv.request.objClass = req.objClass;
    relabelobject.call(srv);

    // update response info
    res.newInstanceName = entityInstances[req.objName].instance_name;
    res.success = true;
  }
  else
  {
    res.reason = "Could not find object to relabel: '" + req.objName + "'";
    res.success = false;
  }
  return true;
}

bool Agent::assertNewClassCB(ontology_svcs::AssertClassRequest& req, ontology_svcs::AssertClassResponse& res)
{
  ontology.assertClass(req.className, req.parentName);

  // update local onto graph
  auto node = ontograph.add_node(req.className);
  auto pnode = ontograph.find(req.parentName);
  if (pnode == nullptr)
    pnode = ontograph.add_node(req.parentName); // if parent didn't exist, add it
  ontograph.add_edge(node, pnode);

  res.success = true;
  return true;
}

bool Agent::saveLogCB(ontology_svcs::SaveLogRequest& req, ontology_svcs::SaveLogResponse& res)
{
  static unsigned int count = 0;
  std::string countstr = std::to_string(count++);
  if (motions_L.size() > 0)
  {
    writeMotionLog(motions_L, log_dir + "/motion_log_left." + countstr + ".txt");
  }

  if (motions_R.size() > 0)
  {
    writeMotionLog(motions_R, log_dir + "/motion_log_right." + countstr + ".txt");
  }

  if (taskGraph.size() > 0)
  {
    taskGraph.toFile(log_dir + "/task_graph." + countstr + ".bag", false);
    taskGraph.toFile(log_dir + "/task_graph." + countstr + ".txt", true);
  }

  writeActivities(log_dir + "/activities." + countstr + ".txt");

  res.location = log_dir + "/<log>." + countstr;
  res.success = true;
  return true;
}

void Agent::simStateChangedCB(const ontology_msgs::SimulationStateChange::ConstPtr& msg)
{
  switch (msg->new_state)
  {
    case ontology_msgs::SimulationStateChange::SIMULATION_STARTED:
    {
      double last_time = baseTimeStamp;
      if (motions_L.size() > 0)
        last_time = motions_L.back().end_time;
      if (motions_R.size() > 0)
        last_time = motions_R.back().end_time > last_time ? motions_R.back().end_time : last_time;

      baseTimeStamp = last_time;
      break;
    }
    case ontology_msgs::SimulationStateChange::SIMULATION_STOPPED:
    {
      double last_time = baseTimeStamp;
      if (motions_L.size() > 0)
        last_time = motions_L.back().end_time;
      if (motions_R.size() > 0)
        last_time = motions_R.back().end_time > last_time ? motions_R.back().end_time : last_time;

      baseTimeStamp = last_time;
      break;
    }
    case ontology_msgs::SimulationStateChange::SIMULATION_LOADED:
    {
      break;
    }
    default:
      ROS_ERROR("Unknown Simulation State: %d", msg->new_state);
  }
}

void Agent::propertyChangedCB(const ontology_msgs::PropertyChanged::ConstPtr& msg)
{
  ontology.assertProperty(entityInstances[msg->instanceName].instance_name, msg->propertyName, msg->propertyValue);
}

void Agent::objPoseUpdateCB(const ontology_msgs::ObjUpdate::ConstPtr& msg)
{
  entityPositions[msg->name] = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
}

void Agent::updateUI()
{
  ontology_msgs::DisplayLabel msg;
  if (motions_L.size() > 0)
    msg.text = motions_L.back().classification + "\n" + motions_L.back().obj_actedOn;
  else
    msg.text = UNKNOWN;
  leftLabel_pub.publish(msg);

  if (motions_R.size() > 0)
    msg.text = motions_R.back().classification + "\n" + motions_R.back().obj_actedOn;
  else
    msg.text = UNKNOWN;
  rightLabel_pub.publish(msg);

  wrefresh(stdscr);
  clear();
  if (motions_L.size() < 1)
  {
    attron(colors.Yellow);
    write(stdscr, 10, 10, "Waiting for end-effector data...");
    return;
  }
  else
  {
    attron(colors.White);
    write(stdscr, 2, 2, "Left Hand: ");
    attron(colors.Yellow);
    write(stdscr, motions_L.back().effectorName);
    attron(colors.White);
    wmove(stdscr, 3, 2);
    whline(stdscr, ACS_HLINE, 36);

    writeToggle(stdscr, 4, 4, " Moving ", motions_L.back().moving);
    writeToggle(stdscr, " Tool Use ", motions_L.back().toolUse);

    attron(colors.Grey);
    write(stdscr, "    Segment " + std::to_string(motions_L.size()) + ": " + std::to_string(motions_L.back().end_time - motions_L.back().start_time) + "s");

    attron(colors.White);
    write(stdscr, 5, 2, " Velocity: (" + 
                      std::to_string(motions_L.back().velocity.x()) + ", " +
                      std::to_string(motions_L.back().velocity.y()) + ", " +
                      std::to_string(motions_L.back().velocity.z()) + ")"
         );

    write(stdscr, 6, 2, " Position: (" +
                      std::to_string(entityPositions[motions_L.back().effectorName].x()) + ", " +
                      std::to_string(entityPositions[motions_L.back().effectorName].y()) + ", " +
                      std::to_string(entityPositions[motions_L.back().effectorName].z()) + ")"
          );

    attron(colors.Green);
    write(stdscr, 7, 2, " Activity: " + motions_L.back().classification);

    // Object Acted on
    int acted_on_h = 9;
    int l_obj_offset = 5;
    attron(colors.White);
    write(stdscr, acted_on_h++, 2, "   Obj Acted On:");
    wmove(stdscr, acted_on_h++, 4);
    whline(stdscr, ACS_HLINE, 50);

    if (motions_L.back().obj_actedOn == NONE)
      attron(colors.GreyHighlight);
    else
      attron(colors.Yellow);
    write(stdscr, acted_on_h++, l_obj_offset, motions_L.back().obj_actedOn);

    if (motions_L.back().obj_actedOn != NONE)
    {
      attron(colors.GreenHighlight);
      std::string className = entityInstances[motions_L.back().obj_actedOn].instance_name;
      std::string shortName = ontology.getShortName(className);
      write(stdscr, acted_on_h, l_obj_offset, shortName);

      auto res = ontology.getProperties(className);
      for (auto p : res)
      {
        acted_on_h++;
        attron(colors.White);
        write(stdscr, acted_on_h, l_obj_offset, ontology.getShortName(p.first) + " : ");
        attron(colors.Green);
        write(stdscr, ontology.getShortName(p.second));
      }
    }

    // Object in Hand
    attron(colors.White);
    write(stdscr, acted_on_h + 2, 5, "Obj In Hand:");
    wmove(stdscr, acted_on_h + 3, 4);
    whline(stdscr, ACS_HLINE, 50);
    if (motions_L.back().obj_inHand == NONE)
      attron(colors.GreyHighlight);
    else
      attron(colors.Yellow);
    write(stdscr, acted_on_h + 4, l_obj_offset, motions_L.back().obj_inHand);

    if (motions_L.back().obj_inHand != NONE)
    {
      attron(colors.GreenHighlight);
      int base_h = acted_on_h + 5;
      std::string className = entityInstances[motions_L.back().obj_inHand].instance_name;
      std::string shortName = ontology.getShortName(className);
      write(stdscr, base_h, l_obj_offset, shortName);

      auto res = ontology.getProperties(className);
      for (auto p : res)
      {
        base_h++;
        attron(colors.White);
        write(stdscr, base_h, l_obj_offset, ontology.getShortName(p.first) + " : ");
        attron(colors.Green);
        write(stdscr, ontology.getShortName(p.second));
      }
    }
  }

  int r_col_offset = 57;
  if (motions_R.size() < 1)
  {
    attron(colors.Yellow);
    write(stdscr, r_col_offset + 10, 10, "Waiting for end-effector data...");
    return;
  }
  else
  {
    attron(colors.White);
    write(stdscr, 2, r_col_offset + 2, "Right Hand: ");
    attron(colors.Yellow);
    write(stdscr, motions_R.back().effectorName);
    attron(colors.White);
    wmove(stdscr, 3, r_col_offset + 2);
    whline(stdscr, ACS_HLINE, 36);

    writeToggle(stdscr, 4, r_col_offset + 4, " Moving ", motions_R.back().moving);
    writeToggle(stdscr, " Tool Use ", motions_R.back().toolUse);

    attron(colors.Grey);
    write(stdscr, "    Segment " + std::to_string(motions_R.size()) + ": " + std::to_string(motions_R.back().end_time - motions_R.back().start_time) + "s");
    
    attron(colors.White);
    write(stdscr, 5, r_col_offset + 2, " Velocity: (" + 
                      std::to_string(motions_R.back().velocity.x()) + ", " +
                      std::to_string(motions_R.back().velocity.y()) + ", " +
                      std::to_string(motions_R.back().velocity.z()) + ")"
         );

    write(stdscr, 6, r_col_offset + 2, " Position: (" +
                      std::to_string(entityPositions[motions_R.back().effectorName].x()) + ", " +
                      std::to_string(entityPositions[motions_R.back().effectorName].y()) + ", " +
                      std::to_string(entityPositions[motions_R.back().effectorName].z()) + ")"
          );

    attron(colors.Green);
    write(stdscr, 7, r_col_offset + 2, " Activity: " + motions_R.back().classification);

    // obj acted on
    int acted_on_h = 9;
    int l_obj_offset = 5 + r_col_offset;
    attron(colors.White);
    write(stdscr, acted_on_h++, r_col_offset + 2, "   Obj Acted On:");
    wmove(stdscr, acted_on_h++, r_col_offset + 4);
    whline(stdscr, ACS_HLINE, 50);

    if (motions_R.back().obj_actedOn == NONE)
      attron(colors.GreyHighlight);
    else
      attron(colors.Yellow);
    write(stdscr, acted_on_h++, l_obj_offset, motions_R.back().obj_actedOn);


    if (motions_R.back().obj_actedOn != NONE)
    {
      attron(colors.GreenHighlight);
      std::string className = entityInstances[motions_R.back().obj_actedOn].instance_name;
      std::string shortName = ontology.getShortName(className);
      write(stdscr, acted_on_h, l_obj_offset, shortName);

      auto res = ontology.getProperties(className);
      for (auto p : res)
      {
        acted_on_h++;
        attron(colors.White);
        write(stdscr, acted_on_h, l_obj_offset, ontology.getShortName(p.first) + " : ");
        attron(colors.Green);
        write(stdscr, ontology.getShortName(p.second));
      }
    }

    // obj in hand
    attron(colors.White);
    write(stdscr, acted_on_h + 2, r_col_offset + 5, "Obj In Hand:");
    wmove(stdscr, acted_on_h + 3, r_col_offset + 4);
    whline(stdscr, ACS_HLINE, 50);
    
    if (motions_R.back().obj_inHand == NONE)
      attron(colors.GreyHighlight);
    else
      attron(colors.Yellow);
    write(stdscr, acted_on_h + 4, l_obj_offset, motions_R.back().obj_inHand);
    
    if (motions_R.back().obj_inHand != NONE)
    {
      attron(colors.GreenHighlight);
      int base_h = acted_on_h + 5;
      std::string className = entityInstances[motions_R.back().obj_inHand].instance_name;
      std::string shortName = ontology.getShortName(className);
      write(stdscr, base_h, l_obj_offset, shortName);

      auto res = ontology.getProperties(className);
      for (auto p : res)
      {
        base_h++;
        attron(colors.White);
        write(stdscr, base_h, l_obj_offset, ontology.getShortName(p.first) + " : ");
        attron(colors.Green);
        write(stdscr, ontology.getShortName(p.second));
      }
    }
  }

  wmove(stdscr, 0, 56);
  attron(colors.Grey);
  wvline(stdscr, ACS_VLINE, 999);
  attron(colors.White);
  wmove(stdscr, 25, 1);
  whline(stdscr, ACS_HLINE, 999);
  write(stdscr, 26, 10, "Known Activities: " + std::to_string(knownActivities.size()));
  write(stdscr, 26, 70, "Current sim time: " + std::to_string(motions_L.back().end_time) + "s");
}

void Agent::write(WINDOW* wnd, std::string str)
{
  waddstr(wnd, str.c_str());
}

void Agent::write(WINDOW* wnd, int y, int x, std::string str)
{
  wmove(wnd, y, x);
  write(wnd, str);
}

void Agent::writeToggle(WINDOW* wnd, std::string str, bool enabled)
{
  if (enabled)
    attron(colors.GreenHighlight);
  else
    attron(colors.GreyHighlight);

  waddstr(wnd, str.c_str());
}

void Agent::writeToggle(WINDOW* wnd, int y, int x, std::string str, bool enabled)
{
  wmove(wnd, y, x);
  writeToggle(wnd, str, enabled);
}

void Agent::init_curses()
{
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  start_color();

  // low-color fallback
  if (COLORS < 255)
  {
    std::cout << "Your terminal is reporting support for fewer than 256 colors. Falling back to 8-color mode." << std::endl;
    std::cout << "For full-color support ensure $TERM is set to a 256-color terminal" << std::endl;
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_BLUE,  COLOR_BLACK);
    init_pair(3, COLOR_WHITE, COLOR_GREEN);
    init_pair(4, COLOR_WHITE,COLOR_RED);
    init_pair(5, COLOR_YELLOW,COLOR_BLACK);
    init_pair(6, COLOR_GREEN, COLOR_BLACK);
  }
  else
  {
    init_pair(1, 255, 0);
    init_pair(2, 240, 0);
    init_pair(3, 122, 24);
    init_pair(4, 240, 244);
    init_pair(5, 172, 0);
    init_pair(6, 122, 0);
  }

  colors.White          = COLOR_PAIR(1);
  colors.Grey           = COLOR_PAIR(2);
  colors.GreyHighlight  = COLOR_PAIR(4);
  colors.GreenHighlight = COLOR_PAIR(3);
  colors.Green          = COLOR_PAIR(6);
  colors.Yellow         = COLOR_PAIR(5);
}

void Agent::checkMoving(MotionSegment& m)
{
  if (m.velocity.norm() > MOVE_MIN_VEL)
    m.moving = true;
  else
    m.moving = false;
}

void Agent::checkToolUse(MotionSegment& m)
{
  if (m.obj_actedOn != NONE && m.obj_inHand != NONE)
    m.toolUse = true;
  else
    m.toolUse = false;
}

void Agent::checkActedOn(MotionSegment& m, Eigen::Vector3d currPos)
{
  // stationary objects cannot act on others
  if (m.moving)
  {
    float minDist = 9999999.0f;
    std::string minObj = NONE;
   
    // get hand effector names if we have them 
    std::string hand1, hand2;
    if (motions_L.size() > 0)
      hand1 = motions_L.back().effectorName;
    if (motions_R.size() > 0)
      hand2 = motions_R.back().effectorName;

    for (auto p : entityPositions)
    {
      // don't consider pairs of objects sharing the same class
      if (entityInstances[p.first].base_class == entityInstances[m.obj_inHand].base_class)
        continue;
      // don't consider distance to obj in hand
      if (p.first == m.obj_inHand)
        continue;
      // don't consider distance to our own hands
      if (hand1 == p.first)
        continue;
      if (hand2 == p.first)
        continue;
      // don't consider distance to ourself
      if (p.first == m.effectorName)
        continue;

      float d = (p.second - currPos).squaredNorm();
      if (d < minDist)
      {
        // only take new minimum if we're moving closer to the object
        double d2 = (p.second - (currPos+m.velocity.normalized()*0.001f)).squaredNorm();
        if (d2 < d)
        {
          minDist = d;
          minObj = p.first;
        }
      }
    }

    if (minDist < ACT_MIN_DIST)
    {
      m.obj_actedOn = minObj;
    }
    else
    {
      m.obj_actedOn = NONE;
    }
  }
}

void Agent::checkObjProperties(MotionSegment& m)
{
  // decided this was unnecessary in discussion w/ Karinne
  return;

  // capture current properties of inhand/actedon objects
  if (m.obj_inHand != NONE)
  {
    auto inhand_props = ontology.getProperties(entityInstances[m.obj_inHand].instance_name);
    for (auto p : inhand_props)
    {
      m.obj_inHand_properties[p.first] = p.second;
    }
  }
  if (m.obj_actedOn != NONE)
  {
    auto actedon_props = ontology.getProperties(entityInstances[m.obj_actedOn].instance_name);
    for (auto p : actedon_props)
    {
      m.obj_actedOn_properties[p.first] = p.second;
    }
  }
}

void Agent::appendToMotionLog(MotionSegment& m, std::vector<MotionSegment>& log)
{
  if (log.size() == 0)
  {
    log.push_back(m);
  }
  else
  {
    // current motion is continuation of previous
    if (m.Matches(log.back()))
    {
      log.back().end_time = m.end_time;
      log.back().velocity = m.velocity;

      // if this segment has been going on long enough, classify it
      if (log.back().classification == UNKNOWN_ACTIVITY && log.back().Duration() > MIN_ACTIVITY_DURATION)
      {
        log.back().classification = classify(log.back());
        log.back().fresh = true;
      }
    }
    else // current motion is new
    {
      if (log.back().Duration() < MIN_ACTIVITY_DURATION)
      {
        log.pop_back();
        if (log.size() > 0 && m.Matches(log.back()))
        {
          log.back().end_time = m.end_time;
          log.back().velocity = m.velocity;
        }
        else
        {
          log.push_back(m);
        }
      }
      else
      {
        log.push_back(m);
      }
    }
  }
}

Activity& Agent::processUnknownActivity(MotionSegment& m)
{
  if (mindState == MindState::Observing) // we're looking for a specific activity to be demonstrated
  {
    bool exists = false;
    // check to see if we already have info about this activity
    for (auto& a : knownActivities)
    {
      if (a.name == targetActivity)
      {
        exists = true;
        auto inHand_classes = ontology.getSubclassesOfClass(entityInstances[m.obj_inHand].base_class);
        auto actOn_classes = ontology.getSubclassesOfClass(entityInstances[m.obj_actedOn].base_class);

        for (auto c : inHand_classes)
        {
          a.SetProperty("inHand",
                        ActivityPropertyType::IsSubclassOf,
                        ontology.getShortName(c)
                        );
        }
        for (auto c : actOn_classes)
        {
          a.SetProperty("actedOn",
                        ActivityPropertyType::IsSubclassOf,
                        ontology.getShortName(c)
                        );
        }
/* // Removed after discussion w/ Karinne
        // instance properties
        auto inHand_properties = ontology.getProperties(entityInstances[m.obj_inHand].instance_name);
        auto actedOn_properties = ontology.getProperties(entityInstances[m.obj_actedOn].instance_name);

        for (auto p : inHand_properties)
        {
          std::vector<std::string> vals = {p.first, p.second};
          a.SetProperty(S_INHAND,
                        ActivityPropertyType::HasValue,
                        vals
                       );
        }
        for (auto p : actedOn_properties)
        {
          std::vector<std::string> vals = {p.first, p.second};
          a.SetProperty(S_ACTEDON,
                        ActivityPropertyType::HasValue,
                        vals
                       );
        }
*/
        bubbleActivityProperty(a);
        return a;
      }
    }

    if (!exists)
    {
      Activity newActivity = Activity(targetActivity, "#"+targetActivity);
      newActivity.SetProperty("inHand",
                              ActivityPropertyType::IsSubclassOf,
                              "#"+entityInstances[m.obj_inHand].base_class
                              );
      newActivity.SetProperty("actedOn", 
                              ActivityPropertyType::IsSubclassOf,
                              "#"+entityInstances[m.obj_actedOn].base_class
                              );
/* // removed after conversation w/ Karinne
      // instance properties
      auto inHand_properties = ontology.getProperties(entityInstances[m.obj_inHand].instance_name);
      auto actedOn_properties = ontology.getProperties(entityInstances[m.obj_actedOn].instance_name);

      for (auto p : inHand_properties)
      {
        std::vector<std::string> vals = {p.first, p.second};
        newActivity.SetProperty(S_INHAND,
                      ActivityPropertyType::HasValue,
                      vals
                     );
      }
      for (auto p : actedOn_properties)
      {
        std::vector<std::string> vals = {p.first, p.second};
        newActivity.SetProperty(S_ACTEDON,
                      ActivityPropertyType::HasValue,
                      vals
                     );
      }
*/
      bubbleActivityProperty(newActivity);
      knownActivities.push_back(newActivity);
    }

  }
  else
  {
    Activity newActivity = Activity(makeID("GranularActivity_"), "#GranularActivity");
    newActivity.SetProperty(S_INHAND,
                            ActivityPropertyType::IsSubclassOf,
                            "#"+entityInstances[m.obj_inHand].base_class
                           );
    newActivity.SetProperty(S_ACTEDON,
                            ActivityPropertyType::IsSubclassOf,
                            "#"+entityInstances[m.obj_actedOn].base_class
                           );
/* removed after conversation w/ Karinne
    // instance properties
      auto inHand_properties = ontology.getProperties(entityInstances[m.obj_inHand].instance_name);
      auto actedOn_properties = ontology.getProperties(entityInstances[m.obj_actedOn].instance_name);

      for (auto p : inHand_properties)
      {
        std::vector<std::string> vals = {p.first, p.second};
        newActivity.SetProperty(S_INHAND,
                      ActivityPropertyType::HasValue,
                      vals
                     );
      }
      for (auto p : actedOn_properties)
      {
        std::vector<std::string> vals = {p.first, p.second};
        newActivity.SetProperty(S_ACTEDON,
                      ActivityPropertyType::HasValue,
                      vals
                     );
      }
*/
    bubbleActivityProperty(newActivity);
    knownActivities.push_back(newActivity);
  }

  return knownActivities.back();
}

Activity& Agent::getActivityByName(std::string name)
{
  for (auto& a : knownActivities)
  {
    if (a.name == name)
      return a;
  }

  return unknownActivity;
}

void Agent::addTaskTransition(Activity from, Activity to)
{
  auto prev_node = taskGraph.find(from);
  auto curr_node = taskGraph.find(to);

  if (prev_node == nullptr)
   prev_node = taskGraph.add_node(from);
  if (curr_node == nullptr)
   curr_node = taskGraph.add_node(to);

  if (taskGraph.has_edge(prev_node, curr_node))
   taskGraph.set_edge_weight(prev_node,
                            curr_node,
                            taskGraph.get_edge_weight(prev_node, curr_node) + 1);
  else
   taskGraph.add_edge(prev_node, curr_node);

  taskGraph_pub.publish(taskGraph.toMessage());
}

void Agent::leftHandUpdate(const ontology_msgs::HandUpdate::ConstPtr& msg)
{
  MotionSegment curr;
  curr.effectorName = msg->handState.name;
  curr.obj_inHand   = msg->obj_inHand;
  curr.velocity.x() = msg->handState.velocity.x;
  curr.velocity.y() = msg->handState.velocity.y;
  curr.velocity.z() = msg->handState.velocity.z;

  Eigen::Vector3d currPos = {msg->handState.position.x, msg->handState.position.y, msg->handState.position.z};

  checkMoving(curr);
  checkActedOn(curr, currPos);
  checkToolUse(curr);
  checkObjProperties(curr);

  curr.start_time = getOffsetTime(msg->handState.timestamp, baseTimeStamp);
  curr.end_time = curr.start_time;
  appendToMotionLog(curr, motions_L);

  if (motions_L.back().classification == UNKNOWN_GRANULAR_ACTIVITY)
  {
    Activity& act = processUnknownActivity(motions_L.back());
    taskGraph.add_node(act);
    motions_L.back().classification = act.name;
    motions_L.back().fresh = true;
  }

  // bail before updating task transition
  if (motions_L.back().classification == UNKNOWN_ACTIVITY)
    return;

  if (taskGraph.size() == 0) // this is the first activity EVER
  {
    taskGraph.add_node(getActivityByName(classify(motions_L.back())));
  }
  else if (motions_L.size() > 1 && motions_L.back().fresh)
  {
    motions_L.back().fresh = false;
    Activity& prev = getActivityByName((*(motions_L.end()-2)).classification);
    Activity& curr = getActivityByName(motions_L.back().classification);

    if (prev != curr)
    {
      addTaskTransition(prev, curr);
    }
  }
}

void Agent::rightHandUpdate(const ontology_msgs::HandUpdate::ConstPtr& msg)
{
  MotionSegment curr;
  curr.effectorName = msg->handState.name;
  curr.obj_inHand   = msg->obj_inHand;
  curr.velocity.x() = msg->handState.velocity.x;
  curr.velocity.y() = msg->handState.velocity.y;
  curr.velocity.z() = msg->handState.velocity.z;

  Eigen::Vector3d currPos = {msg->handState.position.x, msg->handState.position.y, msg->handState.position.z};
  checkMoving(curr);
  checkActedOn(curr, currPos);
  checkToolUse(curr);
  checkObjProperties(curr);

  curr.start_time = getOffsetTime(msg->handState.timestamp, baseTimeStamp);
  curr.end_time = curr.start_time;
  appendToMotionLog(curr, motions_R);
  
  if (motions_R.back().classification == UNKNOWN_GRANULAR_ACTIVITY)
  {
    Activity& act = processUnknownActivity(motions_R.back());
    taskGraph.add_node(act);
    motions_R.back().classification = act.name;
  }
  
  // bail before updating task transition
  if (motions_R.back().classification == UNKNOWN_ACTIVITY)
    return;
  
  if (taskGraph.size() == 0) // this is the first activity EVER
  {
    taskGraph.add_node(getActivityByName(classify(motions_R.back())));
  }
  else if (motions_R.size() > 1 && motions_R.back().fresh)
  {
    motions_R.back().fresh = false;
    Activity& prev = getActivityByName((*(motions_R.end()-2)).classification);
    Activity& curr = getActivityByName(motions_R.back().classification);

    if (prev != curr)
    {
      addTaskTransition(prev, curr);
    }
  }
}

std::vector<std::string> Agent::bubbleActivityProperty(std::string parent, std::vector<std::string> childset)
{
  std::vector<std::string> result;

  auto children = ontology.getDirectSubclassesOfClass(parent);
  std::cout << "\t" << parent << " with " << children.size() << " children" << std::endl;

  bool bubble = true;
  for (auto c : children)
  {
    auto shortname = ontology.getShortName(c);
    ROS_INFO("\t\t%s", c.c_str());
    if (std::find(childset.begin(), childset.end(), shortname) == childset.end())
    {
      bubble = false;
      break; // no bubble
    }
  }

  if (bubble)
  {
    ROS_INFO("BUBBLING %s", parent.c_str());
    result.push_back(parent); // bubble up to this parent
    childset.push_back(parent);
    auto parent_node = ontograph.find(parent);
    if (parent_node != nullptr)
    {
      auto parents = ontograph.parents(parent_node);
      for (auto p : parents)
      {
        auto next = bubbleActivityProperty(p->data, childset);
        if (next.size() > 0)
        {
          result.insert(std::end(result),
                        std::make_move_iterator(std::begin(next)),
                        std::make_move_iterator(std::end(next)));
        }
      }
    }
  }

  return result;
}

void Agent::bubbleActivityProperty(Activity& activity, std::string newClass)
{
  if (activity.HasProperty(S_INHAND))
  {
    auto bubbled_to = bubbleActivityProperty(activity.GetProperty(S_INHAND), newClass);
    for (auto& b : bubbled_to)
    {
      activity.SetProperty(S_INHAND, ActivityPropertyType::IsSubclassOf, b);
      ontology.appendProperty(b, "ActivityTool", activity.ontologyClass);
    }
  }

  if (activity.HasProperty(S_ACTEDON))
  {
    auto bubbled_to = bubbleActivityProperty(activity.GetProperty(S_ACTEDON), newClass);
    for (auto& b : bubbled_to)
    {
      activity.SetProperty(S_ACTEDON, ActivityPropertyType::IsSubclassOf, b);
      ontology.appendProperty(b, "ActivityTarget", activity.ontologyClass);
    }
  }
}

std::vector<std::string> Agent::bubbleActivityProperty(ActivityProperty& activityProp, std::string newClass)
{
  std::vector<std::string> result;

  if (activityProp.has(ActivityPropertyType::IsSubclassOf))
  {
    auto node = ontograph.find(newClass);
    if (node != nullptr)
    {
      auto parents = ontograph.parents(node);
      for (auto& p : parents)
      {
        auto bubble_to = bubbleActivityProperty(p->data, activityProp.values[ActivityPropertyType::IsSubclassOf]);
        if (bubble_to.size() > 0)
        {
          result.insert(std::end(result),
                        std::make_move_iterator(std::begin(bubble_to)),
                        std::make_move_iterator(std::end(bubble_to)));
        }
      }
    }
  }

  return result;
}

void Agent::bubbleActivityProperty(Activity& activity)
{
  // handle tool use
  if (activity.HasProperty("inHand"))
  {
    auto property = activity.GetProperty("inHand");
    if (property.has(ActivityPropertyType::IsSubclassOf))
    {
      for (auto& v : property.values[ActivityPropertyType::IsSubclassOf])
      {
        auto node = ontograph.find(v);
        if (node != nullptr)
        {
          auto parents = ontograph.parents(node);
          for (auto& p : parents)
          {
            auto bubble_to = bubbleActivityProperty(p->data, property.values[ActivityPropertyType::IsSubclassOf]);

            if (bubble_to.size() > 0)
            {
              for (auto b : bubble_to)
              {
                ROS_INFO("BUBBLNG %s up to %s", "ActivityTool", p->data.c_str());
                activity.SetProperty("inHand", ActivityPropertyType::IsSubclassOf, b);
                ontology.appendProperty(b, "ActivityTool", activity.ontologyClass);
              }
            }
          }
        }
      }
    }
  }

  // handle tool target
  if (activity.HasProperty("actedOn"))
  {
    auto property = activity.GetProperty("actedOn");
    if (property.has(ActivityPropertyType::IsSubclassOf))
    {
      for (auto& v : property.values[ActivityPropertyType::IsSubclassOf])
      {
        auto node = ontograph.find(v);
        if (node != nullptr)
        {
          auto parents = ontograph.parents(node);
          for (auto& p : parents)
          {
            auto bubble_to = bubbleActivityProperty(p->data, property.values[ActivityPropertyType::IsSubclassOf]);

            if (bubble_to.size() > 0)
            {
              for (auto b : bubble_to)
              {
                activity.SetProperty("actedOn", ActivityPropertyType::IsSubclassOf, b);
                ontology.appendProperty(b, "ActivityTarget", activity.ontologyClass);
              }
            }
          }
        }
      }
    }
  }
}

Graph<std::string> Agent::getActivityGraph(Activity& activity)
{
  Graph<std::string> g;

  auto root = g.add_node("<Activity> " + activity.name);
  auto hand = g.add_node("<IN HAND>");
  auto acto = g.add_node("<ACT ON>");
  g.add_edge(hand, root);
  g.add_edge(acto, root);

  if (activity.HasProperty(S_INHAND))
  {
    for (auto c : activity.GetProperty(S_INHAND).values[ActivityPropertyType::IsSubclassOf])
    {
      Graph<std::string>::FromOntology(c, ontology, g);
      auto node = g.find(c);
      g.add_edge(node, hand);
    }
  }
  if (activity.HasProperty(S_ACTEDON))
  {
    for (auto c : activity.GetProperty(S_ACTEDON).values[ActivityPropertyType::IsSubclassOf])
    {
      Graph<std::string>::FromOntology(c, ontology, g);
      auto node = g.find(c);
      g.add_edge(node, acto);
    }
  }

  return g;
}

void Agent::run()
{
  targetActivity = "WashDishes";
  //mindState = MindState::Observing;

  // if we have no activity knowledge at all,
  // fill in at least a few basic primitive actions
  if (knownActivities.size() == 0)
  {
    // basic activities w/ no properties or constraints
    knownActivities.push_back(Activity("IdleMotion", "#IdleMotion"));
    knownActivities.push_back(Activity("Reach", "#Reach"));
    knownActivities.push_back(Activity("Take", "#Take"));
    knownActivities.push_back(Activity("Release", "#Release"));
    knownActivities.push_back(Activity("PutSomethingSomewhere", "#PutSomethingSomewhere"));

    // default known granular activities
    Activity dishWashing("WashDishes", "#WashingDishes");
    dishWashing.SetProperty("inHand", ActivityPropertyType::IsSubclassOf, "#CleaningImplement");
    dishWashing.SetProperty("actedOn", ActivityPropertyType::IsSubclassOf, "#FoodVessel");
    knownActivities.push_back(dishWashing);
  }

  ROS_INFO("Setting up ROS connection & handlers...");
  ros::NodeHandle n;

  std::string log_prefix;
  ros::param::get("~log_prefix", log_prefix);
  if (log_prefix.empty())
    log_prefix = "log_";

  ROS_INFO("Creating log directory: %s", (log_dir = makeStampedDirectory(log_prefix)).c_str());
  
  ros::Subscriber simStateChanged_sub = n.subscribe("/sgv/simulation_state_changed",
                                                    1000,
                                                    &Agent::simStateChangedCB,
                                                    this);
  ros::Subscriber propertyChanged_sub = n.subscribe("/sgv/onto/property_changed",
                                                    1000,
                                                    &Agent::propertyChangedCB,
                                                    this);
  ros::Subscriber objUpdate_sub = n.subscribe("/sgv/onto/object_moved",
                                                    1000,
                                                    &Agent::objPoseUpdateCB,
                                                    this);
  ontolist   = n.serviceClient<ontology_svcs::GetAllClassInstancesWithPoses>("/sgv/onto/GetAllClassInstancesWithPoses");
  relabelobject = n.serviceClient<ontology_svcs::RelabelObject>("/sgv/sim/RelabelObject");

  ros::Subscriber leftHand_sub  = n.subscribe("/sgv/human/hands/left",  1000, &Agent::leftHandUpdate,  this);
  ros::Subscriber rightHand_sub = n.subscribe("/sgv/human/hands/right", 1000, &Agent::rightHandUpdate, this);

  leftLabel_pub  = n.advertise<ontology_msgs::DisplayLabel>("/sgv/human/hands/left/label", 5);
  rightLabel_pub = n.advertise<ontology_msgs::DisplayLabel>("/sgv/human/hands/right/label", 5);

  auto mind_svc = n.advertiseService("/sgv/bot/mindgoal", &Agent::instructBehaviorCB, this);
  auto act_svc  = n.advertiseService("/sgv/bot/getActivities", &Agent::getActivitiesCB, this);
 
  getactivitygraph_srv = n.advertiseService("/sgv/bot/getActivityGraph", &Agent::getActivityGraphCB, this);
  relabelobject_srv    = n.advertiseService("/sgv/bot/relabelObject", &Agent::relabelObjectCB, this);
  relabelactivity_srv  = n.advertiseService("/sgv/bot/relabelActivity", &Agent::relabelActivityCB, this);
  assertnewclass_srv   = n.advertiseService("/sgv/bot/assertNewClass", &Agent::assertNewClassCB, this);
  savelog_srv          = n.advertiseService("/sgv/bot/saveLog", &Agent::saveLogCB, this);

  ros::Publisher graph_update_pub = n.advertise<ontology_msgs::GraphUpdate>("/sgv/onto/graph/updates", 5);
  taskGraph_pub                   = n.advertise<ontology_msgs::Graph>("/sgv/tasks/graph", 5);

  ROS_INFO("Populating local graph from ontology...");
  Graph<std::string>::FromOntology("#SpatialThing", ontology, ontograph);

  ROS_INFO("Checking for services...");
  if (!ros::service::exists("/sgv/onto/GetAllClassInstancesWithPoses", true))
  {
    ROS_INFO("Waiting for requisite services to come online...");
    if (!ros::service::waitForService("/sgv/onto/GetAllClassInstancesWithPoses"))
    {
      ROS_ERROR("Unrecoverable failure.");
      return;
    }
  }

  ROS_INFO("Requesting all objects with ontology classes...");

  ontology_svcs::GetAllClassInstancesWithPoses srv;
  if (ontolist.call(srv))
  {
    ROS_INFO("Creating KnowROB instances of objects in simulation...");
    for (int i = 0; i < srv.response.classes.size(); ++i)
    {
      std::string new_id = ontology.newInstance(srv.response.classes[i]);
      entityInstances.emplace(srv.response.entityNames[i], EntityInstance(new_id, srv.response.classes[i]));
      entityPositions.emplace(srv.response.entityNames[i],
                              Eigen::Vector3d(srv.response.positions[i].x,
                                              srv.response.positions[i].y,
                                              srv.response.positions[i].z)
      );

      for (auto property : srv.response.properties[i].properties)
      {
        ontology.assertProperty(new_id, property.name, property.value);
      } 
    }
  }
  else
  {
    ROS_ERROR("Bad response? :(");
    return;
  }

  ROS_INFO("Initialization Complete!");
  ROS_INFO("Spinning up...");
  init_curses();
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    updateUI();
    loop_rate.sleep();
  }
}

