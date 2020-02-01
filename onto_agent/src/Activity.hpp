#ifndef _ACTIVITY_H_
#define _ACTIVITY_H_

#include <map>
#include <string>
#include <vector>
#include <stdexcept>

#include "Constants.hpp"
#include "Ontology.hpp"

enum ActivityPropertyType
{
  IsSubclassOf,
  IsClass,
  IsNotSubclassOf,
  IsNotClass,
  HasValue,
  DoesNotHaveValue,
  InvalidActivityPropertyType
};

static std::string ActivityPropertyTypeName(ActivityPropertyType t)
{
  switch(t)
  {
    case IsSubclassOf:
      return "IsSubclassOf";
    case IsClass:
      return "IsClass";
    case IsNotSubclassOf:
      return "IsNotSubclassOf";
    case IsNotClass:
      return "IsNotClass";
    case HasValue:
      return "HasValue";
    case DoesNotHaveValue:
      return "DoesNotHaveValue";
    default:
      return "InvalidActivityPropertyType!";
  }
}

static ActivityPropertyType ActivityPropertyFromName(std::string name)
{
  if (name == "IsSubclassOf")
    return ActivityPropertyType::IsSubclassOf;
  else if (name == "IsClass")
    return ActivityPropertyType::IsClass;
  else if (name == "IsNotSubclassOf")
    return ActivityPropertyType::IsNotSubclassOf;
  else if (name == "IsNotClass")
    return ActivityPropertyType::IsNotClass;
  else if (name == "HasValue")
    return ActivityPropertyType::HasValue;
  else if (name == "DoesNotHaveValue")
    return ActivityPropertyType::DoesNotHaveValue;
  else
    return ActivityPropertyType::InvalidActivityPropertyType;
}

struct ActivityProperty
{
  std::map<ActivityPropertyType, std::vector<std::string> > values;

  void append(ActivityPropertyType type, std::string value)
  {
    this->values[type].push_back(value);
  }

  void append(ActivityPropertyType type, std::vector<std::string> value)
  {
    this->values[type].insert(std::end(this->values[type]), std::begin(value), std::end(value));
  }

  bool has(ActivityPropertyType type)
  {
    return values.find(type) != values.end();
  }

  bool has(ActivityPropertyType type, std::string value)
  {
    if (!has(type))
      return false;
    return std::find(values[type].begin(), values[type].end(), value) != values[type].end();
  }

  bool operator==(const ActivityProperty& other) const
  {
    if (values.size() != other.values.size())
      return false;

    for (auto v : values)
    {
      if (other.values.find(v.first) == other.values.end())
        return false;
    }

    return true;
  }

  bool operator!=(const ActivityProperty& other) const
  {
    return !(*this == other);
  }
};

struct Activity
{
  std::string name;
  std::string ontologyClass;
  std::map<std::string, ActivityProperty> properties;

public:

  Activity(std::string Name, std::string OntologyClass) : name(Name), ontologyClass(OntologyClass)
  {
  }

  void SetProperty(std::string property, ActivityPropertyType type, std::string value)
  {
    if (this->HasProperty(property) && properties[property].has(type))
    {
      if (!properties[property].has(type, value))
        properties[property].append(type, value);
    }
    else
    {
      properties[property].append(type, value);
    }
  }

  void SetProperty(std::string property, ActivityPropertyType type, std::vector<std::string> values)
  {
    properties[property].append(type, values);
  }

  bool HasProperty(std::string property)
  {
    return properties.find(property) != properties.end();
  }

  ActivityProperty& GetProperty(std::string property)
  {
    if (this->HasProperty(property))
    {
      return properties[property];
    }
    throw new std::runtime_error("ActivityProperty " + property + " does not exist!");
  }

  bool operator ==(const Activity& other) const
  {
    if (name != other.name)
      return false;

    if (properties.size() != other.properties.size())
      return false;

    for (auto p : properties)
    {
      if (std::find(other.properties.begin(), other.properties.end(), p) == other.properties.end())
        return false;
    }
    return true;
  }

  bool operator !=(const Activity& other) const
  {
    return !(*this == other);
  }

  operator std::string() {
    std::string out = "";
    out += name;
    if (this->HasProperty(S_INHAND) && this->HasProperty(S_ACTEDON))
    {
      out += " inHand { ";
      for (auto p : properties[S_INHAND].values)
      {
        out += ActivityPropertyTypeName(p.first) + " ( ";
        for (size_t i = 0; i < p.second.size(); i++)
        {
          out += p.second[i];
          if (i < p.second.size() - 1)
            out += ", ";
        }
        out += " ) ";
      }
      out += " }";

      out += " actedOn { ";
      for (auto p : properties[S_ACTEDON].values)
      {
        out += ActivityPropertyTypeName(p.first) + " ( ";
        for (size_t i = 0; i < p.second.size(); i++)
        {
          out += p.second[i];
          if (i < p.second.size() - 1)
            out += ", ";
        }
        out += " ) ";
      }
      out += " }";
    }
    return out;
    /*
    out += name + "[" + ontologyClass + "]";
    out += " || ";
    for (auto p : properties)
    {
      out += p.first + " -> ";
      out += ActivityPropertyTypeName(p.second.type) + "( ";
      for (auto it : p.second.values)
      {
        out += it;
        out += ", ";
      }
      out += " )";

      if (p != *(properties.rbegin()))
      {
        out += " : ";
      }
    }

    return out;
    */
  }
};

#endif // _ACTIVITY_H_
