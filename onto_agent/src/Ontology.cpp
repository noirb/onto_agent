#include "Ontology.hpp"

using namespace json_prolog;

std::string Ontology::newInstance(std::string className)
{
  std::string instanceName;
  PrologQueryProxy bdgs = pl.query("rdf_instance_from_class(knowrob:'" + className + "', NewInstance)");

  for (PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    for (auto val : *it)
    {
      instanceName = val.second.toString();
    }
  }

  bdgs.finish();

  return instanceName;
}

void Ontology::assertClass(std::string className, std::string parentName)
{
  PrologQueryProxy bdgs = pl.query("rdf_assert(knowrob:'" + className + "', rdfs:subClassOf, knowrob:'" + parentName + "')");
  bdgs.finish();
}

void Ontology::clearProperty(std::string instance, std::string property)
{
  PrologQueryProxy bdgs = pl.query("rdf_retractall('" + instance + "', knowrob:'" + property + "', V)");
  bdgs.finish();
}

void Ontology::assertProperty(std::string instance, std::string property, std::string value)
{
  // clear old value for property if it exists
  clearProperty(instance, property);

  // assert new value
  PrologQueryProxy bdgs = pl.query("rdf_assert('" + instance + "', knowrob:'" + property + "', knowrob:'" + value + "')");
  bdgs.finish();
}

void Ontology::appendProperty(std::string instance, std::string property, std::string value)
{
  // get current values
  std::string current_value;
  auto properties = this->getProperties(instance);
  for (auto p : properties)
  {
    if (p.first.find(property) != std::string::npos) // instance already has property!
    {
      size_t field_start = p.second.find('#');
      if (field_start != std::string::npos)
        current_value = p.second.substr(field_start+1);
      else
        current_value = p.second;
      break;
    }
  }

  // don't add value if it's already attached to the property
  if (current_value.length() == 0)
  {
    current_value = value;
  }
  else if (current_value.find(value) == std::string::npos)
  {
    current_value += "," + value;
  }

  // add new value
  assertProperty(instance, property, current_value);
}

std::map<std::string, std::string> Ontology::getProperties(std::string instance)
{
  std::map<std::string, std::string> result;
  PrologQueryProxy bdgs = pl.query("owl_has('" + instance + "', P, O)");
  for (PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    result.emplace((*it)["P"].toString(), (*it)["O"].toString());
  }
  bdgs.finish();
  return result;
}

std::vector<std::string> Ontology::getIndividuals(std::string type)
{
  std::vector<std::string> result;
  PrologQueryProxy bdgs = pl.query("owl_individual_of(A, knowrob:'" + type + "')");
  for (auto& it : bdgs)
  {
    result.push_back(it["A"]);
  }
  bdgs.finish();
  return result;
}

std::vector<std::string> Ontology::getClassesFromInstance(std::string instanceName)
{
  std::vector<std::string> result;
  PrologQueryProxy bdgs = pl.query("owl_individual_of('" + instanceName + "', C)");
  for (auto& it : bdgs)
  {
    result.push_back(it["C"]);
  }
  bdgs.finish();
  return result;
}

std::vector<std::string> Ontology::getSubclassesOfClass(std::string className)
{
  std::vector<std::string> result;
  std::string classQuery;
  if (isShortName(className))
    classQuery = "knowrob:'" + className.substr(1) + "'";
  else
    classQuery = "'" + className + "'";

  PrologQueryProxy bdgs = pl.query("owl_subclass_of(A, " + classQuery + ")");
  for (auto& it : bdgs)
  {
    result.push_back(it["A"]);
  }
  bdgs.finish();
  return result;
}

std::vector<std::string> Ontology::getDirectSubclassesOfClass(std::string className)
{
  std::vector<std::string> result;
  std::string classQuery;
  if (isShortName(className))
    classQuery = "knowrob:'" + className.substr(1) + "'";
  else
    classQuery = "'" + className + "'";

  PrologQueryProxy bdgs = pl.query("owl_direct_subclass_of(A, " + classQuery + ")");
  for (auto& it : bdgs)
  {
    result.push_back(it["A"]);
  }
  bdgs.finish();
  return result;
}

bool Ontology::isSubclassOf(std::string instance, std::string baseClass)
{
  std::vector<std::string> classes = getClassesFromInstance(instance);
  for (auto& c : classes)
  {
    // TODO: Try to find a way to reliably pull this info from the ontology
    if (c.find(baseClass) != std::string::npos)
    {
      return true;
    }
  }
  return false;

  std::string classQuery;
  if (isShortName(baseClass))
    classQuery = "knowrob:'" + baseClass.substr(1) + "'";
  else
    classQuery = "'" + baseClass + "'";

  PrologQueryProxy bdgs = pl.query("owl_subclass_of('" + instance + "', " + classQuery + ")");
  bool res = false;
  for (auto& it : bdgs)
  {
    res = true;
    break;      /// TODO: There has GOT to be a better way to test for success here...
  }
  bdgs.finish();
  return res;
}

std::string Ontology::getShortName(std::string iri)
{
    auto split_pos = iri.find('#');
    if (split_pos != std::string::npos)
    {
      return iri.substr(split_pos);
    }
    else
    {
      return iri;
    }
}

std::string Ontology::getShorterName(std::string iri)
{
  auto split_pos = iri.find('#');
  if (split_pos != std::string::npos)
  {
    return iri.substr(split_pos+1);
  }
  else
  {
    return iri;
  }
}

bool Ontology::isShortName(std::string name)
{
  if (name[0] == '#')
    return true;
  else
    return false;
}

