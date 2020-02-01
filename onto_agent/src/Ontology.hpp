#ifndef _ONTOLOGY_H_
#define _ONTOLOGY_H_

#include <string>
#include <map>
#include <vector>

#include <json_prolog/prolog.h>

using namespace json_prolog;

class Ontology
{
private:
  Prolog pl = Prolog(true);

public:
  Ontology()
  {
    if (pl.waitForServer())
      pl = Prolog(true);
  }

  std::string newInstance(std::string className);

  // adds a new class entry to the ontology
  void assertClass(std::string className, std::string parentName);

  // removes the given property from the given class or instance
  void clearProperty(std::string instance, std::string property);

  // sets property of instance to value in the ontology
  void assertProperty(std::string instance, std::string property, std::string value);

  // adds value to property in the ontology
  void appendProperty(std::string instance, std::string property, std::string value);

  // retrieves all properties associated with the given class or instance
  std::map<std::string, std::string> getProperties(std::string instance);

  std::vector<std::string> getIndividuals(std::string type);

  std::vector<std::string> getSubclassesOfClass(std::string className);

  std::vector<std::string> getDirectSubclassesOfClass(std::string className);

  std::vector<std::string> getClassesFromInstance(std::string instanceName);

  bool isSubclassOf(std::string instance, std::string baseClass);

  // returns a shortened version of the given iri
  // e.g. http://ontolo.gy/onto.owl#className --> #className
  static std::string getShortName(std::string iri);
  // returns a shortened iri without a preceding '#'
  static std::string getShorterName(std::string iri);
  // checks to see if the given name is a complete iri or not
  bool isShortName(std::string name);
};

#endif // _ONTOLOGY_H
