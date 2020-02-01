#ifndef _SGV_UTILS_H_
#define _SGV_UTILS_H_

#define USE_BOOST_FILESYSTEM

#ifdef USE_BOOST_FILESYSTEM
#include <boost/filesystem.hpp>
#endif

#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <ctime>

#include "Activity.hpp"

std::vector<std::string> static tokenize(std::string s)
{
  std::vector<std::string> tokens;
  std::string curtok = "";
  for (char c : s)
  {
    if (c == ' ')
    {
      // end current token
      if (!curtok.empty())
      {
        tokens.push_back(curtok);
        curtok = "";
      }
    }
    else if (c == ',')
    {
      // end current token & add ',' as new
      if (!curtok.empty())
      {
        tokens.push_back(curtok);
      }
      curtok = c;
    }
    else
    {
      // extend current token
      curtok += c;
    }
  }

  // last token
  if (!curtok.empty())
    tokens.push_back(curtok);

  return tokens;
}

Activity static parseActivity(std::string text)
{
  std::vector<std::string> tokens = tokenize(text);
  if (tokens.size() < 1)
    throw std::runtime_error("Expected to find activity name, got nothing");

  Activity activity = Activity(tokens[0], "#"+tokens[0]);

  // not a basic activity
  if (tokens.size() > 1)
  {
    std::string curProp;
    std::string curConstraint;
    for (size_t i = 1; i < tokens.size(); i++)
    {
      if (tokens[i] == S_INHAND || tokens[i] == S_ACTEDON)
      {
        curProp = tokens[i];
      }
      else if (tokens[i] == "{")
      {
        // getting constraints next
        continue;
      }
      else if (tokens[i] == "(")
      {
        // getting constraint values next
        continue;
      }
      else if (tokens[i] == "}")
      {
        // done with property
        curProp = "";
      }
      else if (tokens[i] == ")")
      {
        // done with constraint values for property
        curConstraint = "";
      }
      else if (tokens[i] == ",")
      {
        continue;
      }
      else
      {
        if (!curProp.empty() && curConstraint.empty())
          curConstraint = tokens[i];
        else if (!curProp.empty() && !curConstraint.empty())
          activity.SetProperty(curProp, ActivityPropertyFromName(curConstraint), tokens[i]);
        else
          throw std::runtime_error("Error encountered at token: '" + tokens[i] + "'  Activity Property & Constraints MUST be defined first!");
      }
    }
  }
  return activity;
}

std::vector<Activity> static parseActivities(std::string filename)
{
  std::vector<Activity> activities;

  std::ifstream infile(filename);
  std::string line;
  while (std::getline(infile, line))
  {
    activities.push_back(parseActivity(line));
  }

  return activities;
}

std::string static makeStampedDirectory(std::string prefix)
{
  auto now = std::time(nullptr);
  char buf[sizeof("YYYY-MM-DD_HHMMSS")];
  std::string dirname = prefix + std::string(buf, buf + std::strftime(buf, sizeof(buf), "%F_%H%M%S", std::gmtime(&now)));

#ifdef USE_BOOST_FILESYSTEM
  if (boost::filesystem::create_directories(dirname))
    return dirname;
  else
    throw std::runtime_error("Could not create directory: " + dirname);
#else
  const int dir_err = mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  if (-1 == dir_err)
  {
    throw std::runtime_error("Could not create directory: " + dirname + "\n\t\t" + strerror(errno));
  }

  return dirname;
#endif
}

#endif // _SGV_UTILS_H_
