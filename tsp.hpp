/*
* Copyright (c) 2015 Owen Glofcheski
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
*    1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
*
*    2. Altered source versions must be plainly marked as such, and must not
*    be misrepresented as being the original software.
*
*    3. This notice may not be removed or altered from any source
*    distribution.
*/

#ifndef _TSP_HPP_
#define _TSP_HPP_

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Basic Utility Types
typedef std::string CityID;
struct Coordinate {
  int x, y;
};

typedef std::unordered_set<CityID> CitySet;
typedef std::vector<CityID> CityList;
typedef std::unordered_map<std::pair<CityID, CityID>, float> CityDistanceMap;
typedef std::unordered_map<CityID, Coordinate> CityCoordinateMap;

std::ostream &operator<< (std::ostream &os, const CityList &list) {
  for (CityList::const_iterator it = list.begin(); it != list.end(); ++it) {
    if (it != list.begin()) {
      os << " -> ";
    }
    os << (*it);
  }
  return os;
}

struct CityPath {
  CityPath (const CityList &base_cities, float base_cost)
      : cities(base_cities), cost(base_cost), heuristic_cost(0) {
  }

  CityList cities;

  float cost;
  float heuristic_cost;

  bool operator< (const CityPath &other) const {
    // Sort primarily by heuristic cost, city list otherwise
    if (heuristic_cost == other.heuristic_cost) {
      return cities < other.cities;
    }
    return heuristic_cost < other.heuristic_cost;
  }

  bool operator== (const CityPath &other) const {
    return cities == other.cities;
  }
};

std::ostream &operator<< (std::ostream &os, const CityPath &path) {
  os << path.cities << std::endl;
  os << "cost: " << path.cost;
  return os;
}

typedef std::unordered_set<CityPath> CityPathSet;

struct CityEdge {
  CityEdge () : city1(""), city2(""), cost(0) {
  }

  CityEdge (const CityID &c1, const CityID &c2, float cost) 
      : city1(c1), city2(c2), cost(cost) {
  }
  
  CityID city1;
  CityID city2;

  float cost;

  bool operator< (const CityEdge &other) const {
    return cost < other.cost;
  }
};

typedef std::vector<CityEdge> CityEdgeList;

// Hash Operators
namespace std {

template<>
class hash<pair<CityID, CityID> > {
public:
  size_t operator() (const pair<CityID, CityID> &city_pair) const {
    return hash<CityID>()(city_pair.first) ^ hash<CityID>()(city_pair.second);
  }
};

template<>
class hash<CityList> {
public:
  size_t operator() (const CityList &list) const {
    size_t hash_val = 0;
    for (CityList::const_iterator it = list.begin(); it != list.end(); ++it) {
      hash_val ^= hash<CityID>()(*it);
    }
    return hash_val;
  }
};

template<>
class hash<CityPath> {
public:
  size_t operator() (const CityPath &path) const {
    return hash<CityList>()(path.cities);
  }
};

} // namespace std

/*
 * Returns the euclidean distance between two cities in a given CityMap
 */
inline float EuclideanDistance(const CityCoordinateMap &city_coordinate_map, 
                               const CityID &c_id1, 
                               const CityID &c_id2) {
  CityCoordinateMap::const_iterator it1 = city_coordinate_map.find(c_id1);
  assert(it1 != city_coordinate_map.end());
  CityCoordinateMap::const_iterator it2 = city_coordinate_map.find(c_id2);
  assert(it2 != city_coordinate_map.end());
  return sqrt(pow(it1->second.x - it2->second.x, 2) +
      pow(it1->second.y - it2->second.y, 2));
}

/*
 * Loads data located in the file <data_file> into a CitySet
 * and a distance map (since we don't need to coordinates - only distances)
 * Returns true if the load succeeded 
 */
inline const bool LoadData(const std::string &data_file, 
                           CitySet &city_set, 
                           CityDistanceMap &city_distance_map) {
  int city_count;
  CityID city_id;
  Coordinate city_coordinate;

  CityCoordinateMap coordinate_map;

  std::ifstream ifs(data_file);
  if (!ifs.is_open()) {
    return false;
  }

  ifs >> city_count;
  while (city_count > 0) {
    ifs >> city_id >> city_coordinate.x >> city_coordinate.y;

    city_set.insert(city_id);
    coordinate_map.insert(std::make_pair(city_id, city_coordinate));

    --city_count;
  }

  for (CitySet::const_iterator it1 = city_set.begin(); it1 != city_set.end(); ++it1) {
    for (CitySet::const_iterator it2 = city_set.begin(); it2 != city_set.end(); ++it2) {
      float distance = EuclideanDistance(coordinate_map, *it1, *it2);
      city_distance_map.insert(std::make_pair(std::make_pair(*it1, *it2), distance));
    }
  }

  return true;
}

/*
 * Utility function to swap two CityIDs in a CityList
 */
inline void Swap(CityList &str_list, int i, int j) {
  std::string temp = str_list[i];
  str_list[i] = str_list[j];
  str_list[j] = temp;
}

/*
 * Returns the total cost of a given CityList given a corresponding CityMap
 */
inline float TotalCost(const CityDistanceMap &city_distance_map, const CityList &path) {
  float sum = 0;
  for (unsigned int i = 0; i < path.size() - 1; ++i) {
    CityDistanceMap::const_iterator it = 
        city_distance_map.find(std::make_pair(path[i], path[i + 1]));
    assert(it != city_distance_map.end());
    sum += it->second;
  }
  return sum;
}


#endif
