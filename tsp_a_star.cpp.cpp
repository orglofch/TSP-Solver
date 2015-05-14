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

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>
#include <list>
#include <set>

#include "tsp.hpp"

using namespace std;

/*
 * Prints usage
 */
void Usage() {
  cout << "Usage: tsp_a_star [city-data] <start-city-id>" << endl;
}

/*
 * Returns the heuristic cost of a given partial travel configuration
 * Uses the Minimum spanning tree heuristic
 *
 * Uses Kruskals algorithm to find the minimum spanning tree
 */
float HeuristicCost(const CitySet &city_set, 
                    const CityDistanceMap &city_distance_map,
                    const CityPath &path, 
                    const CityID &start_id) {
  // Create a set to accelerate finding cities not yet in set
  CitySet current_cities(path.cities.begin(), path.cities.end());

  // Create set of non-visited cities
  CityList new_cities;
  for (CitySet::const_iterator it = city_set.begin(); it != city_set.end(); ++it) {
    if (current_cities.find(*it) == current_cities.end()) {
      new_cities.push_back(*it);
    }
  }
  // Include the start and end of the current path
  new_cities.push_back(start_id);
  new_cities.push_back(path.cities[path.cities.size() - 1]);

  CityEdgeList new_edges(new_cities.size() * (new_cities.size() - 1));
  int edge_count = 0;
  // Create set of all edges not already on path / don't create cycles
  for (unsigned int i = 0; i < new_cities.size(); ++i) {
    for (unsigned int j = i + 1; j < new_cities.size(); ++j) {
      if (new_cities[i] != new_cities[j]) {
        CityEdge &city_edge = new_edges[edge_count];
        city_edge.city1 = new_cities[i];
        city_edge.city2 = new_cities[j];
  
        CityDistanceMap::const_iterator distance_it = 
            city_distance_map.find(make_pair(new_cities[i], new_cities[j]));
        assert(distance_it != city_distance_map.end());
        city_edge.cost = distance_it->second;
        ++edge_count;
      }
    }
  }
  
  CityEdgeList mst;
  unordered_map<int, CityList> forests; // mapping of forest IDs to cities
  unordered_map<CityID, int> forest_map; // mapping of cities to forest IDs
 
  // Initialize each city as its own forest
  int next_forest_id = 0;
  for (CityList::const_iterator it = new_cities.begin(); it != new_cities.end(); ++it) {
    CityList forest;
    forest.push_back(*it);
    forests.insert(make_pair(next_forest_id, forest));
    forest_map.insert(make_pair(*it, next_forest_id++));
  }

  // Sort edges by cost
  sort(new_edges.begin(), new_edges.end());

  for (CityEdgeList::const_iterator it = new_edges.begin(); it != new_edges.end(); ++it) {
    // If they are in disjoint sets merge them and add them to the MST
    if (forest_map[it->city1] != forest_map[it->city2]) {
      // Update all members of city2s disjoint set to have same forest id as city1
      CityList &city1_forest = forests[forest_map[it->city1]];
      CityList &city2_forest = forests[forest_map[it->city2]];
      for (CityList::iterator tree = city2_forest.begin(); tree != city2_forest.end(); ++tree) {
        forest_map[*tree] = forest_map[it->city1];
        city1_forest.push_back(*tree);
      }
      mst.push_back(*it);
    }
  }

  // Return the cost of the minimum spanning tree
  int mst_cost = 0;
  for (CityEdgeList::const_iterator it = mst.begin(); it != mst.end(); ++it) {
    mst_cost += it->cost;
  }
  return mst_cost;
}

/*
 * Returns the set all neighbour CityPath configurations for a
 * given CityList into the passed CityPathSet
 */
void Neighbours(const CitySet &city_set, 
                const CityDistanceMap &city_distance_map,
                const CityPath &current_path, 
                const CityID &start_id,
                CityPathSet &neighbours) {
  // Create a set to accelerate finding duplicates
  CitySet current_cities(current_path.cities.begin(), current_path.cities.end());

  const CityID &last_city_id = current_path.cities[current_path.cities.size() - 1];

  // NOTE: We don't need to generate the successors alphabetically because the 
  // 'open' set uses alphabetical ordering for nodes with the same heuristic_cost
  // I assume this is sufficient given Piazza discussion

  for (CitySet::const_iterator it = city_set.begin(); it != city_set.end(); ++it) {
    // Only travel to cities not already on the path, and only travel to start city
    // if we've exhausted all other cities (we know that tour won't work otherwise)
    if (current_cities.find(*it) == current_cities.end() ||
        (*it == start_id && current_cities.size() == city_set.size())) {
      CityPath new_path(current_path.cities, current_path.cost);
      new_path.cities.push_back(*it);

      CityDistanceMap::const_iterator distance_it = 
          city_distance_map.find(make_pair(last_city_id, *it));
      assert(distance_it != city_distance_map.end());
      new_path.cost += distance_it->second;
      new_path.heuristic_cost = new_path.cost + 
          HeuristicCost(city_set, city_distance_map, new_path, start_id);
   
      neighbours.insert(new_path);
    }
  }
}

/*
 * TSP solver, via A* Search
 *
 * Input:  [city_set] set of all cities
 *         [city_distance_map] mapping of pairs of cities
 *                             to their distances
 *         [start_id] city to start at
 *         [print_nodes] whether the program should print
 *                       the number of nodes generated
 * Return: Nothing, Prints the shortest tour to stdout
 */
void Search(const CitySet &city_set,
            const CityDistanceMap &city_distance_map,
            const CityID &start_id,
            bool print_nodes) {
  // Immediately return if only have one city
  // Otherwise we'll end up with A -> A
  if (city_set.size() == 1) {
    CityPath tour(CityList(), 0);
    tour.cities.push_back(start_id);
    if (print_nodes) {
      cout << 1 << endl;
    }
    cout << tour << endl; 
    return;
  }
  
  CityPathSet visited;
  set<CityPath> open; // functions as priority queue

  CityPath start_path(CityList(), 0);
  start_path.cities.push_back(start_id);
  start_path.heuristic_cost = 
      HeuristicCost(city_set, city_distance_map, start_path, start_id);

  // Initialize start state
  open.insert(start_path);

  // The original state counts as a node
  int node_count = 1;

  while (!open.empty()) {
    // Select node in open set with min f_score
    const CityPath current_path = *open.begin();
    const CityID &current_city = current_path.cities[current_path.cities.size() - 1];

    // Mark node as visited and remove from open list
    open.erase(open.begin());
    visited.insert(current_path);

    // The end state is once we've visited all nodes and returned to the start city
    if (current_path.cities.size() == city_set.size() + 1 && current_city == start_id) {
      if (print_nodes) {
        cout << node_count << endl;
      }
      cout << current_path << endl;
      return;
    }
 
    // Generate successor configurations
    CityPathSet neighbours;
    Neighbours(city_set, city_distance_map, current_path, start_id, neighbours);

    for (CityPathSet::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
      // If the neighbour hasn't already been visited
      if (visited.find(*it) == visited.end()) {
       
        // Neighbours should always be new based on how we generate successors
        pair<set<CityPath>::iterator, bool> ret = open.insert(*it);
        assert(ret.second); // Make sure we don't generate duplicate nodes somehow

        // We count nodes as they are generated, not as they are expanded
        ++node_count;
      }
    }
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    Usage();
    return 1;
  }

  // Default arguments
  string data_file = argv[1];
  CityID start_city_id = "A";
  bool print_nodes = false;
   
  // Load
  CitySet city_set;
  CityDistanceMap city_distance_map;
  if (!LoadData(data_file, city_set, city_distance_map)) {
    cout << "Failed to load datafile." << endl;
    return 1;
  }

  assert(city_set.find(start_city_id) != city_set.end());

  // Process flags
  for (int i = 2; i < argc; ++i) {
    if (strcmp(argv[i], "-city") == 0) {
      if (city_set.find(argv[i + 1]) != city_set.end()) {
        start_city_id = argv[i + 1];
        ++i;
      } else {
        cout << "Invalid start city: " << argv[i + 1] << endl;
        return 1;
      }
    } else if (strcmp(argv[i], "-pn") == 0) {
      print_nodes = true;
    }
  }

  assert(city_set.find(start_city_id) != city_set.end());

  // Search
  Search(city_set, city_distance_map, start_city_id, print_nodes);

  return 0;
}
