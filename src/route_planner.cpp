#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // Find closest nodes to input coordinates
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
  
}

// Function calculates the H Value between nodes.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return node->distance(*end_node); //determine distance between node and end node

}

/* Function finds the neighbors of the current node, sets the current node parent, g value and h value
then populates the open list with the neighbor nodes found and marks them as visited.
*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  current_node->FindNeighbors(); //find neighbors of current node
  
  for (auto neighbor : current_node->neighbors){
      
      neighbor->parent = current_node;
      neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
      neighbor->h_value = CalculateHValue(neighbor);
      open_list.emplace_back(neighbor);
      neighbor->visited = true;
    
  }

}

// This function is used to compare the f values of adjacent nodes in the open list.
bool Compare(RouteModel::Node *a, RouteModel::Node *b) {
  
  return ((*a).g_value + (*a).h_value) > ((*b).g_value + (*b).h_value); 
  
}

/* This function sorts the open list by f value in descending order, creates a pointer that points to the end element
of the open list, pops the end element off the open list and returns the next node pointer */
RouteModel::Node* RoutePlanner::NextNode() {
  
  sort(open_list.begin(), open_list.end(), Compare);
  RouteModel::Node* next_node = (open_list.back());
  open_list.pop_back();
  return next_node;

}

// Construct the final path linking the start and end nodes.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	// Since node parents are initialized to nullptr, this loop will run until current_node = start_node, which will have a null parent 
  	while ((current_node->parent != nullptr)){
      
      path_found.emplace_back(*current_node);
      distance += current_node->distance(*(current_node->parent));
      current_node = current_node->parent;

    }
  
  path_found.emplace_back(*current_node); //place the start node at the beginning of the final path
  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  std::reverse(path_found.begin(), path_found.end()); //reverse the path_found to get the nodes in correct order
  return path_found;

}

// Function to perform the A* Search
void RoutePlanner::AStarSearch() {
  
  	start_node->visited = true; // start node is automatically visited at the beginning of the search
    RouteModel::Node *current_node = start_node; // initialize current node to the start_node
  	open_list.emplace_back(current_node); // initialize the open list
  	AddNeighbors(current_node); //populate neighbors for the current node
  	//loop until the open list is empty
  	while(!(open_list.empty())){
       current_node = NextNode(); //get the next node
       // If  the distance from the current node to the end node is 0, then we've reached the end.
       if (current_node->distance(*(end_node)) == 0){
         m_Model.path = RoutePlanner::ConstructFinalPath(end_node); // Construct the path for visualization
         return;
       }
      AddNeighbors(current_node); 
    }
}