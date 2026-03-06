#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm> // Needed to reverse the final path vector

using namespace std;

// The Blueprint (Define this outside main)
struct Node {
    int x;
    int y;
    
    float f;
    float g;
    float h;

    int parent_x = -1;
    int parent_y = -1;
};

// Comparator for the priority queue to sort by lowest f cost
struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f; // '>' because priority_queue is a max-heap by default, this makes it a min-heap
    }
};

// THE NEW ARCHITECTURE: The Class
class AStarPlanner {
public:
    // This is the function the ROS node will eventually call!
    vector<Node> find_path(int start_x, int start_y, int goal_x, int goal_y, int map_width, int map_height, const vector<int>& grid) {
        
        vector<Node> final_path;
        priority_queue<Node, vector<Node>, CompareNode> open_list;
        unordered_map<int, Node> closed_map;
        
        Node my_start_node;
        my_start_node.x = start_x;
        my_start_node.y = start_y;
        my_start_node.g = 0;
        my_start_node.h = 0;
        my_start_node.f = my_start_node.g + my_start_node.h;

        open_list.push(my_start_node);

        while (!open_list.empty()){
            Node current_node = open_list.top();
            open_list.pop();

            // 1. Calculate the unique 1D ID for this coordinate
            int current_id = (current_node.y * map_width) + current_node.x;

            // 2. O(1) Lookup: Check if it exists in the Hash Map
            if (closed_map.find(current_id) != closed_map.end()) {
                continue; // We already expanded this! Throw it away.
            }

            // 3. Save it to the Hash Map
            closed_map[current_id] = current_node;

            // --- THE GOAL CHECK ---
            if (current_node.x == goal_x && current_node.y == goal_y) {
                
                Node trace_node = current_node;

                // 1. Loop backwards until we find the Start Node (parent_x == -1)
                while (trace_node.parent_x != -1) {
                    // Add the current step to our path
                    final_path.push_back(trace_node);
                    
                    // 1. Calculate the unique ID of the parent
                    int parent_id = (trace_node.parent_y * map_width) + trace_node.parent_x;
                    
                    // 2. Instantly grab the parent node from the Hash Map!
                    trace_node = closed_map[parent_id];
                }
                // Add the start node itself to complete the path
                final_path.push_back(trace_node); 

                // Reverse the path vector to return it Start -> Goal
                reverse(final_path.begin(), final_path.end());

                break; // Shatter the master while loop. The algorithm is finished.
            }
            
            float step_cost = 0;
            // Loop through the Y offsets (-1, 0, 1)
            for (int dy = -1; dy <= 1; dy++) {
                // Loop through the X offsets (-1, 0, 1)
                for (int dx = -1; dx <= 1; dx++) {
                    
                    // 1. Calculate the neighbor's actual coordinates
                    int neighbor_x = current_node.x + dx;
                    int neighbor_y = current_node.y + dy;

                    // 2. Skip the node itself (where dx is 0 AND dy is 0)
                    if (dx == 0 && dy == 0) {
                        continue; // This skips to the next iteration of the loop
                    }

                    if (neighbor_x >= 0 && neighbor_x < map_width && neighbor_y >= 0 && neighbor_y < map_height){
                        if (grid[neighbor_y * map_width + neighbor_x] == 1){
                            continue;
                        }
                        else {
                            if (dx == 0 || dy == 0){
                                step_cost = 1.0;
                            }
                            else if (dx != 0 && dy != 0){
                                step_cost = 1.414;
                            }
                        }

                        int neighbor_id = (neighbor_y * map_width) + neighbor_x;
                        if (closed_map.find(neighbor_id) != closed_map.end()) {
                            continue; // Skip this neighbor, it's already closed!
                        }
                        
                        float dist_x = abs(neighbor_x - goal_x);
                        float dist_y = abs(neighbor_y - goal_y);

                        float g_cost = current_node.g + step_cost;
                        float h_cost = 1.0 * (dist_x + dist_y) + (1.414 - 2.0 * 1.0) * std::min(dist_x, dist_y);
                        float f_cost = g_cost + h_cost;

                        // Push directly to the priority queue. 
                        Node new_neighbor_node;
                        new_neighbor_node.x = neighbor_x;
                        new_neighbor_node.y = neighbor_y;
                        new_neighbor_node.g = g_cost;
                        new_neighbor_node.h = h_cost;
                        new_neighbor_node.f = f_cost;
                        new_neighbor_node.parent_x = current_node.x;
                        new_neighbor_node.parent_y = current_node.y;
                        open_list.push(new_neighbor_node);
                    }
                }
            }
        }
        
        return final_path; 
    }
};

// The Clean Main Function
int main() {
    // We define our world
    int map_width = 5;
    int map_height = 5;
    vector<int> grid = {
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, // A gap in the wall!
        0, 0, 1, 0, 0
    };

    // We instantiate our new Tool!
    AStarPlanner planner;

    // We ask the tool to do the math
    cout << "Planning path..." << endl;
    vector<Node> path = planner.find_path(0, 0, 4, 4, map_width, map_height, grid);

    // We print the results
    if (path.empty()) {
        cout << "FAILED: No path found!" << endl;
    } else {
        cout << "\n--- FINAL DRIVING PATH ---" << endl;
        for (int p = 0; p < path.size(); p++) {
            cout << "Drive to: [" << path[p].x << ", " << path[p].y << "]" << endl;
        }
    }

    return 0;
}