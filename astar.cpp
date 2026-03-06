#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

using namespace std;

int map_width = 5;
int map_height = 5;
// 0 is free space, 1 is a wall. 
// We are building a wall right down the middle (x=2).
vector<int> grid = {
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, // A gap in the wall!
    0, 0, 1, 0, 0
};

// The Blueprint (Define this outside main)
struct Node {
    int x;
    int y;
    
    // TODO: Add your cost variables here. Use the correct data type for decimal math!
    float f;
    float g;
    float h;
    // TODO: Add variables to store the parent's x and y coordinates.
    // Initialize them to -1 so we know when a node doesn't have a parent yet.
    int parent_x = -1;
    int parent_y = -1;
};


int main() {
    // Now we can use the blueprint to create a Node
    vector<Node> open_list;
    vector<Node> closed_list;

    Node current_node; // The node with the lowest f cost

    float neighbor_x = 0;
    float neighbor_y = 0;

    int goal_x = 4;
    int goal_y = 4;
    
    Node my_start_node;
    my_start_node.x = 0;
    my_start_node.y = 0;
    my_start_node.g = 0;
    my_start_node.h = 0;
    my_start_node.f = my_start_node.g + my_start_node.h;


    open_list.push_back(my_start_node);

    while (open_list.size() > 0){
        float min_f = numeric_limits<float>::max(); // Initialize min_f to the maximum possible float value
        int x = -1;
        for (int i = 0; i < open_list.size(); i++){

            if (open_list[i].f < min_f){
                min_f = open_list[i].f;
                x = i;  
                    
            }
        }
        

        current_node = open_list[x];
        closed_list.push_back(current_node);
        open_list.erase(open_list.begin() + x);
        // cout << "Cost of the node with the lowest f is " << min_f << endl;
        cout << "coordinates of the node with the lowest f is " << current_node.x << ", " << current_node.y << endl;


        // --- NEW: THE GOAL CHECK ---
        if (current_node.x == goal_x && current_node.y == goal_y) {
            cout << "GOAL REACHED at " << current_node.x << ", " << current_node.y << "!" << endl;
            vector<Node> final_path;
            Node trace_node = current_node;

            // 1. Loop backwards until we find the Start Node (parent_x == -1)
            while (trace_node.parent_x != -1) {
                // Add the current step to our path
                final_path.push_back(trace_node);
                
                // TODO: Find the parent!
                // Write a for-loop that searches through the 'closed_list'.
                // If you find a node in the closed_list whose 'x' and 'y' match 
                // the trace_node.parent_x and trace_node.parent_y, do TWO things:
                // 1. trace_node = closed_list[that_index];
                // 2. break; (to stop searching the closed_list and continue the while loop)
                for (int i = 0; i < closed_list.size(); i++ ){
                    if (trace_node.parent_x == closed_list[i].x && trace_node.parent_y == closed_list[i].y ){
                        trace_node = closed_list[i];
                        break;
                    }
                }
            }
            // Add the start node itself to complete the path
            final_path.push_back(trace_node); 

            // 2. Print the final path instructions!
            cout << "\n--- FINAL DRIVING PATH ---" << endl;
            // Since we traced backwards from Goal to Start, we must print the vector in reverse.
            for (int p = final_path.size() - 1; p >= 0; p--) {
                cout << "Drive to: [" << final_path[p].x << ", " << final_path[p].y << "]" << endl;
            }

            break; // Shatter the master while loop. The algorithm is finished.
        }
        

        float step_cost = 0;
        // Loop through the Y offsets (-1, 0, 1)
        for (int dy = -1; dy <= 1; dy++) {
            // Loop through the X offsets (-1, 0, 1)
            for (int dx = -1; dx <= 1; dx++) {
                
                // 1. Calculate the neighbor's actual coordinates
                float neighbor_x = current_node.x + dx;
                float neighbor_y = current_node.y + dy;

                // 2. Skip the node itself (where dx is 0 AND dy is 0)
                if (dx == 0 && dy == 0) {
                    continue; // This skips to the next iteration of the loop
                }

                if (neighbor_x >= 0 && neighbor_x < map_width && neighbor_y >= 0 && neighbor_y < map_height){
                    if (grid[neighbor_y * map_width + neighbor_x] == 1){
                        // cout << "Wall found at: " << neighbor_x << ", " << neighbor_y << endl;
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

                    bool in_closed_list = false;
                    for (int c = 0; c < closed_list.size(); c++) {
                        if (closed_list[c].x == neighbor_x && closed_list[c].y == neighbor_y) {
                            in_closed_list = true;
                            break;
                        }
                    }

                    if (in_closed_list == true) {
                        continue; // Skip this neighbor. We already fully evaluated it!
                    }
                    
                    float dist_x = abs(neighbor_x - goal_x);
                    float dist_y = abs(neighbor_y - goal_y);

                    float g_cost = current_node.g + step_cost;
                    float h_cost = 1.0 * (dist_x + dist_y) + (1.414 - 2.0 * 1.0) * std::min(dist_x, dist_y);
                    float f_cost = g_cost + h_cost;

                    // Check if this neighbor is already in the open list
                    bool in_open_list = false;
                    for (int o = 0; o < open_list.size(); o++) {
                        if (open_list[o].x == neighbor_x && open_list[o].y == neighbor_y) {
                            in_open_list = true;
                            if (g_cost < open_list[o].g) {
                                // Found a better path — update the node in place
                                open_list[o].g = g_cost;
                                open_list[o].h = h_cost;
                                open_list[o].f = f_cost;
                                open_list[o].parent_x = current_node.x;
                                open_list[o].parent_y = current_node.y;
                            }
                            break;
                        }
                    }

                    if (in_open_list == false) {
                        Node new_neighbor_node;
                        new_neighbor_node.x = neighbor_x;
                        new_neighbor_node.y = neighbor_y;
                        new_neighbor_node.g = g_cost;
                        new_neighbor_node.h = h_cost;
                        new_neighbor_node.f = f_cost;
                        new_neighbor_node.parent_x = current_node.x;
                        new_neighbor_node.parent_y = current_node.y;
                        open_list.push_back(new_neighbor_node);
                    }

                // cout << "Added neighbor at " << neighbor_x << ", " << neighbor_y << " with f_cost: " << f_cost << endl;
                }
            }
        }
    }
    return 0;
}