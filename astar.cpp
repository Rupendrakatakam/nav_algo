#include <iostream>
#include <vector>
#include <limits>

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
    Node my_start_node;
    my_start_node.x = 0;
    my_start_node.y = 0;

    Node node_a;
    node_a.x = 1;
    node_a.y = 1;
    node_a.f = 3.2;

    Node node_b;
    node_b.x = 2;
    node_b.y = 2;
    node_b.f = 8.2;

    // TODO: Assign values to the g, h, and f costs of my_start_node.
    my_start_node.g = 0;
    my_start_node.h = 5;
    my_start_node.f = my_start_node.g + my_start_node.h;


    vector<Node> open_list;
    open_list.push_back(my_start_node);
    open_list.push_back(node_a);
    open_list.push_back(node_b);

    vector<Node> closed_list;

    float min = numeric_limits<float>::max(); // Initialize min to the maximum possible float value
    int x = -1;
    for (int i = 0; i < open_list.size(); i++){

        if (open_list[i].f < min){
            min = open_list[i].f;
            x = i;  
                 
        }
    }
    
    Node current_node = open_list[x];
    closed_list.push_back(current_node);
    open_list.erase(open_list.begin() + x);
    cout << "Cost of the node with the lowest f is " << min << endl;
    cout << "coordinates of the node with the lowest f is " << current_node.x << ", " << current_node.y << endl;

     
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

            // // 3. The Guardrail: Check if the neighbor is inside the map boundaries!
            // if ( neighbor_x >= 0 && neighbor_x < map_width && neighbor_y >= 0 && neighbor_y < map_height ) {
            //     cout << "Valid neighbor found at: " << neighbor_x << ", " << neighbor_y << endl;
            // }

            if (grid[neighbor_y * map_width + neighbor_x] == 1){
                cout << "Wall found at: " << neighbor_x << ", " << neighbor_y << endl;
                continue;
            }
            else {
                cout << "Free space found at: " << neighbor_x << ", " << neighbor_y << endl;
            }
        }
    }

    return 0;
}