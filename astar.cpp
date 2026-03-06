#include <iostream>
#include <vector>
#include <limits>

using namespace std;

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

    float min = numeric_limits<float>::max(); // Initialize min to the maximum possible float value
    int x = -1;
    int y = -1;
    for (int i = 0; i < open_list.size(); i++){

        if (open_list[i].f < min){
            min = open_list[i].f;
            x = open_list[i].x;
            y = open_list[i].y;
            
        }
    }
    cout << "Cost of the node with the lowest f is " << min << endl;
    cout << "coordinates of the node with the lowest f is " << x << ", " << y << endl;

    return 0;
}