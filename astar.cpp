#include <iostream>


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
    
    // TODO: Assign values to the g, h, and f costs of my_start_node.
    my_start_node.g = 0;
    my_start_node.h = 5;
    my_start_node.f = my_start_node.g + my_start_node.h;



    cout << "Node created at (" << my_start_node.x << ", " << my_start_node.y << ")" << endl;
    cout << "F cost :" << my_start_node.f << endl;
    
    
    return 0;
}