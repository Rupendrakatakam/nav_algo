#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <iomanip>
using namespace std;

struct cell{
    int x,y;
    float dist;
};

// =======================================================================
// YOUR ASSIGNMENT: Build the Brushfire Costmap Generator
// =======================================================================
vector<float> generate_costmap(const vector<int>& binary_grid, int width, int height, float robot_radius, float inflation_radius) {
    // 1. Initialize maps
    vector<float> costmap(width * height, 0.0);
    vector<float> dist_map(width * height, 1e9); // Start with infinity distance everywhere
    queue<cell> q;

    // 2. Ignite the Fire (Setup the Queue)
    // TODO: Loop through the entire binary_grid.
    // If a cell is a wall (1), set its dist_map value to 0.0 and push it into the queue 'q'.

    for (int i = 0; i < height ; i++){
        for (int j = 0; j < width ; j++){
            if (binary_grid[i * width + j] == 1){
                dist_map[i * width + j] = 0.0;
                q.push({i, j, 0.0});
            }
        }
    }

    // 3. Let it Spread (Multi-Source BFS)
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    float cost_step[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414}; // Straight vs Diagonal distances

    // TODO: While the queue is not empty:
    //   a. Pop the front cell.
    //   b. Loop through its 8 neighbors.
    //   c. Calculate new_dist = current cell's distance + cost_step.
    //   d. If the neighbor is within bounds, AND new_dist < inflation_radius, AND new_dist < dist_map[neighbor]:
    //      - Update dist_map[neighbor] = new_dist.
    //      - Push the neighbor into the queue.
    while (!q.empty()){
        cell front_cell = q.front();
        q.pop();
        
        for(int i = 0 ; i < 8 ; i++){
            int new_x = front_cell.x + dx[i];
            int new_y = front_cell.y + dy[i];
            float new_dist = front_cell.dist + cost_step[i];

            if(new_x >= 0 && new_x < width && new_y >= 0 && new_y < height && new_dist < inflation_radius && new_dist < dist_map[new_x * width + new_y]){
                dist_map[new_x * width + new_y] = new_dist;
                q.push({new_x, new_y, new_dist});
            }  
        }
    }

    // 4. Calculate the Final Costs
    float decay_factor = 2.0; // Controls how fast the danger drops off
    // TODO: Loop through the dist_map.
    // If dist <= robot_radius: set costmap to 100.0
    // Else if dist <= inflation_radius: set costmap to 100.0 * exp(-decay_factor * (dist - robot_radius))
    // Else: costmap is 0.0

    for(int i = 0 ; i < width ; i++){
        for(int j = 0 ; j < height ; j++){
            if(dist_map[i * width + j] <= robot_radius){
                costmap[i * width + j] = 100.0;
            }
            else if(dist_map[i * width + j] <= inflation_radius){
                costmap[i * width + j] = 100.0 * exp(-decay_factor * (dist_map[i * width + j] - robot_radius));
            }
            else{
                costmap[i * width + j] = 0.0;
            }
        }
    }


    return costmap;
}
// A helper function to print the map to the terminal nicely
void print_map(const vector<float>& map_data, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float val = map_data[y * width + x];
            if (val >= 100.0) cout << " ██ ";      // Lethal wall
            else if (val == 0.0) cout << " .. ";   // Free space
            else cout << setw(3) << setfill('0') << (int)val << " "; // Gradient zone
        }
        cout << "\n";
    }
}

int main() {
    int width = 10;
    int height = 10;
    
    // 10x10 grid. A 2x2 solid wall block is sitting in the middle.
    vector<int> binary_grid = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    float robot_radius = 1.0;     // Robot takes up roughly 1 cell
    float inflation_radius = 3.5; // Spread danger outwards for 3.5 cells

    vector<float> costmap = generate_costmap(binary_grid, width, height, robot_radius, inflation_radius);

    cout << "INFLATED COSTMAP:" << endl;
    print_map(costmap, width, height);

    return 0;
}