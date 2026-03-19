#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <iomanip>

using namespace std;

struct Point { float x, y; };
struct BFSCell { int x, y; float dist; };

class DynamicCostmap {
private:
    vector<float> static_costmap; 
    vector<float> active_costmap; // This is the map A* and DWA will actually use
    int width, height;
    float robot_radius, inflation_radius;

public:
    DynamicCostmap(const vector<float>& base_map, int w, int h, float r_rad, float i_rad) {
        static_costmap = base_map;
        active_costmap = base_map;
        width = w;
        height = h;
        robot_radius = r_rad;
        inflation_radius = i_rad;
    }

    // =======================================================================
    // YOUR ASSIGNMENT: Build the Dynamic Ingestion Engine
    // =======================================================================
    void update_live_sensors(const vector<Point>& lidar_hits) {
        // 1. Reset the active map to the static map (clearing out old moving obstacles)
        active_costmap = static_costmap;
        
        // We need a separate map just to track the distances for this specific dynamic update
        vector<float> dynamic_dist_map(width * height, 1e9f);
        queue<BFSCell> q;

        // 2. Drop the new Lidar hits onto the map
        for (const auto& hit : lidar_hits) {
            // Convert physical coordinates to grid indices
            int grid_x = round(hit.x);
            int grid_y = round(hit.y);
            
            // TODO: If the grid_x and grid_y are within the map bounds:
            // a. Set dynamic_dist_map at this location to 0.0
            // b. Push this cell into the queue
            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height){
                int idx = grid_y * width + grid_x; // ROW-MAJOR FIX
                dynamic_dist_map[idx] = 0.0f;
                active_costmap[idx] = 100.0f; // Direct Lidar hit is instantly lethal
                q.push({grid_x, grid_y, 0.0});
            }
        }

        int dx[] = {-1, 1,  0, 0, -1, -1,  1, 1};
        int dy[] = { 0, 0, -1, 1, -1,  1, -1, 1};
        float step[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

        // 3. Localized Brushfire
        // TODO: Write the BFS loop! (Use the same logic from your last costmap)
        // BUT INSTEAD OF OVERWRITING EVERYTHING:
        // When you calculate a new distance for a neighbor, convert that distance to a penalty cost.
        // Rule: Only update active_costmap[neighbor] if this new penalty cost is GREATER than active_costmap[neighbor].
        // (This ensures a temporary moving human doesn't overwrite a solid concrete wall).
        float decay_factor = 2.0f;
        while(!q.empty()){
            BFSCell current = q.front();
            q.pop();

            for (int i=0; i < 8; i++ ){
                int new_x = current.x + dx[i];
                int new_y = current.y + dy[i];
                float new_dist = current.dist + step[i];

                if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height){
                    int idx = new_y * width + new_x; // ROW-MAJOR FIX
                    if (new_dist < dynamic_dist_map[idx] && new_dist <= inflation_radius){
                        dynamic_dist_map[idx] = new_dist;
                        q.push({new_x, new_y, new_dist});
                    }

                    float penalty = 0.0;
                    if (new_dist <= robot_radius){
                        penalty = 100.0;
                    }
                    else if(new_dist <= inflation_radius){
                        penalty = 100.0 * exp(-decay_factor * (new_dist - robot_radius));
                    }

                    if (penalty > active_costmap[idx]){
                        active_costmap[idx] = penalty;
                        // q.push({new_x, new_y, new_dist});
                    }
                }
            }

        }

    }

    // Returns the fully fused map for the planner to use
    const vector<float>& get_active_map() const {
        return active_costmap;
    }
};

void print_map(const vector<float>& map_data, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float val = map_data[y * width + x];
            if (val >= 100.0) cout << " ██ ";      
            else if (val == 0.0) cout << " .. ";   
            else cout << setw(3) << setfill('0') << (int)val << " "; 
        }
        cout << "\n";
    }
    cout << "\n";
}

int main() {
    int width = 12;
    int height = 8;
    
    // We start with a base map that has a single permanent wall on the right
    vector<float> base_costmap(width * height, 0.0f);
    for(int y=2; y<=5; y++) base_costmap[y*width + 9] = 100.0f; // Solid wall
    
    // We inflate the static map manually just for setup purposes
    DynamicCostmap live_map(base_costmap, width, height, 1.0, 3.5);
    // Trick to inflate the static wall: pass it as a lidar hit, then steal the active map
    vector<Point> static_walls = {{9,2}, {9,3}, {9,4}, {9,5}};
    live_map.update_live_sensors(static_walls);
    base_costmap = live_map.get_active_map(); 

    // Now, we create our REAL dynamic engine using the pre-inflated static map
    DynamicCostmap engine(base_costmap, width, height, 1.0, 3.5);

    cout << "--- FRAME 1: A Human appears at (2, 4) ---" << endl;
    vector<Point> scan_1 = {{2.0, 4.0}}; 
    engine.update_live_sensors(scan_1);
    print_map(engine.get_active_map(), width, height);

    cout << "--- FRAME 2: The Human moves to (5, 4) ---" << endl;
    vector<Point> scan_2 = {{5.0, 4.0}}; 
    engine.update_live_sensors(scan_2);
    print_map(engine.get_active_map(), width, height);

    return 0;
}