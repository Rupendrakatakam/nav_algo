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

    }

    // Returns the fully fused map for the planner to use
    const vector<float>& get_active_map() const {
        return active_costmap;
    }
};