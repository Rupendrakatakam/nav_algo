#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <fstream>

using namespace std;

// ==============================================================================
// 1. DATA STRUCTURES
// ==============================================================================
struct Node { int x, y; float f, g, h; int parent_x = -1, parent_y = -1; };
struct CompareNode { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
struct RobotState { float x, y, theta, v, w; };
struct Window { float v_min, v_max, w_min, w_max; };
struct Command { float v, w; };
struct Point { float x, y; };

// ==============================================================================
// 2. THE GLOBAL PLANNER (A*)
// ==============================================================================
class AStarPlanner {
public:
    vector<Node> find_path(int start_x, int start_y, int goal_x, int goal_y, int map_width, int map_height, const vector<int>& grid) {
        vector<Node> final_path;
        priority_queue<Node, vector<Node>, CompareNode> open_list;
        unordered_map<int, Node> closed_map;
        
        Node start_node = {start_x, start_y, 0, 0, 0, -1, -1};
        open_list.push(start_node);

        while (!open_list.empty()){
            Node current_node = open_list.top();
            open_list.pop();

            int current_id = (current_node.y * map_width) + current_node.x;
            if (closed_map.find(current_id) != closed_map.end()) continue;
            closed_map[current_id] = current_node;

            if (current_node.x == goal_x && current_node.y == goal_y) {
                Node trace_node = current_node;
                while (trace_node.parent_x != -1) {
                    final_path.push_back(trace_node);
                    int parent_id = (trace_node.parent_y * map_width) + trace_node.parent_x;
                    trace_node = closed_map[parent_id];
                }
                final_path.push_back(trace_node); 
                reverse(final_path.begin(), final_path.end());
                break; 
            }
            
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = current_node.x + dx, ny = current_node.y + dy;

                    if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height){
                        if (grid[ny * map_width + nx] == 1) continue;

                        int neighbor_id = (ny * map_width) + nx;
                        if (closed_map.find(neighbor_id) != closed_map.end()) continue;
                        
                        float step_cost = (dx == 0 || dy == 0) ? 1.0 : 1.414;
                        float g_cost = current_node.g + step_cost;
                        float dist_x = abs(nx - goal_x), dist_y = abs(ny - goal_y);
                        float h_cost = 1.0 * (dist_x + dist_y) + (1.414 - 2.0) * min(dist_x, dist_y);
                        
                        open_list.push({nx, ny, g_cost + h_cost, g_cost, h_cost, current_node.x, current_node.y});
                    }
                }
            }
        }
        return final_path; 
    }
};

// ==============================================================================
// 3. THE LOCAL CONTROLLER (DWA)
// ==============================================================================
vector<RobotState> predict_trajectory(RobotState start, float v, float w, float sim_time, float dt) {
    vector<RobotState> traj;
    RobotState curr = start;
    traj.push_back(curr);
    for(float t = 0; t < sim_time; t += dt){
        curr.v = v; curr.w = w;
        curr.x += curr.v * cos(curr.theta) * dt;
        curr.y += curr.v * sin(curr.theta) * dt;
        curr.theta += curr.w * dt;
        traj.push_back(curr);
    }
    return traj;
}

Window generate_dynamic_window(RobotState r, float max_v, float min_v, float max_w, float min_w, float max_a, float max_d, float dt) {
    return {
        max(min_v, r.v + max_d * dt), min(max_v, r.v + max_a * dt),
        max(min_w, r.w + max_d * dt), min(max_w, r.w + max_a * dt)
    };
}

vector<vector<RobotState>> generate_all_trajectories(RobotState r, Window win, float sim_time, float dt, float v_res, float w_res) {
    vector<vector<RobotState>> all;
    for(float v = win.v_min; v <= win.v_max + 0.01; v += v_res)
        for(float w = win.w_min; w <= win.w_max + 0.01; w += w_res)
            all.push_back(predict_trajectory(r, v, w, sim_time, dt));    
    return all;
}

Command calculate_best_command(vector<vector<RobotState>>& rollouts, float g_x, float g_y, vector<Point>& obs, float radius) {
    float best_cost = 1e9;
    Command best_cmd = {0.0, 0.0};

    // Tuned weights for smooth tracking
    float alpha = 1.0;  // Goal
    float beta = 0.2;   // Clearance
    float gamma = 3.0;  // Speed
    float delta = 1.0;  // Heading

    for(auto& traj : rollouts){
        auto end_state = traj.back();
        float dist_to_goal = hypot(g_x - end_state.x, g_y - end_state.y);
        
        float min_obs_dist = 1e9;
        bool crashed = false;
        for (auto& state : traj){ 
            for (auto& o : obs){
                float dist = hypot(o.x - state.x, o.y - state.y);
                if (dist <= radius) { crashed = true; break; }
                if (dist < min_obs_dist) min_obs_dist = dist;
            }
            if (crashed) break;
        }
        if(crashed) continue;     
        
        float clearance_cost = 1.0 / min_obs_dist; 
        
        float angle_to_goal = atan2(g_y - end_state.y, g_x - end_state.x);
        float heading_err = angle_to_goal - end_state.theta;
        while (heading_err > M_PI)  heading_err -= 2.0 * M_PI;
        while (heading_err < -M_PI) heading_err += 2.0 * M_PI;
        
        float cost = (alpha * dist_to_goal) + (beta * clearance_cost) + (delta * abs(heading_err)) - (gamma * end_state.v);
        
        if(cost < best_cost){
            best_cost = cost;
            best_cmd = {traj.back().v, traj.back().w};
        }
    }
    return best_cmd;
}

// ==============================================================================
// 4. COST MAP
// ==============================================================================
class CostMap {
public:
    vector<int> costmap_data;

}



// ==============================================================================
// 5. THE MASTER NODE (Bringing it all together)
// ==============================================================================
int main() {
    // A 10x10 Map. 0 = Free, 1 = Wall.
    int map_w = 10, map_h = 10;
    vector<int> grid = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 0, 0, 0, 0, 1, 1, 0,
        0, 1, 1, 0, 0, 0, 0, 1, 1, 0,
        0, 1, 1, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    // 1. RUN GLOBAL PLANNER (A*)
    AStarPlanner astar;
    int start_x = 0, start_y = 0;
    int goal_x = 9, goal_y = 9;
    vector<Node> global_path = astar.find_path(start_x, start_y, goal_x, goal_y, map_w, map_h, grid);

    if(global_path.empty()) {
        cout << "A* Failed! No path to goal." << endl;
        return -1;
    }

    // Convert grid walls into DWA obstacles
    vector<Point> obstacles;
    ofstream map_log("map.csv");
    map_log << "x,y\n";
    for(int y=0; y<map_h; y++) {
        for(int x=0; x<map_w; x++) {
            if(grid[y*map_w + x] == 1) {
                obstacles.push_back({(float)x, (float)y});
                map_log << x << "," << y << "\n";
            }
        }
    }
    map_log.close();

    // Log the A* Path
    ofstream path_log("global_path.csv");
    path_log << "x,y\n";
    for(auto n : global_path) path_log << n.x << "," << n.y << "\n";
    path_log.close();

    // 2. RUN LOCAL PLANNER (DWA)
    RobotState robot = {(float)start_x, (float)start_y, 0.0, 0.0, 0.0};
    float robot_radius = 0.8;
    float dt = 0.1, sim_time = 2.0;

    ofstream robot_log("robot_path.csv");
    robot_log << "x,y,theta\n";

    cout << "STARTING TRACKING EXECUTION..." << endl;
    
    int path_index = 0; // The waypoint we are currently chasing
    
    for(int iteration = 0; iteration < 1000; iteration++) {
        // CARROT CHASING LOGIC:
        // Find a target waypoint roughly 1.5 meters ahead of the robot.
        float lookahead_dist = 1.5;
        while (path_index < global_path.size() - 1) {
            float dx = global_path[path_index].x - robot.x;
            float dy = global_path[path_index].y - robot.y;
            if (hypot(dx, dy) >= lookahead_dist) break;
            path_index++;
        }
        
        float target_x = global_path[path_index].x;
        float target_y = global_path[path_index].y;

        // Run DWA aimed at the Carrot
        Window win = generate_dynamic_window(robot, 1.0, 0.0, 1.0, -1.0, 2.0, -2.0, dt);
        auto rollouts = generate_all_trajectories(robot, win, sim_time, dt, 0.1, 0.1);
        Command cmd = calculate_best_command(rollouts, target_x, target_y, obstacles, robot_radius);

        // Execute physics
        auto exec = predict_trajectory(robot, cmd.v, cmd.w, dt, dt);
        robot = exec.back();
        robot_log << robot.x << "," << robot.y << "," << robot.theta << "\n";

        // Have we reached the absolute final goal?
        if (hypot(goal_x - robot.x, goal_y - robot.y) < 0.3) {
            cout << "ARRIVED AT FINAL GOAL!" << endl;
            break;
        }
    }

    robot_log.close();
    cout << "Simulation data dumped. Ready for Python Visualization." << endl;
    return 0;
}