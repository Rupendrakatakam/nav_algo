#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <unordered_map>
#include <fstream>

using namespace std;

// --- Data Structures ---
struct Node { int x, y; float f, g, h; int parent_x = -1, parent_y = -1; };
struct CompareNode { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
struct Point { float x, y; };
struct BFSCell { int x, y; float dist; };
struct RobotState { float x, y, theta, v, w; };
struct Window { float v_min, v_max, w_min, w_max; };
struct Command { float v, w; };

// --- Dynamic Costmap ---
class DynamicCostmap {
private:
    vector<float> static_costmap, active_costmap;
    int width, height;
    float robot_radius, inflation_radius;
public:
    DynamicCostmap(const vector<float>& base_map, int w, int h, float r_rad, float i_rad) 
        : static_costmap(base_map), active_costmap(base_map), width(w), height(h), robot_radius(r_rad), inflation_radius(i_rad) {}

    void update_live_sensors(const vector<Point>& lidar_hits) {
        active_costmap = static_costmap;
        vector<float> dynamic_dist_map(width * height, 1e9f);
        queue<BFSCell> q;

        for (const auto& hit : lidar_hits) {
            int gx = round(hit.x), gy = round(hit.y);
            if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                int idx = gy * width + gx;
                dynamic_dist_map[idx] = 0.0f;
                active_costmap[idx] = 100.0f;
                q.push({gx, gy, 0.0f});
            }
        }

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
        float step[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

        while (!q.empty()) {
            BFSCell cur = q.front(); q.pop();
            for (int i = 0; i < 8; i++) {
                int nx = cur.x + dx[i], ny = cur.y + dy[i];
                float nd = cur.dist + step[i];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int idx = ny * width + nx;
                    if (nd < dynamic_dist_map[idx] && nd <= inflation_radius) {
                        dynamic_dist_map[idx] = nd;
                        q.push({nx, ny, nd});
                        
                        float new_cost = (nd <= robot_radius) ? 100.0f : 100.0f * exp(-2.0f * (nd - robot_radius));
                        if (new_cost > active_costmap[idx]) active_costmap[idx] = new_cost;
                    }
                }
            }
        }
    }
    const vector<float>& get_active_map() const { return active_costmap; }
};

// --- A* Planner ---
class AStarPlanner {
public:
    vector<float> generate_static_costmap(const vector<int>& grid, int w, int h, float r_rad, float i_rad) {
        vector<float> costmap(w * h, 0.0f);
        vector<float> dist_map(w * h, 1e9f);
        queue<BFSCell> q;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                if (grid[y * w + x] == 1) {
                    dist_map[y * w + x] = 0.0f;
                    q.push({x, y, 0.0f});
                }
            }
        }

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1}, dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
        float step[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41f, 1.41f, 1.41f, 1.41f};

        while (!q.empty()) {
            BFSCell cur = q.front(); q.pop();
            for (int i = 0; i < 8; i++) {
                int nx = cur.x + dx[i], ny = cur.y + dy[i];
                float nd = cur.dist + step[i];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                if (nd >= i_rad || nd >= dist_map[ny * w + nx]) continue;
                dist_map[ny * w + nx] = nd;
                q.push({nx, ny, nd});
            }
        }

        for (int i = 0; i < w * h; i++) {
            if (dist_map[i] <= r_rad) costmap[i] = 100.0f;
            else if (dist_map[i] <= i_rad) costmap[i] = 100.0f * exp(-2.0f * (dist_map[i] - r_rad));
        }
        return costmap;
    }

    vector<Node> find_path(int sx, int sy, int gx, int gy, int w, int h, const vector<float>& costmap) {
        vector<Node> path;
        priority_queue<Node, vector<Node>, CompareNode> open;
        unordered_map<int, Node> closed;
        open.push({sx, sy, 0, 0, 0, -1, -1});

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1}, dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!open.empty()) {
            Node cur = open.top(); open.pop();
            int cid = cur.y * w + cur.x;
            if (closed.count(cid)) continue;
            closed[cid] = cur;

            if (cur.x == gx && cur.y == gy) {
                Node n = cur;
                while (n.parent_x != -1) {
                    path.push_back(n);
                    n = closed[n.parent_y * w + n.parent_x];
                }
                path.push_back(n);
                reverse(path.begin(), path.end());
                return path;
            }

            for (int i = 0; i < 8; i++) {
                int nx = cur.x + dx[i], ny = cur.y + dy[i];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                int nid = ny * w + nx;
                if (closed.count(nid) || costmap[nid] >= 100.0f) continue;

                float g_cost = cur.g + ((dx[i]==0||dy[i]==0)?1.0f:1.414f) + (costmap[nid]/20.0f);
                float dx_g = abs(nx - gx), dy_g = abs(ny - gy);
                float h_cost = (dx_g + dy_g) + (1.414f - 2.0f) * min(dx_g, dy_g);
                open.push({nx, ny, g_cost + h_cost, g_cost, h_cost, cur.x, cur.y});
            }
        }
        return path;
    }

    bool is_path_blocked(int w, const vector<float>& costmap, const vector<Node>& path) {
        for (const auto& n : path) {
            if (costmap[n.x + n.y * w] >= 100.0f) return true;
        }
        return false;
    }
};

// --- DWA Planner ---
vector<RobotState> predict_traj(RobotState s, float v, float w, float sim, float dt) {
    vector<RobotState> t; t.push_back(s);
    for(float time=0; time<sim; time+=dt) {
        s.v = v; s.w = w;
        s.x += s.v * cos(s.theta) * dt; s.y += s.v * sin(s.theta) * dt; s.theta += s.w * dt;
        t.push_back(s);
    }
    return t;
}

Command calculate_dwa(RobotState r, float tx, float ty, const vector<float>& costmap, int w, int h) {
    float best_c = 1e9; Command best_cmd = {0,0};
    for(float v = 0.1; v <= 1.0; v += 0.1) {
        for(float omega = -1.0; omega <= 1.0; omega += 0.1) {
            auto traj = predict_traj(r, v, omega, 2.0, 0.1);
            auto end = traj.back();
            bool crashed = false; float max_cost = 0;
            
            for(auto s : traj) {
                int idx = round(s.x) + round(s.y) * w;
                if(idx >= 0 && idx < w*h) {
                    if(costmap[idx] >= 50.0f) { crashed = true; break; }
                    if(costmap[idx] > max_cost) max_cost = costmap[idx];
                }
            }
            if(crashed) continue;

            float heading_err = atan2(ty - end.y, tx - end.x) - end.theta;
            while(heading_err > M_PI) heading_err -= 2*M_PI;
            while(heading_err < -M_PI) heading_err += 2*M_PI;

            float cost = 1.0*hypot(tx-end.x, ty-end.y) + 500.0*(max_cost/100.0) + 1.0*abs(heading_err) + 10.0*(end.v < 0.05f ? 1.0f : 0.0f);
            if(cost < best_c) { best_c = cost; best_cmd = {v, omega}; }
        }
    }
    
    return best_cmd;
}

// --- MASTER EXECUTION ---
int main() {
    int w = 20, h = 20;
    vector<int> grid(w * h, 0);
    // Maze: each wall leaves a wide gap so robot can always slip through
    // Wall 1: x=4, rows 0..10  → gap rows 11..19
    for(int i=0; i<=10; i++) grid[i*w + 4] = 1;
    // Wall 2: x=9, rows 9..19  → gap rows 0..8
    for(int i=9; i<=19; i++) grid[i*w + 9] = 1;
    // Wall 3: x=14, rows 0..10 → gap rows 11..19
    for(int i=0; i<=10; i++) grid[i*w + 14] = 1;

    AStarPlanner planner;
    // Use tight inflation for static walls, but smaller for dynamic obstacles
    vector<float> static_map = planner.generate_static_costmap(grid, w, h, 1.2, 2.5);
    // Dynamic: smaller inflation (2.0) so human doesn't seal an 8-cell-wide gap
    DynamicCostmap live_map(static_map, w, h, 1.2, 1.5);

    RobotState robot = {1.0, 1.0, 0.0, 0.0, 0.0};
    int gx = 18, gy = 18;
    
    // Initial Plan
    vector<Node> global_path = planner.find_path(1, 1, gx, gy, w, h, static_map);
    
    // Human 1: patrols the bottom section of gap-1 (x=4, y=11..15)
    // They obstruct the lower half of the gap forcing the robot to time its crossing
    Point human1 = {4.0, 13.0};
    float h1_dir = 1.0;

    // Human 2: patrols gap-2 (x=9, y=3..7) — crosses the upper opening
    Point human2 = {9.0, 5.0};
    float h2_dir = 1.0;

    ofstream log("sim_log.csv");
    log << "rx,ry,rtheta,hx1,hy1,hx2,hy2\n";

    cout << "Starting God Mode Simulation..." << endl;

    for (int step = 0; step < 2000; step++) {
        // 1. Move humans
        human1.y += h1_dir * 0.15;
        if (human1.y > 15.5 || human1.y < 11.5) h1_dir *= -1;

        human2.y += h2_dir * 0.15;
        if (human2.y > 7.5 || human2.y < 2.5) h2_dir *= -1;

        live_map.update_live_sensors({human1, human2});

        // 2. Replanning Trigger!
        // If we have no path, OR the current path is blocked, try to find a new one
        if (global_path.empty() || planner.is_path_blocked(w, live_map.get_active_map(), global_path)) {
            global_path = planner.find_path(round(robot.x), round(robot.y), gx, gy, w, h, live_map.get_active_map());
        }

        // 3. Fallback Behavior
        if (global_path.empty()) {
            // If robot is already within the goal radius, finish even though a path is missing
            if (hypot(gx - robot.x, gy - robot.y) < 1.0) {
                cout << "Goal Reached!" << endl;
                break;
            }
            robot.v = 0.0;
            robot.w = 0.0;
            log << robot.x << "," << robot.y << "," << robot.theta << "," << human1.x << "," << human1.y << "," << human2.x << "," << human2.y << "\n";
            continue; 
        }

        // 4. Find Carrot (Proper Pure Pursuit Lookahead)
        float min_dist = 1e9;
        int closest_idx = 0;
        for (size_t i = 0; i < global_path.size(); i++) {
            float d = hypot(robot.x - global_path[i].x, robot.y - global_path[i].y);
            if (d < min_dist) { 
                min_dist = d; 
                closest_idx = i; 
            }
        }
        int lookahead = 20;
int target_idx = min((int)global_path.size() - 1, closest_idx + lookahead); 
        float tx = global_path[target_idx].x;
        float ty = global_path[target_idx].y;

        // 5. DWA
        Command cmd = calculate_dwa(robot, tx, ty, live_map.get_active_map(), w, h);
        auto exec = predict_traj(robot, cmd.v, cmd.w, 0.1, 0.1);
        robot = exec.back();

        log << robot.x << "," << robot.y << "," << robot.theta << "," << human1.x << "," << human1.y << "," << human2.x << "," << human2.y << "\n";

        if (hypot(gx - robot.x, gy - robot.y) < 1.0) {
            cout << "Goal Reached!" << endl;
            break;
        }
    }
    
    log.close();

    ofstream cmap("costmap.csv");
    for(int y = 0; y < h; y++) {
        for(int x = 0; x < w; x++) {
            cmap << live_map.get_active_map()[y * w + x];
            if(x < w-1) cmap << ",";
        }
        cmap << "\n";
    }
    cmap.close();

    cout << "Simulation complete. Run Python visualizer." << endl;
    return 0;
}