#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <unordered_map>

using namespace std;

struct Node {
    int x, y;
    float f, g, h;
    int parent_x = -1;
    int parent_y = -1;
};

struct BFSCell {
    int x, y;
    float dist;
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f;
    }
};

// ============================================================
// THE CLASS: AStarPlanner with integrated costmap generation
// ============================================================
class AStarPlanner {
public:

    // --------------------------------------------------------
    // STEP 1: Brushfire costmap generator (built right in)
    // --------------------------------------------------------
    vector<float> generate_costmap(
        const vector<int>& binary_grid,
        int width, int height,
        float robot_radius, float inflation_radius)
    {
        vector<float> costmap(width * height, 0.0f);
        vector<float> dist_map(width * height, 1e9f);
        queue<BFSCell> q;

        // Ignite the fire: seed every wall cell
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (binary_grid[y * width + x] == 1) {
                    dist_map[y * width + x] = 0.0f;
                    q.push({x, y, 0.0f});
                }
            }
        }

        // Multi-source BFS spread
        int dx[] = {-1, 1,  0, 0, -1, -1,  1, 1};
        int dy[] = { 0, 0, -1, 1, -1,  1, -1, 1};
        float step[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

        while (!q.empty()) {
            BFSCell cur = q.front(); q.pop();
            for (int i = 0; i < 8; i++) {
                int nx = cur.x + dx[i];
                int ny = cur.y + dy[i];
                float nd = cur.dist + step[i];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                if (nd >= inflation_radius) continue;
                if (nd >= dist_map[ny * width + nx]) continue;
                dist_map[ny * width + nx] = nd;
                q.push({nx, ny, nd});
            }
        }

        // Map distances to costs
        float decay = 2.0f;
        for (int i = 0; i < width * height; i++) {
            float d = dist_map[i];
            if      (d <= robot_radius)     costmap[i] = 100.0f;
            else if (d <= inflation_radius) costmap[i] = 100.0f * exp(-decay * (d - robot_radius));
            else                            costmap[i] = 0.0f;
        }
        return costmap;
    }

    // --------------------------------------------------------
    // STEP 2: A* path planner (uses the costmap above)
    // --------------------------------------------------------
    vector<Node> find_path(
        int start_x, int start_y,
        int goal_x,  int goal_y,
        int map_width, int map_height,
        const vector<float>& costmap)
    {
        vector<Node> final_path;
        priority_queue<Node, vector<Node>, CompareNode> open_list;
        unordered_map<int, Node> closed_map;

        Node start;
        start.x = start_x; start.y = start_y;
        start.g = 0.0f; start.h = 0.0f; start.f = 0.0f;
        open_list.push(start);

        int dx[] = {-1, 1,  0, 0, -1, -1,  1, 1};
        int dy[] = { 0, 0, -1, 1, -1,  1, -1, 1};

        while (!open_list.empty()) {
            Node current = open_list.top(); open_list.pop();

            int cur_id = current.y * map_width + current.x;

            // Skip if already processed
            if (closed_map.count(cur_id)) continue;
            closed_map[cur_id] = current;

            // Goal reached — reconstruct path
            if (current.x == goal_x && current.y == goal_y) {
                Node n = current;
                while (n.parent_x != -1 || n.parent_y != -1) {
                    final_path.push_back(n);
                    int pid = n.parent_y * map_width + n.parent_x;
                    n = closed_map[pid];
                }
                final_path.push_back(n); // include start
                reverse(final_path.begin(), final_path.end());
                return final_path;
            }

            // Expand neighbors
            for (int i = 0; i < 8; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                if (nx < 0 || nx >= map_width || ny < 0 || ny >= map_height) continue;

                int nid = ny * map_width + nx;
                if (closed_map.count(nid)) continue;

                // Block lethal cells
                if (costmap[nid] >= 100.0f) continue;

                float step_cost = (dx[i] == 0 || dy[i] == 0) ? 1.0f : 1.414f;
                float penalty   = costmap[nid] / 20.0f;
                float g_cost    = current.g + step_cost + penalty;

                float dist_x = abs(nx - goal_x), dist_y = abs(ny - goal_y);
                float h_cost = (dist_x + dist_y) + (1.414f - 2.0f) * min(dist_x, dist_y);

                Node neighbor;
                neighbor.x = nx; neighbor.y = ny;
                neighbor.g = g_cost; neighbor.h = h_cost; neighbor.f = g_cost + h_cost;
                neighbor.parent_x = current.x; neighbor.parent_y = current.y;
                open_list.push(neighbor);
            }
        }
        return final_path; // empty = no path found
    }

    // --------------------------------------------------------
    // CONVENIENCE: Plan with auto-generated costmap
    // --------------------------------------------------------
    vector<Node> plan(
        const vector<int>& binary_grid,
        int width, int height,
        int start_x, int start_y,
        int goal_x,  int goal_y,
        float robot_radius    = 1.0f,
        float inflation_radius = 3.5f)
    {
        vector<float> costmap = generate_costmap(
            binary_grid, width, height, robot_radius, inflation_radius);
        print_costmap(costmap, width, height);
        return find_path(start_x, start_y, goal_x, goal_y, width, height, costmap);
    }

private:
    void print_costmap(const vector<float>& costmap, int width, int height) {
        cout << "\nINFLATED COSTMAP:\n";
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float v = costmap[y * width + x];
                if (v >= 100.0f) cout << " ██ ";
                else if (v == 0.0f) cout << " .. ";
                else cout << setw(3) << setfill('0') << (int)v << " ";
            }
            cout << "\n";
        }
        cout << "\n";
    }
};

// ============================================================
// MAIN
// ============================================================
int main() {
    AStarPlanner planner;

    int width = 10, height = 10;
    vector<int> grid = {
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

    // Plan from top-left to bottom-right, navigating around the 2x2 wall
    cout << "Planning from (0,0) to (9,9) ...\n";
    vector<Node> path = planner.plan(grid, width, height, 0, 0, 9, 9);

    if (path.empty()) {
        cout << "FAILED: No path found!\n";
    } else {
        cout << "--- FINAL DRIVING PATH ---\n";
        for (const Node& n : path)
            cout << "  [" << n.x << ", " << n.y << "]\n";
        cout << "Total steps: " << path.size() << "\n";
    }
    return 0;
}