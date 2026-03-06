#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std;

// ==============================================================================
// TODO: PASTE YOUR Node STRUCT, CompareNode STRUCT, AND AStarPlanner CLASS HERE!
// (Ensure you applied the int8_t and Wall Threshold updates!)
// ==============================================================================

// The Blueprint (Define this outside main)
struct Node {
    int x;
    int y;
    
    float f;
    float g;
    float h;

    int parent_x = -1;
    int parent_y = -1;
};

// Comparator for the priority queue to sort by lowest f cost
struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f; // '>' because priority_queue is a max-heap by default, this makes it a min-heap
    }
};

// THE NEW ARCHITECTURE: The Class
class AStarPlanner {
public:
    // This is the function the ROS node will eventually call!
    vector<Node> find_path(int start_x, int start_y, int goal_x, int goal_y, int map_width, int map_height, const vector<int8_t>& grid) {
        
        vector<Node> final_path;
        priority_queue<Node, vector<Node>, CompareNode> open_list;
        unordered_map<int, Node> closed_map;
        
        Node my_start_node;
        my_start_node.x = start_x;
        my_start_node.y = start_y;
        my_start_node.g = 0;
        my_start_node.h = 0;
        my_start_node.f = my_start_node.g + my_start_node.h;

        open_list.push(my_start_node);

        while (!open_list.empty()){
            Node current_node = open_list.top();
            open_list.pop();

            // 1. Calculate the unique 1D ID for this coordinate
            int current_id = (current_node.y * map_width) + current_node.x;

            // 2. O(1) Lookup: Check if it exists in the Hash Map
            if (closed_map.find(current_id) != closed_map.end()) {
                continue; // We already expanded this! Throw it away.
            }

            // 3. Save it to the Hash Map
            closed_map[current_id] = current_node;

            // --- THE GOAL CHECK ---
            if (current_node.x == goal_x && current_node.y == goal_y) {
                
                Node trace_node = current_node;

                // 1. Loop backwards until we find the Start Node (parent_x == -1)
                while (trace_node.parent_x != -1) {
                    // Add the current step to our path
                    final_path.push_back(trace_node);
                    
                    // 1. Calculate the unique ID of the parent
                    int parent_id = (trace_node.parent_y * map_width) + trace_node.parent_x;
                    
                    // 2. Instantly grab the parent node from the Hash Map!
                    trace_node = closed_map[parent_id];
                }
                // Add the start node itself to complete the path
                final_path.push_back(trace_node); 

                // Reverse the path vector to return it Start -> Goal
                reverse(final_path.begin(), final_path.end());

                break; // Shatter the master while loop. The algorithm is finished.
            }
            
            float step_cost = 0;
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

                    if (neighbor_x >= 0 && neighbor_x < map_width && neighbor_y >= 0 && neighbor_y < map_height){
                        if (grid[neighbor_y * map_width + neighbor_x] >= 50 || grid[neighbor_y * map_width + neighbor_x] == -1){
                            continue; // Treat obstacles (>=50) and unknown space (-1) as solid walls!
                        }
                        else {
                            if (dx == 0 || dy == 0){
                                step_cost = 1.0;
                            }
                            else if (dx != 0 && dy != 0){
                                step_cost = 1.414;
                            }
                        }

                        int neighbor_id = (neighbor_y * map_width) + neighbor_x;
                        if (closed_map.find(neighbor_id) != closed_map.end()) {
                            continue; // Skip this neighbor, it's already closed!
                        }
                        
                        float dist_x = abs(neighbor_x - goal_x);
                        float dist_y = abs(neighbor_y - goal_y);

                        float g_cost = current_node.g + step_cost;
                        float h_cost = 1.0 * (dist_x + dist_y) + (1.414 - 2.0 * 1.0) * std::min(dist_x, dist_y);
                        float f_cost = g_cost + h_cost;

                        // Push directly to the priority queue. 
                        Node new_neighbor_node;
                        new_neighbor_node.x = neighbor_x;
                        new_neighbor_node.y = neighbor_y;
                        new_neighbor_node.g = g_cost;
                        new_neighbor_node.h = h_cost;
                        new_neighbor_node.f = f_cost;
                        new_neighbor_node.parent_x = current_node.x;
                        new_neighbor_node.parent_y = current_node.y;
                        open_list.push(new_neighbor_node);
                    }
                }
            }
        }
        
        return final_path; 
    }
};

class AStarNode : public rclcpp::Node {
public:
    AStarNode() : Node("astar_node") {
        // 1. Setup Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarNode::map_callback, this, std::placeholders::_1));
        
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&AStarNode::start_callback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarNode::goal_callback, this, std::placeholders::_1));
        
        // 2. Setup Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        
        RCLCPP_INFO(this->get_logger(), "A* Node initialized. Waiting for /map, /initialpose, and /goal_pose...");
    }

private:
    nav_msgs::msg::OccupancyGrid map_data_;
    bool map_received_ = false;
    bool start_received_ = false;
    double start_world_x_, start_world_y_;

    AStarPlanner planner_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_data_ = *msg;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received! Width: %d, Height: %d", msg->info.width, msg->info.height);
    }

    void start_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        start_world_x_ = msg->pose.pose.position.x;
        start_world_y_ = msg->pose.pose.position.y;
        start_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Start pose received!");
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!map_received_ || !start_received_) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan. Missing map or start pose.");
            return;
        }

        double goal_world_x = msg->pose.position.x;
        double goal_world_y = msg->pose.position.y;
        double res = map_data_.info.resolution;
        double origin_x = map_data_.info.origin.position.x;
        double origin_y = map_data_.info.origin.position.y;

        // 1. Coordinate Transform: Real World (meters) to Grid (pixels)
        int start_grid_x = (start_world_x_ - origin_x) / res;
        int start_grid_y = (start_world_y_ - origin_y) / res;
        int goal_grid_x = (goal_world_x - origin_x) / res;
        int goal_grid_y = (goal_world_y - origin_y) / res;

        RCLCPP_INFO(this->get_logger(), "Planning from grid [%d, %d] to [%d, %d]", start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);

        // 2. Trigger the Core Engine
        vector<::Node> grid_path = planner_.find_path(
            start_grid_x, start_grid_y, goal_grid_x, goal_grid_y, 
            map_data_.info.width, map_data_.info.height, map_data_.data);

        if (grid_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed! Target might be inside a wall.");
            return;
        }

        // 3. Coordinate Transform: Grid (pixels) back to Real World (meters)
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto& node : grid_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = (node.x * res) + origin_x;
            pose.pose.position.y = (node.y * res) + origin_y;
            pose.pose.position.z = 0.0;
            path_msg.poses.push_back(pose);
        }

        // 4. Send it to the motors!
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Path published to /plan successfully!");
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}
