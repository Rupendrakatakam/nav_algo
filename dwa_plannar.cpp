#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;

// This represents the physical state of your robot at any given millisecond
struct RobotState {
    float x;
    float y;
    float theta; // Heading in radians
    float v;     // Linear velocity (m/s)
    float w;     // Angular velocity (rad/s)
};

struct Window {
    float v_min;
    float v_max;
    float w_min;
    float w_max;
};

// We need to return two numbers (v and w), so we make a quick struct for it.
struct Command {
    float v;
    float w;
};

//walls and obstrucles
struct Point {
    float x;
    float y;
};

// =======================================================================
// YOUR ASSIGNMENT: Build the Physics Simulator
// =======================================================================
vector<RobotState> predict_trajectory(RobotState start_state, float target_v, float target_w, float sim_time, float dt) {
    vector<RobotState> trajectory;
    
    RobotState current_state = start_state;
    trajectory.push_back(current_state);
    
    float time = 0.0;

    while(time < sim_time){
        current_state.v = target_v;
        current_state.w = target_w;
        float x_next = current_state.x + current_state.v * cos(current_state.theta)*dt;
        float y_next = current_state.y + current_state.v * sin(current_state.theta)*dt;
        float theta_next = current_state.theta + current_state.w*dt;

        current_state.x = x_next;
        current_state.y = y_next;
        current_state.theta = theta_next;

        trajectory.push_back(current_state);
        time += dt;
    }

    return trajectory;
}

Window generate_dynamic_window(RobotState robot, float max_v, float min_v, float max_w, float min_w, float max_accel, float max_decel, float dt) {
    
    // How much velocity can we physically change in one time step?
    float d_v_max = robot.v + (max_accel * dt);
    float d_v_min = robot.v + (max_decel * dt);
    float d_w_max = robot.w + (max_accel * dt);
    float d_w_min = robot.w + (max_decel * dt);

    // The Window is the most restrictive combination of our hardware limits and our physics limits
    Window win;
    win.v_max = min(max_v, d_v_max);
    win.v_min = max(min_v, d_v_min);
    win.w_max = min(max_w, d_w_max);
    win.w_min = max(min_w, d_w_min);
    
    return win;
}

// A trajectory is a vector of RobotStates. 
// So a list of trajectories is a vector of vectors!
vector<vector<RobotState>> generate_all_trajectories(RobotState robot, Window win, float sim_time, float dt, float v_res, float w_res) {
    vector<vector<RobotState>> all_rollouts;

    for(float v = win.v_min; v <= win.v_max; v += v_res){
        for(float w = win.w_min; w <= win.w_max; w += w_res){
            all_rollouts.push_back(predict_trajectory(robot, v, w, sim_time, dt));    
        }
    }
    return all_rollouts;
}

Command calculate_best_command(vector<vector<RobotState>> all_trajectories, float goal_x, float goal_y, vector<Point> obstacles, float robot_radius) {
    float best_cost = numeric_limits<float>::max(); // Start with infinitely high cost
    Command best_cmd = {0.0, 0.0}; // Default command is to stop

    float alpha = 1.0; // Weight for goal distance
    float gamma = 3.0; // Weight for speed
    float beta = 0.2; // Weight for clearance
    float delta = 1.5; // Weight for heading 
    for(auto traj : all_trajectories){
        // --- 1. GOAL DISTANCE (Only look at the finish line) ---
        auto final_state = traj.back();
        float goal_distance = sqrt(pow(goal_x - final_state.x, 2) + pow(goal_y - final_state.y, 2));
        float min_dist_to_obstacle = numeric_limits<float>::max();
        bool crashed = false;

        for (auto state : traj){ 
            for (auto obs : obstacles){
                float dist = sqrt(pow(obs.x - state.x, 2) + pow(obs.y - state.y, 2));
                // If the distance is less than the robot's physical radius, it crashed!
                if (dist <= robot_radius) {
                    crashed = true;
                    break;
                }
                // Keep track of the absolute closest we ever got to an obstacle
                if (dist < min_dist_to_obstacle) {
                    min_dist_to_obstacle = dist;
                }
            }
            if (crashed){
                break;
            }
        }
        if(crashed) continue;     // Stop checking this trajectory if we already hit something
        
        // 4. The New Cost Formula
        float clearance_cost = 1.0 / min_dist_to_obstacle; // Higher cost if we get too close
        
        // --- NEW: HEADING ALIGNMENT ---
        // 1. What angle is the goal relative to the world?
        float angle_to_goal = atan2(goal_y - final_state.y, goal_x - final_state.x);
        
        // 2. What is the difference between where we are looking and where the goal is?
        float heading_error = angle_to_goal - final_state.theta;
        
        // 3. Normalize the error to be between -PI and +PI
        while (heading_error > M_PI)  heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        float heading_cost = abs(heading_error);

        // Add beta * clearance_cost to your final cost calculation!
        float cost = (alpha * goal_distance) + (beta*clearance_cost) + (delta*heading_cost) - (gamma * final_state.v);
        if(cost < best_cost){
            best_cost = cost;
            best_cmd = {final_state.v, final_state.w};
        }
    }

    return best_cmd;
}

#include <fstream> // NEW: For writing to CSV

// ... (Keep all your structs and DWA functions exactly the same) ...

int main() {
    RobotState robot = {0.0, 0.0, 0.0, 0.0, 0.0};
    float goal_x = 3.0;
    float goal_y = 3.0;
    float robot_radius = 0.3;
    
    // Let's create a "wall" of obstacles to test our visualizer
    vector<Point> obstacles = { 
        {1.5, 1.2}, {1.5, 1.5}, {1.5, 1.8}, {1.5, 2.1} 
    };

    float max_v = 1.0;
    float min_v = 0.0; // No reversing
    float max_w = 1.0;
    float min_w = -1.0;
    float max_accel = 2.0;
    float max_decel = -2.0;
    
    float sim_time = 3.0; 
    float dt = 0.1;       

    // --- NEW: OPEN THE CSV FILE ---
    ofstream log_file("robot_path.csv");
    log_file << "x,y,theta" << "\n"; // Write the header
    log_file << robot.x << "," << robot.y << "," << robot.theta << "\n";

    cout << "STARTING DWA NAVIGATION..." << endl;
    
    int iteration = 0;
    float dist_to_goal = hypot(goal_x - robot.x, goal_y - robot.y);

    while (dist_to_goal > 0.1) {
        Window dynamic_window = generate_dynamic_window(robot, max_v, min_v, max_w, min_w, max_accel, max_decel, dt);
        vector<vector<RobotState>> all_trajectories = generate_all_trajectories(robot, dynamic_window, sim_time, dt, 0.1, 0.1);
        Command best_cmd = calculate_best_command(all_trajectories, goal_x, goal_y, obstacles, robot_radius);
        
        vector<RobotState> execution = predict_trajectory(robot, best_cmd.v, best_cmd.w, dt, dt);
        robot = execution.back(); 
        
        // --- NEW: LOG THE DATA ---
        log_file << robot.x << "," << robot.y << "," << robot.theta << "\n";

        dist_to_goal = hypot(goal_x - robot.x, goal_y - robot.y);
        iteration++;
        
        if (iteration > 400) {
            cout << "SIMULATION TIMEOUT." << endl;
            break;
        }
    }

    log_file.close(); // Close the file
    cout << "\nFinished. Data written to robot_path.csv" << endl;

    return 0;
}