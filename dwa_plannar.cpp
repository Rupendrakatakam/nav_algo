#include <iostream>
#include <vector>
#include <cmath>

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

Command calculate_best_command(vector<vector<RobotState>> all_trajectories, float goal_x, float goal_y) {
    float best_cost = numeric_limits<float>::max(); // Start with infinitely high cost
    Command best_cmd = {0.0, 0.0}; // Default command is to stop

    float alpha = 1.0; // Weight for goal distance
    float gamma = 0.2; // Weight for speed

    // TODO: Write the scoring loop!
    // 1. Loop through 'all_trajectories'
    // 2. Extract the final RobotState of the current trajectory: auto final_state = traj.back();
    // 3. Calculate distance to goal (goal_x, goal_y)
    // 4. Calculate the cost: (alpha * distance) - (gamma * final_state.v)
    // 5. If this cost is less than 'best_cost', update 'best_cost' and set 'best_cmd' to this trajectory's v and w!
    for(auto traj : all_trajectories){
        auto final_state = traj.back();
        float distance = sqrt(pow(goal_x - final_state.x, 2) + pow(goal_y - final_state.y, 2));
        float cost = (alpha * distance) - (gamma * final_state.v);
        if(cost < best_cost){
            best_cost = cost;
            best_cmd = {final_state.v, final_state.w};
        }
    }

    return best_cmd;
}

int main() {
    // 1. Our robot is sitting at the origin, facing completely straight (0 radians), stopped.
    RobotState robot = {0.0, 0.0, 0.0, 0.0, 0.0};

    float goal_x = 2.0;
    float goal_y = 2.0;

    //physical limits
    float max_v = 1.0;
    float min_v = -1.0;
    float max_w = 0.5;
    float min_w = -0.5;
    float max_accel = 2.0;
    float max_decel = -2.0;
    
    // 2. We want to test what happens if we command it to drive forward at 1.0 m/s 
    //    while slightly turning left at 0.5 rad/s.
    float test_v = 1.0;
    float test_w = 0.5;
    float sim_time = 3.0; // Predict 3 seconds into the future
    float dt = 0.1;       // Calculate a frame every 0.1 seconds

    cout << "Simulating Trajectory: v = " << test_v << " m/s, w = " << test_w << " rad/s" << endl;
    
    vector<RobotState> predicted_path = predict_trajectory(robot, test_v, test_w, sim_time, dt);
    Window dynamic_window = generate_dynamic_window(robot, max_v, min_v, max_w, min_w, max_accel, max_decel, dt);

    // 3. Print out the predicted path to verify the physics
    // for (int i = 0; i < predicted_path.size(); i += 5) { // Print every 0.5 seconds to save screen space
    //     cout << "Time: " << i * dt << "s -> x: " << predicted_path[i].x 
    //          << ", y: " << predicted_path[i].y 
    //          << ", theta: " << predicted_path[i].theta << endl;
    // }

    vector<vector<RobotState>> all_trajectories = generate_all_trajectories(robot, dynamic_window, sim_time, dt, 0.1, 0.1);
    cout << "Generated " << all_trajectories.size() << " trajectories." << endl;
    for(int i = 0; i < all_trajectories.size(); i++){
        cout << "Trajectory " << i << ": " << all_trajectories[i].size() << " states" << endl;
    }

    return 0;
}