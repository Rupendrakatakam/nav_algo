use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};

#[derive(Clone, Debug)]
struct Node {
    x: i32,
    y: i32,
    f: f32,
    g: f32,
    h: f32,
    parent_x: i32,
    parent_y: i32,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // We want a min-heap based on `f`, so we reverse the comparison:
        other.f.partial_cmp(&self.f)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

struct BFSCell {
    x: i32,
    y: i32,
    dist: f32,
}

struct AStarPlanner;

impl AStarPlanner {
    fn new() -> Self {
        AStarPlanner
    }

    fn generate_costmap(
        &self,
        binary_grid: &[i32],
        width: usize,
        height: usize,
        robot_radius: f32,
        inflation_radius: f32,
    ) -> Vec<f32> {
        let mut costmap = vec![0.0; width * height];
        let mut dist_map = vec![1e9_f32; width * height];
        let mut q = VecDeque::new();

        for y in 0..height {
            for x in 0..width {
                if binary_grid[y * width + x] == 1 {
                    dist_map[y * width + x] = 0.0;
                    q.push_back(BFSCell {
                        x: x as i32,
                        y: y as i32,
                        dist: 0.0,
                    });
                }
            }
        }

        let dx = [-1, 1, 0, 0, -1, -1, 1, 1];
        let dy = [0, 0, -1, 1, -1, 1, -1, 1];
        let step = [
            1.0, 1.0, 1.0, 1.0, 1.414_f32, 1.414_f32, 1.414_f32, 1.414_f32,
        ];

        while let Some(cur) = q.pop_front() {
            for i in 0..8 {
                let nx = cur.x + dx[i];
                let ny = cur.y + dy[i];
                let nd = cur.dist + step[i];

                if nx < 0 || nx >= width as i32 || ny < 0 || ny >= height as i32 {
                    continue;
                }
                if nd >= inflation_radius {
                    continue;
                }
                let idx = (ny as usize) * width + (nx as usize);
                if nd >= dist_map[idx] {
                    continue;
                }
                dist_map[idx] = nd;
                q.push_back(BFSCell { x: nx, y: ny, dist: nd });
            }
        }

        let decay = 2.0_f32;
        for i in 0..(width * height) {
            let d = dist_map[i];
            if d <= robot_radius {
                costmap[i] = 100.0;
            } else if d <= inflation_radius {
                costmap[i] = 100.0 * (-decay * (d - robot_radius)).exp();
            } else {
                costmap[i] = 0.0;
            }
        }
        costmap
    }

    fn find_path(
        &self,
        start_x: i32,
        start_y: i32,
        goal_x: i32,
        goal_y: i32,
        map_width: usize,
        map_height: usize,
        costmap: &[f32],
    ) -> Vec<Node> {
        let mut final_path = Vec::new();
        let mut open_list = BinaryHeap::new();
        let mut closed_map: HashMap<i32, Node> = HashMap::new();

        open_list.push(Node {
            x: start_x,
            y: start_y,
            f: 0.0,
            g: 0.0,
            h: 0.0,
            parent_x: -1,
            parent_y: -1,
        });

        let dx = [-1, 1, 0, 0, -1, -1, 1, 1];
        let dy = [0, 0, -1, 1, -1, 1, -1, 1];

        while let Some(current) = open_list.pop() {
            let cur_id = current.y * (map_width as i32) + current.x;

            if closed_map.contains_key(&cur_id) {
                continue;
            }
            closed_map.insert(cur_id, current.clone());

            if current.x == goal_x && current.y == goal_y {
                let mut n = current;
                while n.parent_x != -1 || n.parent_y != -1 {
                    final_path.push(n.clone());
                    let pid = n.parent_y * (map_width as i32) + n.parent_x;
                    n = closed_map.get(&pid).cloned().unwrap();
                }
                final_path.push(n);
                final_path.reverse();
                return final_path;
            }

            for i in 0..8 {
                let nx = current.x + dx[i];
                let ny = current.y + dy[i];

                if nx < 0 || nx >= map_width as i32 || ny < 0 || ny >= map_height as i32 {
                    continue;
                }

                let nid = ny * (map_width as i32) + nx;
                if closed_map.contains_key(&nid) {
                    continue;
                }

                let cost_idx = (ny as usize) * map_width + (nx as usize);
                if costmap[cost_idx] >= 100.0 {
                    continue;
                }

                let step_cost = if dx[i] == 0 || dy[i] == 0 { 1.0 } else { 1.414 };
                let penalty = costmap[cost_idx] / 20.0;
                let g_cost = current.g + step_cost + penalty;

                let dist_x = (nx - goal_x).abs() as f32;
                let dist_y = (ny - goal_y).abs() as f32;
                let min_dist = if dist_x < dist_y { dist_x } else { dist_y };
                let h_cost = (dist_x + dist_y) + (1.414 - 2.0) * min_dist;

                open_list.push(Node {
                    x: nx,
                    y: ny,
                    f: g_cost + h_cost,
                    g: g_cost,
                    h: h_cost,
                    parent_x: current.x,
                    parent_y: current.y,
                });
            }
        }
        final_path
    }

    fn print_costmap(&self, costmap: &[f32], width: usize, height: usize) {
        println!("\nINFLATED COSTMAP:");
        for y in 0..height {
            for x in 0..width {
                let v = costmap[y * width + x];
                if v >= 100.0 {
                    print!(" ██ ");
                } else if v == 0.0 {
                    print!(" .. ");
                } else {
                    print!("{:03} ", v as i32);
                }
            }
            println!();
        }
        println!();
    }

    fn plan(
        &self,
        binary_grid: &[i32],
        width: usize,
        height: usize,
        start_x: i32,
        start_y: i32,
        goal_x: i32,
        goal_y: i32,
        robot_radius: Option<f32>,
        inflation_radius: Option<f32>,
    ) -> Vec<Node> {
        let robot_radius = robot_radius.unwrap_or(1.0);
        let inflation_radius = inflation_radius.unwrap_or(3.5);

        let costmap = self.generate_costmap(
            binary_grid,
            width,
            height,
            robot_radius,
            inflation_radius,
        );
        self.print_costmap(&costmap, width, height);

        self.find_path(start_x, start_y, goal_x, goal_y, width, height, &costmap)
    }
}

fn main() {
    let planner = AStarPlanner::new();

    let width = 10;
    let height = 10;
    let grid = vec![
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    println!("Planning from (0,0) to (9,9) ...");
    let path = planner.plan(&grid, width, height, 0, 0, 9, 9, None, None);

    if path.is_empty() {
        println!("FAILED: No path found!");
    } else {
        println!("--- FINAL DRIVING PATH ---");
        for n in &path {
            println!("  [{}, {}]", n.x, n.y);
        }
        println!("Total steps: {}", path.len());
    }
}
