use std::cell::RefCell;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::{Duration, Instant};
use std::{collections::HashMap, rc::Rc};

use clap::Parser;
use rand::Rng;

fn main() {
    let args = Args::parse();

    let grid = match read_grid(&args.grid_file) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("Error reading grid file: {}", e);
            std::process::exit(1);
        }
    };

    let root = Rc::new(RefCell::new(MonteCarloTreeSearchNode::new(
        Drone::new(grid, (0, 0)),
        None,
        None,
    )));
    root.borrow_mut().initialize(Rc::clone(&root));

    let allowed_time = Duration::from_millis(args.time);
    let best_path = root.borrow().best_path(allowed_time);
    println!("Best path: {:?}", best_path);
}

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Time allowed for the algorithm in milliseconds
    #[arg(short, long, default_value_t = 120000)]
    time: u64,

    /// Path to the input grid file
    #[arg(short, long)]
    grid_file: PathBuf,
}

fn read_grid(path: &PathBuf) -> Result<Vec<Vec<usize>>, String> {
    let file = File::open(path).map_err(|e| format!("Failed to open file: {}", e))?;
    let reader = BufReader::new(file);
    let mut grid = Vec::<Vec<usize>>::new();

    // Read all lines
    for (i, line) in reader.lines().enumerate() {
        let line = line.map_err(|e| format!("Error reading line {}: {}", i + 1, e))?;
        let row: Vec<usize> = line
            .split_whitespace()
            .map(|num| {
                num.parse()
                    .map_err(|_| format!("Invalid number '{}' at line {}", num, i + 1))
            })
            .collect::<Result<Vec<usize>, String>>()?;

        // Check if this row has the same length as the first row
        if !grid.is_empty() && row.len() != grid[0].len() {
            return Err(format!(
                "Inconsistent row length at line {}: expected {}, got {}",
                i + 1,
                grid[0].len(),
                row.len()
            ));
        }

        grid.push(row);
    }

    // Validate grid is not empty
    if grid.is_empty() {
        return Err("Grid is empty".to_string());
    }
    if grid[0].is_empty() {
        return Err("Grid has empty rows".to_string());
    }

    Ok(grid)
}

#[derive(Clone, Debug)]
struct Drone {
    initial_position: (usize, usize),
    original_grid: Vec<Vec<usize>>,
    current_position: (usize, usize),
    score: usize,
    grid: Vec<Vec<usize>>,
    grid_size: (usize, usize),
    visited_score_deltas: HashMap<(usize, usize), usize>,
    move_number: usize,
    configuration: GameConfiguration,
    current_position_score: usize,
}

impl Drone {
    pub fn new(grid: Vec<Vec<usize>>, initial_position: (usize, usize)) -> Self {
        let initial_score = grid[initial_position.0][initial_position.1];
        let mut new_grid = grid.clone();
        new_grid[initial_position.0][initial_position.1] = 0;
        Self {
            initial_position,
            original_grid: grid.clone(),
            current_position: initial_position,
            score: initial_score,
            grid_size: (grid.len(), grid[0].len()),
            visited_score_deltas: vec![(initial_position, initial_score)]
                .into_iter()
                .collect(),
            grid: new_grid,
            move_number: 0,
            configuration: GameConfiguration {
                number_of_moves_per_rollout: 1000,
            },
            current_position_score: initial_score,
        }
    }

    pub fn reset(&self) -> Self {
        Self {
            initial_position: self.current_position,
            original_grid: self.original_grid.clone(),
            current_position: self.current_position,
            score: self.grid[self.current_position.0][self.current_position.1],
            grid: self.grid.clone(),
            grid_size: self.grid_size,
            visited_score_deltas: self.visited_score_deltas.clone(),
            move_number: 0,
            configuration: self.configuration.clone(),
            current_position_score: self.current_position_score,
        }
    }

    pub fn legal_moves(&self) -> Vec<Move> {
        if self.current_position == (0, 0) {
            vec![Move::East, Move::South, Move::SouthEast]
        } else if self.current_position == (0, self.grid_size.1 - 1) {
            vec![Move::West, Move::South, Move::SouthWest]
        } else if self.current_position == (self.grid_size.0 - 1, 0) {
            vec![Move::East, Move::North, Move::NorthEast]
        } else if self.current_position == (self.grid_size.0 - 1, self.grid_size.1 - 1) {
            vec![Move::West, Move::North, Move::NorthWest]
        } else if self.current_position.0 == 0 {
            vec![
                Move::East,
                Move::South,
                Move::SouthEast,
                Move::SouthWest,
                Move::West,
            ]
        } else if self.current_position.1 == 0 {
            vec![
                Move::North,
                Move::South,
                Move::East,
                Move::SouthEast,
                Move::NorthEast,
            ]
        } else if self.current_position.0 == self.grid_size.0 - 1 {
            vec![
                Move::East,
                Move::North,
                Move::NorthEast,
                Move::NorthWest,
                Move::West,
            ]
        } else if self.current_position.1 == self.grid_size.1 - 1 {
            vec![
                Move::North,
                Move::South,
                Move::West,
                Move::SouthWest,
                Move::NorthWest,
            ]
        } else {
            vec![
                Move::North,
                Move::South,
                Move::East,
                Move::West,
                Move::NorthWest,
                Move::NorthEast,
                Move::SouthEast,
                Move::SouthWest,
            ]
        }
    }

    pub fn is_done_patrolling(&self) -> bool {
        self.move_number >= self.configuration.number_of_moves_per_rollout
    }

    pub fn patrol_score(&self) -> usize {
        self.score
    }

    // Out of simplicity, we increase the score of the visited positions by 1 at each time step (we assume that we have one move per time step)
    pub fn advance(&self, direction: &Move) -> Drone {
        let (current_x, current_y) = self.current_position;
        let (next_x, next_y) = match direction {
            Move::North => (current_x - 1, current_y),
            Move::South => (current_x + 1, current_y),
            Move::East => (current_x, current_y + 1),
            Move::West => (current_x, current_y - 1),
            Move::NorthWest => (current_x - 1, current_y - 1),
            Move::NorthEast => (current_x - 1, current_y + 1),
            Move::SouthEast => (current_x + 1, current_y + 1),
            Move::SouthWest => (current_x + 1, current_y - 1),
        };
        let mut new_grid = self.grid.clone();
        let move_score = new_grid[next_x][next_y];
        new_grid[next_x][next_y] = 0;
        let mut new_visited_score_deltas = self.visited_score_deltas.clone();
        let visited_positions = new_visited_score_deltas
            .keys()
            .cloned()
            .collect::<Vec<(usize, usize)>>();
        for (x, y) in visited_positions {
            new_grid[x][y] = new_grid[x][y] + 1;
            let current_score_delta = new_visited_score_deltas.get(&(x, y)).unwrap();
            let new_score_delta = if *current_score_delta == 0 {
                0
            } else {
                current_score_delta - 1
            };
            if new_score_delta == 0 {
                new_visited_score_deltas.remove(&(x, y));
            } else {
                new_visited_score_deltas.insert((x, y), new_score_delta);
            }
        }
        new_visited_score_deltas.insert((next_x, next_y), move_score);
        let new_score = self.score + move_score;
        Self {
            initial_position: self.initial_position,
            original_grid: self.original_grid.clone(),
            current_position: (next_x, next_y),
            score: new_score,
            grid: new_grid,
            grid_size: self.grid_size,
            visited_score_deltas: new_visited_score_deltas,
            move_number: self.move_number + 1,
            configuration: self.configuration.clone(),
            current_position_score: move_score,
        }
    }

    pub fn current_position_score(&self) -> usize {
        self.current_position_score
    }
}

#[derive(Clone, Debug)]
pub struct MonteCarloTreeSearchNode {
    drone: Drone,
    parent: Option<Rc<RefCell<MonteCarloTreeSearchNode>>>,
    parent_move: Option<Move>,
    children: RefCell<Vec<Rc<RefCell<MonteCarloTreeSearchNode>>>>,
    number_of_visits: RefCell<usize>,
    total_score: RefCell<usize>,
    untried_moves: RefCell<Vec<Move>>,
    self_ref: Option<Rc<RefCell<MonteCarloTreeSearchNode>>>,
    c_param: f64,
}

impl MonteCarloTreeSearchNode {
    pub(crate) fn new(
        drone: Drone,
        parent: Option<Rc<RefCell<MonteCarloTreeSearchNode>>>,
        parent_move: Option<Move>,
    ) -> Self {
        Self {
            untried_moves: RefCell::new(drone.legal_moves()),
            drone,
            parent,
            parent_move,
            self_ref: None,
            children: RefCell::new(Vec::new()),
            number_of_visits: RefCell::new(0),
            total_score: RefCell::new(0),
            c_param: 1.5,
        }
    }

    pub fn expand(&self) -> Rc<RefCell<MonteCarloTreeSearchNode>> {
        let drone_move = self.untried_moves.borrow_mut().pop().unwrap();

        // Create new node with proper parent reference
        let new_node = Rc::new(RefCell::new(MonteCarloTreeSearchNode::new(
            self.drone.advance(&drone_move),
            Some(Rc::clone(&self.self_ref.as_ref().unwrap())),
            Some(drone_move),
        )));
        new_node.borrow_mut().initialize(Rc::clone(&new_node));

        self.children.borrow_mut().push(Rc::clone(&new_node));

        new_node
    }

    pub fn initialize(&mut self, self_ref: Rc<RefCell<MonteCarloTreeSearchNode>>) {
        self.self_ref = Some(self_ref);
    }

    pub fn is_terminal_node(&self) -> bool {
        self.drone.is_done_patrolling()
    }

    pub fn n(&self) -> usize {
        *self.number_of_visits.borrow()
    }

    pub fn q(&self) -> usize {
        *self.total_score.borrow()
    }

    pub fn rollout_policy(&self, possible_moves: Vec<Move>) -> Move {
        let random_index = rand::thread_rng().gen_range(0..possible_moves.len());
        possible_moves[random_index].clone()
    }

    pub fn rollout(&self) -> usize {
        let mut drone = self.drone.reset();
        while !drone.is_done_patrolling() {
            let possible_moves = drone.legal_moves();
            let drone_move = self.rollout_policy(possible_moves);
            drone = drone.advance(&drone_move);
        }
        drone.patrol_score()
    }

    pub fn backpropagate(&self, result: usize) {
        *self.number_of_visits.borrow_mut() += 1;
        *self.total_score.borrow_mut() += result;
        let mut parent = self.parent.clone();
        loop {
            if let Some(p) = parent {
                *p.borrow().number_of_visits.borrow_mut() += 1;
                *p.borrow().total_score.borrow_mut() += result;
                parent = p.borrow().parent.clone();
            } else {
                break;
            }
        }
    }

    pub fn is_fully_expanded(&self) -> bool {
        self.untried_moves.borrow().is_empty()
    }

    pub fn best_child(&self, c_param: f64) -> Rc<RefCell<MonteCarloTreeSearchNode>> {
        let index_of_best_child = self
            .children
            .borrow()
            .iter()
            .enumerate()
            .map(|(index, child)| {
                let n = child.borrow().n() as f64;
                let q = child.borrow().q() as f64;
                (
                    index,
                    (q / n) + c_param * (2.0 * (self.n() as f64 / n)).sqrt(),
                )
            })
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        Rc::clone(&self.children.borrow()[index_of_best_child.unwrap().0])
    }

    pub fn tree_policy(&self) -> (Rc<RefCell<MonteCarloTreeSearchNode>>, Vec<Move>, usize) {
        let self_ref = self.self_ref.as_ref().unwrap().clone();
        let mut node_ref = self_ref;
        let mut path = vec![];
        let mut score = self.drone.current_position_score();
        loop {
            // Check terminal state
            let is_terminal = node_ref.borrow().is_terminal_node();
            if is_terminal {
                return (node_ref, path, score);
            }

            // Check if needs expansion
            let needs_expansion = !node_ref.borrow().is_fully_expanded();
            if needs_expansion {
                let new_node = node_ref.borrow().expand();
                return (new_node, path, score);
            }

            // Get best child
            let next_node = {
                let node = node_ref.borrow();
                node.best_child(node.c_param)
            };
            if let Some(parent_move) = next_node.borrow().parent_move.clone() {
                path.push(parent_move);
                score += next_node.borrow().drone.current_position_score();
            }

            // Move to next node
            node_ref = next_node;
        }
    }

    pub fn best_path(&self, allowed_time: Duration) -> (Vec<Move>, usize) {
        let start_time = Instant::now();
        let mut best_path = (vec![], 0);

        while start_time.elapsed() < allowed_time {
            let (node_from_last_expansion, path, score) = {
                let root = self.self_ref.as_ref().unwrap().borrow();
                root.tree_policy()
            };
            if score > best_path.1 {
                best_path = (path, score);
            }

            let simulation_score = node_from_last_expansion.borrow_mut().rollout();
            node_from_last_expansion
                .borrow()
                .backpropagate(simulation_score);
        }
        best_path
    }
}

#[derive(Clone, Debug)]
struct GameConfiguration {
    number_of_moves_per_rollout: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Move {
    North,
    South,
    East,
    West,
    NorthWest,
    NorthEast,
    SouthEast,
    SouthWest,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_game_state() {
        // Test initialization
        let grid = vec![vec![5, 2, 3], vec![4, 5, 6], vec![7, 8, 9]];
        let state = Drone::new(grid.clone(), (0, 0));
        let mut new_grid = grid.clone();
        new_grid[0][0] = 0;
        assert_eq!(state.current_position, (0, 0));
        assert_eq!(state.score, 5);
        assert_eq!(state.grid_size, (3, 3));
        assert_eq!(state.grid, new_grid);
        assert_eq!(state.move_number, 0);

        // Test legal moves from corner
        let corner_moves = state.legal_moves();
        assert!(corner_moves.contains(&Move::East));
        assert!(corner_moves.contains(&Move::South));
        assert!(corner_moves.contains(&Move::SouthEast));
        assert_eq!(corner_moves.len(), 3);

        // Test state transition
        let next_state = state.advance(&Move::East);
        assert_eq!(next_state.current_position, (0, 1));
        assert_eq!(next_state.score, 7); // Moved to position with value 2
        assert_eq!(next_state.move_number, 1);
        assert_eq!(next_state.grid[0][1], 0); // Position visited becomes 0
        assert_eq!(next_state.grid[0][0], 1); // Position visited becomes 0
        assert_eq!(next_state.visited_score_deltas.len(), 2);

        // Test visited position increment
        let visited_pos = (0, 1);
        assert!(next_state.visited_score_deltas.contains_key(&visited_pos));

        // Test game over condition
        assert!(!state.is_done_patrolling());
        let mut final_state = state.clone();
        for _ in 0..state.configuration.number_of_moves_per_rollout {
            let action = final_state.legal_moves()[0].clone();
            final_state = final_state.advance(&action);
        }
        assert!(final_state.is_done_patrolling());
    }

    #[test]
    fn test_monte_carlo_tree_search() {
        // Create a simple test grid
        let grid = vec![vec![5, 2, 3], vec![4, 5, 6], vec![7, 8, 9]];

        // Create MCTS root node starting at (0, 0)
        let root = Rc::new(RefCell::new(MonteCarloTreeSearchNode::new(
            Drone::new(grid, (0, 0)),
            None,
            None,
        )));
        root.borrow_mut().initialize(Rc::clone(&root));

        // Run MCTS for a short duration
        let allowed_time = Duration::from_millis(100);
        let (path, score) = root.borrow().best_path(allowed_time);

        // Basic assertions
        assert!(!path.is_empty(), "Path should not be empty");
        assert!(score > 0, "Score should be positive");

        // The score should at least include the starting position value (5)
        assert!(
            score >= 5,
            "Score should be at least the starting position value"
        );
    }

    #[test]
    fn test_mcts_performance() {
        // Create a larger test grid
        let grid = vec![
            vec![1, 2, 3, 4, 5],
            vec![2, 3, 4, 5, 6],
            vec![3, 4, 5, 6, 7],
            vec![4, 5, 6, 7, 8],
            vec![5, 6, 7, 8, 9],
        ];

        let root = Rc::new(RefCell::new(MonteCarloTreeSearchNode::new(
            Drone::new(grid, (0, 0)),
            None,
            None,
        )));
        root.borrow_mut().initialize(Rc::clone(&root));

        // Run MCTS for different durations and compare results
        let short_time = Duration::from_millis(50);
        let long_time = Duration::from_millis(200);

        let (_, short_score) = root.borrow().best_path(short_time);
        let (_, long_score) = root.borrow().best_path(long_time);

        // Longer runtime should generally find better or equal solutions
        assert!(
            long_score >= short_score,
            "Longer runtime ({} ms) should find at least as good a solution as shorter runtime ({} ms). Scores: {} vs {}",
            long_time.as_millis(),
            short_time.as_millis(),
            long_score,
            short_score
        );
    }

    #[test]
    fn test_mcts_consistency() {
        let grid = vec![vec![1, 2, 3], vec![4, 5, 6], vec![7, 8, 9]];
        let allowed_time = Duration::from_millis(100);

        // Run multiple times and check that scores are within reasonable bounds
        let mut scores = Vec::new();
        for _ in 0..5 {
            let root = Rc::new(RefCell::new(MonteCarloTreeSearchNode::new(
                Drone::new(grid.clone(), (0, 0)),
                None,
                None,
            )));
            root.borrow_mut().initialize(Rc::clone(&root));

            let (_, score) = root.borrow().best_path(allowed_time);
            scores.push(score);
        }

        // Calculate mean and standard deviation
        let mean = scores.iter().sum::<usize>() as f64 / scores.len() as f64;
        let variance = scores
            .iter()
            .map(|&x| (x as f64 - mean).powi(2))
            .sum::<f64>()
            / scores.len() as f64;
        let std_dev = variance.sqrt();

        println!("Scores: {:?}", scores);
        println!("Mean: {}, Std Dev: {}", mean, std_dev);

        // Check that standard deviation is within reasonable bounds
        assert!(
            std_dev / mean < 0.5,
            "Results are too inconsistent: mean={}, std_dev={}",
            mean,
            std_dev
        );
    }
}
