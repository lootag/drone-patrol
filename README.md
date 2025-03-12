# Monte Carlo Tree Search Drone Pathfinding

This Rust program implements a Monte Carlo Tree Search (MCTS) algorithm to find an optimal path for a drone within a grid. The drone's objective is to maximize its score by moving through the grid cells, where each cell contains a score value.

## Features

-   **MCTS Algorithm:** Implements the core MCTS phases: selection, expansion, simulation (rollout), and backpropagation.
-   **Grid-based Environment:** Reads grid data from a file, representing the drone's environment.
-   **Drone Movement:** Defines drone movement rules and scoring logic.
-   **Command-line Arguments:** Uses `clap` for parsing command-line arguments, including the grid file path and time limit.
-   **Performance Testing:** Includes comprehensive unit tests to ensure correctness and performance.

## Getting Started

### Prerequisites

-   Rust and Cargo: Ensure you have Rust and Cargo installed on your system. You can download them from [rustup.rs](https://rustup.rs/).

### Building and Running

1.  **Build the project:**

    ```bash
    cargo build --release
    ```

2.  **Run the program:**

    ```bash
    cargo run --release -- --grid-file <path_to_grid_file> --time <time_in_milliseconds>
    ```

    -   `<path_to_grid_file>`: Path to the input grid file.
    -   `<time_in_milliseconds>`: Time allowed for the algorithm in milliseconds (default is 120000).

    Example:

    ```bash
    cargo run --release -- --grid-file examples/20.txt --time 60000
    ```

### Input Grid File Format

The input grid file should contain a space-separated matrix of integers, where each integer represents the score value of a grid cell.

Example (`examples/20.txt`):

### Example grid files.

Example grid files are located in the examples directory.

## Code Structure

-   `main.rs`: Contains the main program logic, including command-line argument parsing, grid reading, and MCTS execution.
-   `Drone`: Represents the drone's state, including its position, score, and grid.
-   `MonteCarloTreeSearchNode`: Implements the MCTS node, including selection, expansion, simulation, and backpropagation logic.
-   `Move`: Defines the drone's possible movement directions.
-   `GameConfiguration`: Holds the game's configuration parameters.
-   `read_grid`: Function to read the grid from a file.
-   `tests`: Module containing unit tests for the program.

## Testing

To run the unit tests:

```bash
cargo test
```

## Algorithm Explanation

The program uses the Monte Carlo Tree Search (MCTS) algorithm to find an optimal path for the drone. The MCTS algorithm consists of the following phases:

-   Selection: Traverse the tree from the root node to a leaf node using the tree policy (e.g., UCT).
-   Expansion: If the selected leaf node is not a terminal node, create one or more child nodes.
-   Simulation (Rollout): Simulate a random game from the selected node to a terminal state using the rollout policy.
-   Backpropagation: Update the statistics of the nodes along the path from the selected node to the root node based on the simulation result.
The algorithm repeats these phases until a time limit is reached, and then returns the best path found.

## Possible Improvements

-   **Parallelization:** The algorithm can be parallelized to speed up the search process.
-   **Drone Swarm:** The algorithm can be extended to a swarm of drones, where each drone explores the grid independently and the best path is the one that maximizes the total score of all drones.
-   **Position score restoration:** The position score is currently restored at each time step by one unit. We would use a more sophisticated score restoration mechanism to account for the score distribution, and the priority we want to give exploration vs exploitation.
-   **Modularization:** The program as it stands is not modularized, because it is very small, and creating different modules would be overkill. However, if we were to add consistent amounts of functionality, it would be a good idea to have the following modules:
    -   **IO:** Contains the functions to read the grid from a whatever source (in this example it is a file on disk, but it could be very well be a socket, database system, etc.)
    -   **Game Management:** Contains the logic of the drone's movement, and the score restoration logic. This could get very complex as we add more ways to restore the score, and more ways to have drones interact with each other and the environment.
    -   **MonteCarloTreeSearchNode:** Implements the MCTS node, including selection, expansion, simulation, and backpropagation logic.
