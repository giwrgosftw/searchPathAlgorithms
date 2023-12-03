/* This code defines a simple 2D grid, represents nodes in the grid, and uses the A* algorithm to find the shortest path
 * from the start node to the goal node while avoiding obstacles. The Manhattan distance heuristic is used to estimate
 * the cost from a node to the goal. The path is reconstructed and printed if a valid path exists.
 */

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <memory>

using namespace std;

// A structure to represent a node in the grid. (for more info, see 'Note 8' at the end of the file)
struct NodeStruct {
    int x, y;   // Coordinates of the node (a point in the grid)
    // d = How far the current and the next node are from each other &
    // h = Heuristic (smell) value = estimated (e.g., manhatan distance from the goal to the current path length
    int d, h;
    int f() const { return d + h; } // p(v) = f = function priority for our priority queue
    NodeStruct* parent; // will be used to reconstruct the path
    // Constructor
    NodeStruct(int x, int y, NodeStruct* p = nullptr) : x(x), y(y), d(0), h(0), parent(p) {}
};

// Instead of writing every time 'vector<vector<>>', we can use the 'using' keyword
using Grid = vector<vector<int>>; // a 2D vector to represent a matrix [row][column]
using Path = vector<NodeStruct*>; // A path is denoted by a collection of Node-pointers (more efficient access)

/**
 * Calculates the Manhattan distance between two nodes: |x2-x1| + |y2-y1|
 * This function is typically used as a heuristic in grid-based pathfinding algorithms like A*.
 * (for more info, read 'Note7' at the end of the file)
 * @param node1 The first node, representing a point in the grid.
 * @param node2 The second node, representing another point in the grid.
 * @return The Manhattan distance between node1 and node2, which is an integer value representing the sum
 *         of the absolute differences of their x and y coordinates.
 */
inline int calculate_manhattan_distance(const NodeStruct& node1, const NodeStruct& node2) {
    return abs(node1.x - node2.x) + abs(node1.y - node2.y);
}

/**
 * Checks if a given node (identified by its x and y coordinates) is valid within the provided grid.
 * A node is considered valid if it lies within the bounds of the grid and is not an obstacle.
 * @param x The x-coordinate of the node to check.
 * @param y The y-coordinate of the node to check.
 * @param grid A constant reference to a 2D grid represented as a vector of vectors of integers.
 * @return A boolean value indicating the validity of the node. Returns true if the node is within
 *         the grid bounds and is not an obstacle (i.e., the corresponding value in the grid is 0).
 *         Otherwise, returns false.
 */
bool isValidNode(int x, int y, const Grid& grid) {
    return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0;
}

/**
 * Performs validation checks for the start and goal nodes in the context of the given grid.
 * Returns true if both the start and goal nodes are within the bounds of the grid and not positioned on obstacles.
 * Otherwise, if either of these conditions is not met, the function outputs an error message and returns false.
 * @param grid A constant reference to a 2D grid represented as a vector of vectors of integers (0 and 1)
 * @param start_node A constant reference to the starting node of the pathfinding algorithm (a NodeStruct)
 * @param goal_node A constant reference to the goal node of the pathfinding algorithm (a NodeStruct)
 * @return A boolean value indicating the result of the validation.
 */
bool validateInputNodes(const Grid& grid, const NodeStruct& start_node, const NodeStruct& goal_node) {
    if (!isValidNode(start_node.x, start_node.y, grid) || !isValidNode(goal_node.x, goal_node.y, grid)) {
        cerr << "Error: Start or goal node is invalid (out of bounds or on an obstacle)." << endl;
        return false; // Indicate validation failure
    }
    return true; // Indicate successful validation
}

/**
 * Reconstructs the path from the start node to the goal node in a pathfinding algorithm.
 * The function backtracks from the goal node to the start node using the parent pointers
 * in each NodeStruct, constructing the path as a sequence of nodes.
 * @param goal_node A pointer to the goal node, which is the end point of the path.
 *                  This node should have a series of parent pointers leading back to the start node.
 * @return Path, which is a vector of NodeStruct pointers, representing the path from the start node
 *         to the goal node. The path is reconstructed in reverse order, starting from the goal node
 *         and following each node's parent pointer until the start node is reached.
 */
Path reconstruct_path(NodeStruct* goal_node) {
    Path path;
    while (goal_node != nullptr) {
        path.push_back(goal_node);  // add starting from the goal node
        goal_node = goal_node->parent;  // the (connected) next_node (parent of the current goal_node) becomes the goal_node
    }
    reverse(path.begin(), path.end()); // reverse so that to be from the start to the goal node
    return path;
}

template<typename Compare>  // since we cannot use 'decltype(cmp)' typename here
/**
 * Explores neighboring nodes of the current node in the context of the A* pathfinding algorithm.
 * For each valid neighboring node, it creates a new node, sets its parent to the current node,
 * calculates its cost, and adds it to the priority queue and the list of all nodes.
 * @tparam Compare The type of the comparison function used for ordering the nodes in the priority queue.
 * @param current_node Pointer to the current node being processed in the A* algorithm.
 * @param goal_node The goal node of the A* pathfinding algorithm.
 * @param grid A 2D grid represented as a vector of vectors of integers
 * @param all_nodes_pointers A vector of shared_ptr<NodeStruct> that keeps track of all nodes created during
 *                           the execution of the algorithm for automatic memory management.
 * @param priority_queue A priority queue of NodeStruct pointers, ordered according to the Compare
 *                       function, which is used to select the next node to process in the A* algorithm.
 * @param visited A 2D vector representing the visited status of nodes in the grid.
 */
void exploreNeighbors(NodeStruct* current_node, const NodeStruct& goal_node, const Grid& grid,
                      vector<shared_ptr<NodeStruct>>& all_nodes_pointers,
                      priority_queue<NodeStruct*, vector<NodeStruct*>, Compare>& priority_queue,
                      vector<vector<bool>>& visited) {
    // Define movement directions (up, down, left, right)
    // (for more info, read 'Note1' at the end of the file)
    const vector<int> dx = {0, 0, -1, 1}; // stay, stay, left, right
    const vector<int> dy = {-1, 1, 0, 0}; // down, up, stay, stay
    // Explore the neighboring nodes
    for (int i = 0; i < dx.size(); ++i) {
        // Implement all the moves for possible neighbours
        int next_move_x = current_node->x + dx[i];
        int next_move_y = current_node->y + dy[i];
        // Boundary Check: ensures the new moves fall within the boundaries of the grid
        // Also, check if it has not been visited before.
        if (!isValidNode(next_move_x, next_move_y, grid) || visited[next_move_x][next_move_y]) {
            continue;
        }
        // Get the (next_node) neighbour node after our new movement
        // The current node (pointer) is the parent of the next_node (see 'Note9' for more info)
        auto next_node = make_shared<NodeStruct>(next_move_x, next_move_y, current_node);
        next_node->d = current_node->d + 1; // Assuming uniform cost for simplicity
        next_node->h = calculate_manhattan_distance(*next_node, goal_node); // calculate heuristic value
        all_nodes_pointers.push_back(next_node); // Add the neighbor node to the vector of all nodes.
        priority_queue.push(next_node.get()); // Add the neighbor node to the priority queue for further exploration.
    }
}

/**
 * Implements the A* pathfinding algorithm to find the shortest path in a grid.
 * @param grid A 2D grid represented as a vector of vectors of integers
 * @param start_node The starting position in the grid, represented as a NodeStruct containing x and y coordinates.
 * @param goal_node The goal position in the grid, represented as a NodeStruct containing x and y coordinates.
 * @return Path (vector of NodeStruct pointers) representing the shortest path from start_node to goal_node.
 */
Path a_star_algorithm(Grid grid, const NodeStruct& start_node, const NodeStruct& goal_node) {
    // Validate that start_pointer and goal nodes are within grid bounds and not on obstacles
    if (!validateInputNodes(grid, start_node, goal_node)) {
        return {}; // Return an empty path if validation fails
    }
    // Create a 2D vector(matrix) to store the visited status of nodes - (for more info, read 'Note2' at the end of the file)
    vector<vector<bool>> visited(grid.size(), vector<bool>(grid[0].size(), false));
    // Custom comparator returning true when node lhs->f() is greater than node rhs->f() (min-heap)
    // (for more info, read 'Note3' at the end of the file)
    auto lowerFValueFirst = [](const NodeStruct* lhs, const NodeStruct* rhs) { return lhs->f() > rhs->f(); };
    // A priority queue which prioritizes elements based on the smallest f value (defined by the above lambda function)
    // (for more info, read 'Note4' and 'Note 5' at the end of the file)
    priority_queue<NodeStruct*, vector<NodeStruct*>, decltype(lowerFValueFirst)> priority_queue(lowerFValueFirst);
    // Using shared_ptr for automatic memory management
    vector<shared_ptr<NodeStruct>> all_nodes_pointers;   // a vector of smart pointers (will be one for each valid node)
    auto start_pointer = make_shared<NodeStruct>(start_node.x, start_node.y);   // a smart pointer for the start node
    // Make the start, fulfill by adding the first info based on the start node
    start_pointer->h = calculate_manhattan_distance(start_node, goal_node);
    all_nodes_pointers.push_back(start_pointer);
    priority_queue.push(start_pointer.get());
    // Continue the loop as long as there are nodes left to explore
    while (!priority_queue.empty()) {
        // Retrieve and remove the node with the lowest 'f' value from the priority queue.
        NodeStruct* current_node = priority_queue.top(); // retrieve
        priority_queue.pop(); // remove
        // If this node has been visited before, skip processing it.
        if (visited[current_node->x][current_node->y]) {
            continue;
        }
        // Check if the current node is the goal. If so, reconstruct and return the path.
        if (current_node->x == goal_node.x && current_node->y == goal_node.y) {
            return reconstruct_path(current_node);
        }
        // Mark the current node as visited.
        visited[current_node->x][current_node->y] = true;
        // Explore the neighboring nodes
        exploreNeighbors(current_node, goal_node, grid, all_nodes_pointers, priority_queue, visited);
    }

    return {}; // Return empty path if goal not reached
}

/**
 * Prints the shortest path found by the A* algorithm.
 * @param path A constant reference to a Path, which is a vector of NodeStruct pointers.
 * This vector represents the path found from the start node to the goal node.
 * Each NodeStruct in the vector contains the x and y coordinates of a step in the path.
 * The function iterates through the provided path and prints the coordinates of each node in the path in order.
 * If the path is empty, indicating that no path was found, it prints a message stating that no path was found.
 * This function is useful for visually verifying the shortest path determined by the A* algorithm.
 */
void printPath(const Path& path){
    if (!path.empty()) {
        cout << "Shortest path:" << endl;
        for (const NodeStruct* node : path) {
            cout << "(" << node->x << ", " << node->y << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found." << endl;
    }
}

int main() {
    // Define the grid (0 represents a clear cell, 1 represents an obstacle)
    Grid grid = {
            {0, 0, 1},
            {1, 0, 1},
            {0, 0, 0}
    };
    // Initialise start and goal node
    NodeStruct start_node(0, 0); // Start at the top-left corner (1st row, 1st column)
    NodeStruct goal_node(2, 2);  // Goal at the bottom-right corner (3rd row, 3rd column)
    // Call the A* star algorithm
    Path path = a_star_algorithm(grid, start_node, goal_node);
    // Print the path
    printPath(path); // expect (0, 0) (0, 1) (1, 1) (2, 1) (2, 2)

    return 0;
}



/* Note 1 - Define movement directions (up, down, left, right):
 * dx and dy are set as arrays because we do not want to modify them later
 * when e.g., goes up, it cannot turn left or right, thus, when dy=1, then dx=0
 * Here's a recap of the possible movements based on the values in dx and dy:
    (x, y)
    (0, -1): Up (decrease in the y-coordinate).
    (0, 1): Down (increase in the y-coordinate).
    (-1, 0): Left (decrease in the x-coordinate).
    (1, 0): Right (increase in the x-coordinate).
 */


/* Note 2 - It is a grid that mirrors the dimensions of the 'input grid':
 * grid.size(): assuming that the grid is rectangular, the number of rows = columns, thus, it returns the number of rows/columns in the grid
 * grid[0].size(): returns the number of columns = number of elements of the first row (noted here for scenarios that it is not rectangular)
 * vector<bool>(grid.size(), false): a vector with a size equal to the number of rows/columns in the grid, and all elements are initialized to false (not yet visited)
 * vector<vector<bool>> visited(grid.size(), vector<bool>(grid.size(), false)): a vector with a size equal to the number of rows/columns in the grid,
                                                                                and each element of the 2D corresponds to an individual cell in the grid
                                                                                with initialized value false
 * For example, if grid.size() is 3:
    *  vector<vector<bool>> visited(grid.size(), ...                                = [ [], [], [] ]  // A 2D vector with 3 empty rows
    *  (..., vector<bool>(grid.size(), ...)                                         = [ , , ]         // An inner vector with 3 elements
    *  vector<vector<bool>> visited(grid.size(), vector<bool>(grid.size(),          = [ [ , , ],
                                                                                        [ , , ],
                                                                                        [ , , ] ]     // A 2D vector with 3 rows and 3 columns (the [ , , ] goes inside each []
    * vector<vector<bool>> visited(grid.size(), vector<bool>(grid.size(), false));  = [ [ false, false, false ],
                                                                                        [ false, false, false ],
                                                                                        [ false, false, false ] ]  // A 2D vector with 3 rows and 3 columns, with value false
 * In human words, think that you have a 3x3 matrix full of 0s, if the start_node is the top left corner 0,
   it becomes 1 and continue exploring based on movements to make some of these (or all of them) 1 as well
*/


/* Note 3 - Lambda Expression compare 'f value' of the two nodes:
 * [](const NodeStruct* lhs, const NodeStruct* rhs) { return lhs->f() > rhs->f(); } is a lambda expression.
 * It's an anonymous function that takes two arguments (lhs and rhs, both pointers to NodeStruct) and returns a boolean value.
 *
 * Comparator Functionality:
 * The purpose of this lambda expression is to define how two elements in the priority queue should be compared.
 * In this case, it's comparing the f values of two NodeStruct objects.
 * The f value in A* algorithm context is typically the sum of g (the cost from the start node to the current node) AND
 * h (the heuristic estimate of the cost from the current node to the goal).
 *
 * Comparison Logic:
 * lhs->f() > rhs->f() means that the function will return true if the f value of the left-hand side (lhs) node is greater
 * than that of the right-hand side (rhs) node.
 *
 * Usage in Priority Queue:
 * The lambda expression is used as a comparator in a priority queue.
 * In C++, the default behavior of the priority queue is to put the largest element at the front.
 * However, in the context of the A* algorithm, you want the node with the smallest f value (lowest cost estimate) to be processed first.
 * So, this custom comparator inverts the usual order by returning true when lhs->f() is greater than rhs->f().
 */


/* Note 4 - For each move add the neighbours to and choose the one with lower cost on top:
1. Exploration: The algorithm starts at the initial node and explores its neighboring nodes (potential goalNode moves).
2. Priority Queue: store the neighboring nodes (of potential goalNode moves) and ordered based on their estimated total cost (lower at the front of the queue).
3. Choosing the Best Move: The algorithm selects the node at the top and removes it from the queue (in any case, priority queue still contains the remaining unexplored nodes)
4. Update and Repeat: After making a move, the algorithm updates its position and repeats the process.
5. Repeat Until Goal: This process continues until the algorithm reaches the goal node. Once the goal is reached, the algorithm may backtrack to reconstruct the optimal path.
The key is that A* doesn't blindly explore all possible moves simultaneously but rather evaluates and prioritizes nodes one at a time based on their estimated total cost.
This prioritization helps it efficiently find the shortest path while avoiding unnecessary exploration.
 */


/* Note 5 - Priority queue:
 * In summary, this line of code declares a priority_queue named 'priority_queue' that stores pointers to NodeStruct.
 * It orders these pointers according to the custom comparator cmp, which prioritizes elements based on the smallest f value
 * (the sum of cost from the start and heuristic estimate to the goal). This behavior is essential for the A* algorithm to work correctly,
 * as it needs to process nodes in the order of their estimated total cost to find the most efficient path.
 *
 * Construction of queue:
 * The priority_queue is constructed with cmp as its comparison object.
 * This means that whenever elements are added to or ordered within the queue, the cmp function will be called to compare them.
 * Thus, instead of using the default max-heap (where the largest element is given priority), it uses the custom order defined by cmp,
 * which, based on your lambda function, creates a min-heap (where the smallest element according to f() value is given priority).
 *
 * decltype keyword:
 * It is a common practise when use lambda functions to use the 'auto' keyword to define the type of the variable that
 * stores the output of that lambda function. However, inside we cannot use 'auto' inside a signature. Thus, 'decltype'
 * is recommended here since is used for type deduction (the compiler automatically determines the type of a variable or an expression),
 * allowing you to determine the type of a variable or an expression without having to explicitly specify it.
 */


/*
 * Note 6 - Exploring neighbours:
 * This code is a typical implementation of the A* algorithm's main loop.
 * The algorithm works by exploring nodes, starting from the start node, and expanding to neighboring nodes until
 * it reaches the goal. It keeps track of visited nodes and uses a priority queue to determine which node to
 * explore next based on the combined cost (f value) of the path to that node and the estimated cost from that
 * node to the goal (h value, estimated using the Manhattan distance in this case). The use of shared_ptr for nodes
 * ensures proper memory management.
 */


/*
 * Note 7 - Manhattan distance:
 * Inline functions are often used for small, frequently called functions, such as simple mathematical operations or
 * accessors in classes, where the function call overhead could impact performance (tell it to run in compile-time).
 *
 * The Manhattan distance is a heuristic commonly used in grid-based pathfinding algorithms where movement
 * is restricted to orthogonal directions (up, down, left, right). It represents the minimum number of steps
 * needed to reach from one point to another if diagonal movement is not allowed.
 */


/* Note 8 - NodeStruct:
 *     A. In the sequence A -> B -> C -> D, here's how the parent-child relationships work:
          - B is a child of A (A is the parent of B).
          - C is a child of B (B is the parent of C).
          - D is a child of C (C is the parent of D).
          In this sequence, each node becomes the child of the previous node, and the previous node becomes
          the parent of the current node. This is how the algorithm keeps track of the path from the start node (A)
          to the current node (D).
          So, when you later want to reconstruct the path from A to D, you follow the parent pointers
          in reverse order, starting from D's parent (C), then C's parent (B), and finally B's parent (A).
          This process allows you to reconstruct the path from A to D.
       B. Since we want to access another NodeStruct (parent) and modify straightforward,
          the best approach is using pointers
 *
 */


/* Note 9 - Smart pointers ('all_nodes_pointers'):
 * Summary:
 *  since the 'shared_ptrs' are stored in the 'all_nodes_pointers', when the algorithm tries to reconstruct the path,
 *  it has to reach the parent pointer for the next node. Then, because the according 'shared_ptr' is stored,
 *  automatically C++ knows that the parent exist through that smart pointer.
 *  On the other hand, the row pointer in the priority_queue is used just for safe access for that moment.
 *  So:
 *    A. 'all_nodes_pointers' vector stores smart pointers to keep nodes alive throughout the algorithm,
 *        ensuring their validity for path reconstruction.
 *    B.  Raw pointers in the priority queue enable efficient exploration, as they provide temporary, direct access to nodes.
 *  This setup divides responsibilities:
 *    A. smart pointers maintain node lifetimes for safe backtracking,
 *    B. while raw pointers allow for quick exploration during the algorithm's execution.
 *
 * Detailed:
 *  'all_nodes_pointers' is not directly used for the logical flow or computations of the A* algorithm beyond storing the shared_ptrs.
 *  Memory Management:
 *      However, without storing these shared_ptrs, the dynamically created nodes would be destroyed immediately
 *      after 'exploreNeighbors' go out of scope, as no shared_ptr would be owning them.
 *      This is because the priority queue in your A* implementation stores raw pointers to these nodes.
 *      These raw pointers do not own the objects they point to and hence do not manage their lifetimes.
 *      In other words, when you push the raw pointer (obtained using .get() on the shared_ptr) into the priority queue,
 *      the queue doesn't manage the memory of the node. It's only holding a pointer to it. The actual memory management is
 *      still the responsibility of the shared_ptr.
 *
 * Role of all_nodes_pointers:
 *   all_nodes_pointers is a vector that stores shared_ptr<NodeStruct> objects. Each shared_ptr manages the memory for
 *   a NodeStruct instance, ensuring that the nodes are not destroyed (deallocated) prematurely.
 *
 * Interaction with all_nodes_pointers:
 *  Although reconstruct_path does not directly interact with all_nodes_pointers, the vector's role is crucial in ensuring
 *  that the nodes being traced back through are still valid.
 *
 * Here's the interaction breakdown:
 * Node Creation and Storage:
 * When new nodes are created in exploreNeighbors, they are wrapped in shared_ptrs for memory management and added to all_nodes_pointers.
 *
 * Validity of Nodes:
 * Because each node is managed by a shared_ptr in all_nodes_pointers, they remain valid throughout the algorithm's execution.
 * This means that as long as the algorithm is running, all created nodes are guaranteed to exist and can be safely accessed.
 *
 * Path Tracing:
 * When reconstructing the path, the algorithm starts from the goal node and follows the parent pointers backward to the start node.
 * Each of these nodes is valid because its memory is managed by one of the shared_ptrs in all_nodes_pointers.
 * Without all_nodes_pointers, these nodes might have been deallocated, making the parent pointers dangling and leading to undefined behavior.
 *
 * Indirect Dependence:
 * Although reconstruct_path doesn't directly reference all_nodes_pointers, the successful execution of this function
 * is indirectly dependent on all_nodes_pointers. The shared_ptrs within this vector ensure that the nodes in the path
 * (linked through parent pointers) have not been deallocated.
 */
