/* This code defines a simple 2D grid, represents nodes in the grid, and uses the A* algorithm to find the shortest path
 * from the start node to the goal node while avoiding obstacles. The Manhattan distance heuristic is used to estimate
 * the cost from a node to the goal. The path is reconstructed and printed if a valid path exists.
 */

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

using namespace std;

// A structure to represent a node in the grid.
/* Since we are not worried if our NodeStruct's data are public
   it is a good practise to create a struct inside the class to represent the NodeStruct.
 */
struct NodeStruct{
    int x, y;   // Coordinates of the node (a point in the grid)
    int d;      // Distance value
    int h;      // Heuristic (smell) value
    int p;      // Total cost p = d + h
    /* A. In the sequence A -> B -> C -> D, here's how the parent-child relationships work:
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
     */
    NodeStruct* parent;
    // Constructor which requires x, y etc. when create
    NodeStruct(int x, int y) : x(x), y(y), d(0), h(0), p(d + h), parent(nullptr) {}
    // Comparison operator based on their total cost (f)
    /* Since we are going to use priority queue,
     * we need a comparison operator (can be auto generated) */
    bool operator<(const NodeStruct& neighbour_node) const{ return h > neighbour_node.h;}
};

// a 2D vector to represent a matrix [row][column] or [x][y]
/*
 * A. Instead of writing every time 'vector<vector<int>>',
 *    we can use the key 'using' so that we can write, e.g., Grid myGrid.
*/
using Grid = vector<vector<int>>;
// A path is denoted by a collection of Node-pointers (more efficient access)
using Path = vector<NodeStruct*>;



// Compare nodes based on their total cost (p).
/* The nodes are not passed as pointers in the method parameters because there is no need
 * to modify them, instead we just compare them (pass-by-reference preferred).
 */
bool compare_nodes_total_cost(NodeStruct& node1, NodeStruct& node2){
    return node1.p > node2.p;
}

// Validations - use references since we just validate
Path validations(Grid& grid, NodeStruct& current_node, NodeStruct& next_node){
    /* Since we are working on a 2D grid,
     * 0: Represents a clear cell (VALID)
     * 1: Represents an obstacle (NOT VALID)
     */
    if(grid[current_node.x][current_node.y] == 1 || grid[next_node.x][next_node.y] == 1){
        cout << "Invalid node" << endl;
        return {};  // return an empty vector of pointers - since the A* has to return a vector of pointers
    }
}

// Manhattan distance between two nodes: |x2-x1| + |y2-y1|
/* Inline functions are often used for small, frequently called functions, such as simple mathematical operations or
 * accessors in classes, where the function call overhead could impact performance (tell it to run in compile-time).
 */
inline int calculate_manhattan_distance(const NodeStruct& node1, const NodeStruct& node2){
    abs(node1.x - node2.x) + abs(node1.y - node2.y);
}

// Create the priority queue including the given start node
priority_queue<NodeStruct> create_priority_queue(NodeStruct& start_node, NodeStruct& goal_node){
    // For each move add the neighbours to and choose the one with lower cost on top
    /*
    1. Exploration: The algorithm starts at the initial node and explores its neighboring nodes (potential goalNode moves).
    2. Priority Queue: store the neighboring nodes (of potential goalNode moves) and ordered based on their estimated total cost (lower at the front of the queue).
    3. Choosing the Best Move: The algorithm selects the node at the top and removes it from the queue (in any case, priority queue still contains the remaining unexplored nodes)
    4. Update and Repeat: After making a move, the algorithm updates its position and repeats the process.
    5. Repeat Until Goal: This process continues until the algorithm reaches the goal node. Once the goal is reached, the algorithm may backtrack to reconstruct the optimal path.
    The key is that A* doesn't blindly explore all possible moves simultaneously but rather evaluates and prioritizes nodes one at a time based on their estimated total cost.
    This prioritization helps it efficiently find the shortest path while avoiding unnecessary exploration.
     */
    priority_queue<NodeStruct> exploring_node_queue;
    // 2D. Calculate the start node (first thing you do)
    start_node.d = 0; // by default
    start_node.h = calculate_manhattan_distance(start_node, goal_node);
    exploring_node_queue.push(start_node);
    return exploring_node_queue;
}

// If the current node is the goal, reconstruct and return the path
Path check_if_goal(NodeStruct& current_node, NodeStruct& start_node, NodeStruct& goal_node){
    if(current_node.x == goal_node.x && current_node.y == goal_node.y){
        // Initialised an empty vector (the path that we will return)
        /* The path is reconstructed by going backwards and using the parent nodes as points
         * So, it is a vector of parent nodes-pointers actually
         * If we choose to use pass-by-object instead of pointers, during path reconstruction,
         * the algorithm would need to create new NodeStruct objects for each node in the path,
         * leading to unnecessary memory consumption and copying overhead.
         */
        Path path;
        // Continue while-loop until it reaches the start node (which does not have a parent)
        while(current_node.parent != nullptr){
            /* A. In C++, the push_back() function is used to add an element to the END of a vector.
             *    There is no standard push() function for vectors as the queue.
             * B. The '&' inside the symbol, when used before a variable name,
             *    (e.g., &variable) it takes the address of the variable,
             *    effectively creating a pointer to that variable
             */
            path.push_back(&current_node); // add the current node (moving backwards necessarily)
            current_node = *current_node.parent; // add its parent node (in the next iteration)
        }
        // Since we removed the start_node at the very beginning from the queue, needs to be added here
        /* We need to do each time since the path is empty every time call the function
         */
        path.push_back(&start_node);
        // Since we are going backwards, we need to reverse
        reverse(path.begin(), path.end());
        return path;
    }
}

// Here is what happens when we are doing the next step
void move_in_the_grid(const int dx[], const int dy[], NodeStruct& current_node, NodeStruct& goal_node, Grid grid,
                      vector<vector<bool>> visited, priority_queue<NodeStruct> priority_queue){
    /* size dx = size dy => in our scenario is 4
 */
    for (int i = 0; i < sizeof(dx); i++) {
        /* Implement all the moves for possible neighbours
         * These calculations check neighboring cells in the grid.
         */
        int next_move_x = current_node.x + dx[i];
        int next_move_y = current_node.y + dy[i];

        // 4A. Boundary Check: ensures the new moves fall within the boundaries of the grid
        if(next_move_x >= 0 && next_move_x < grid.size() && next_move_y >= 0 && next_move_y < grid.size()){
            // 4B. Check if A.the neighbor is not an obstacle and B.has not been visited
            if(grid[next_move_x][next_move_y] != 0 && visited[next_move_x][next_move_y] == false){
                NodeStruct next_node(next_move_x, next_move_y); // get the (neighbour) node after our new movement
                // 4BI. Give it values
                next_node.parent = &current_node; // The current node (pointer) is the parent of the next_node
                next_node.d = current_node.d + 1; // Assuming uniform cost for simplicity
                next_node.h = calculate_manhattan_distance(next_node, goal_node);
//                 next_node.p = next_node.d + next_node.h // Should be calculated automatic
                // 4BII. Add the neighbour to the queue (or further evaluation)
                priority_queue.push(next_node);
            }
        }

    }
}

// Return a vector of pointers, which is the shortest path
Path a_star_algorithm(Grid& grid, NodeStruct& start_node, NodeStruct& goal_node) {
    // 1. Validate each pair of nodes (start_node, goal_node)
    validations(grid, start_node, goal_node);

    // 2. Initializations
    /* Define movement directions (up, down, left, right)
     * dx and dy are arrays because we do not want to modify them
     * Note: when e.g., goes up, it cannot turn left or right, thus, when dy=1, then dx=0
     * Here's a recap of the possible movements based on the values in dx and dy:
        (x, y)
        (0, -1): Up (decrease in the y-coordinate).
        (0, 1): Down (increase in the y-coordinate).
        (-1, 0): Left (decrease in the x-coordinate).
        (1, 0): Right (increase in the x-coordinate).
     */
    const int dx[] = {0, 0, -1, 1}; // stay, stay, left, right
    const int dy[] = {-1, 1, 0, 0}; // down, up, stay, stay
    // 2B. Create a 2D vector(matrix) to store the visited status of nodes - It is a grid that mirrors the dimensions of the input grid (grid).
    /* grid.size(): assuming that the grid is rectangular, the number of rows = columns, thus, it returns the number of rows/columns in the grid
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
    vector<vector<bool>> visited(grid.size(), vector<bool>(grid.size(), false));
    // 2C. Create a priority queue including the given start node
    priority_queue<NodeStruct> priority_queue = create_priority_queue(start_node, goal_node);

    // 3. Path build during exploration (the exploration will be obviously extended while reaching the 4th step and looping
    /* Work until there are no neighbours left
     * Eventually, all of them will be removed,
     * even the last one that not selected
     */
    while (!priority_queue.empty()) {
        //3A. Initialisations
        NodeStruct current_node = priority_queue.top(); // Get the node with lowest p
        priority_queue.pop();  // remove it since we stored it above
        visited[current_node.x][current_node.y] = true; // Mark current node as visited (why need this???)
        // 3B. Check if we reached the goal node
        vector<NodeStruct *> path = check_if_goal(current_node, start_node, goal_node);
        // 3B. Explore the neighbors of the current node (moving in the grid)
        move_in_the_grid(dx, dy, current_node, goal_node, grid, visited, priority_queue);

        // If the open list is empty and the goal was not reached, return an empty path
        cout << "No path found." << endl;
        return {};
    }
}

int main() {
    // Define the grid (0 represents a clear cell, 1 represents an obstacle)
    Grid grid = {
            {0, 0, 1, 0, 0},
            {1, 0, 1, 0, 1},
            {0, 0, 0, 0, 0},
            {0, 1, 1, 1, 0},
            {0, 0, 0, 0, 0}
    };
    // Initialise start and goal node
    NodeStruct start_node(0, 0); // by default the origin
    NodeStruct goal_node(4, 4); // 4th row and 4th column
    // Call the A* star algorithm
    Path path = a_star_algorithm(grid, start_node, goal_node);
    // Print the path
    if(!path.empty()) {
        cout << "Shortest path:" << endl;
        for (const NodeStruct *node: path) {
            cout << "(" << node->x << ", " << node->y << ") ";
        }
        cout << endl;
    }
    return 0;
}

