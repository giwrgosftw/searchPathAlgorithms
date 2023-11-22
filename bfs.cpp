#include <iostream>
#include <vector>
#include <set>
#include <deque>

using namespace std;

bool hasNoObstacles(vector<vector<int>>& maze, pair<int, int> currentCell, pair<int, int> nextCell){
    // Coordinates initialization
    int iCurrent = currentCell.first;
    int jCurrent = currentCell.second;
    int iNext = nextCell.first;
    int jNext = nextCell.second;
    // Check for HORIZONTAL obstacles if the move is in the same row
    if(iCurrent == iNext) {  // row index (i) remains the same, and only the column index changes
        // In case the robot can jump, iterates over all columns between 'current_j' and 'next_j' for obstacles
        if (iCurrent >= 0 && iCurrent < maze.size()){   //  verify that both row and jCurrent are within the bounds of the maze
            for (int col = min(jCurrent, jNext); col <= max(jCurrent, jNext) && col < maze[iCurrent].size(); ++col) {
                if (maze[iCurrent][col] == 1) {
                    return false;
                }
            }
        }
    }
        // Else check for VERTICAL obstacles if the move is in the same row
    else if(jCurrent == jNext) {  // column index (j) stays the same, and the row index changes
        // In case the robot can jump, iterates over all rows between 'current_i' and 'next_i' for obstacles
        for(int row = min(iCurrent, iNext); row<= max(iCurrent, iNext); ++row){
            if (row >= 0 && row < maze.size() && jCurrent >= 0 && jCurrent < maze[row].size()) { //  verify that both row and jCurrent are within the bounds of the maze
                if (maze[row][jCurrent] == 1) {
                    return false;
                }
            }
        }
    }
    return true;
}

void bfsSearch(vector<vector<int>>& maze, int& n, int& m, int& i, int& j, int& k, set<pair<int, int>>& visitedCellsSet,
               deque<pair<pair<int, int>, int>>& toVisitCellsDeque, int& moves){
    // For every robot step 'x' in range 'k'
    for(size_t x{1}; x<=k; ++x){    // Set appropriate LOOP-RANGE so NOT move beyond 'k'
        // Check all four directions [(i + x, j), (i, j + x), (i - x, j), (i, j - x)]
        // A.Check bounds | B.Check obstacles (even between) | C.Check if the next_cell has already been visited
        if((i+x < n) && hasNoObstacles(maze, {i, j}, {i+x, j}) && (visitedCellsSet.find({i+x, j}) == visitedCellsSet.end())){
            // Move/Step is done, we just visited the neighbour/next_cell, thus, assign it as visited
            visitedCellsSet.insert({i+x, j});
            // Move/Step is done, increase the number of moves,  && the 'just visited' cell will be enqueued
            toVisitCellsDeque.push_back({{i+x, j}, moves+1});
        }
        if((j+x < m)  && hasNoObstacles(maze, {i, j}, {i, j+x})  && (visitedCellsSet.find({i, j+x}) == visitedCellsSet.end())){
            visitedCellsSet.insert({i, j+x});
            toVisitCellsDeque.push_back({{i, j+x}, moves+1});
        }
        if((i-x >= 0)  && hasNoObstacles(maze, {i, j}, {i-x, j})  && (visitedCellsSet.find({i-x, j}) == visitedCellsSet.end())){
            visitedCellsSet.insert({i-x, j});
            toVisitCellsDeque.push_back({{i-x, j}, moves+1});
        }
        if((j-x >= 0)  && hasNoObstacles(maze, {i, j}, {i, j-x})  && (visitedCellsSet.find({i, j-x}) == visitedCellsSet.end())){
            visitedCellsSet.insert({i, j-x});
            toVisitCellsDeque.push_back({{i, j-x}, moves+1});
        }
    }
}

/* STEP 4: CHECKING REACHABILITY ('START' OR 'END' CELL IS AN OBSTACLE) */
int checkReachability(vector<vector<int>>& maze, pair<int, int>& start, pair<int, int>& end){
    if(maze[start.first][start.second] == 1 || maze[end.first][end.second] == 1){
        return -1;
    }
    return 0;
}

// This function uses Breadth-First Search (BFS) to explore the maze.
// It keeps track of the cells visited to avoid cycles and uses a deque to process cells in the order they are discovered
// For each cell, it tries to move up to k cells in each direction, checking for obstacles and boundaries.
// If it reaches the target cell (n - 1, m - 1), it returns the number of moves taken to get there.
// If the target cell is unreachable, it returns -1.
/* STEP 1: CHOOSE RIGHT PATHFINDING ALGORITHM (BFS) */
int bfs(vector<vector<int>>& maze, int k){
    // Initialization
    int n = maze.size();    // The total number of rows in the grid
    int m = maze[0].size();  // The total number of columns in the grid
    pair<int, int> start = {0, 0};     // Top-left cell is always (0, 0)
    pair<int, int> end = {n-1, m-1};   // Bottom-right cell is always (n-1, m-1)
    /* STEP 4: CHECKING REACHABILITY ('START' OR 'END' CELL IS AN OBSTACLE) */
    checkReachability(maze, start, end);
    /* STEP 2: CHOOSING RIGHT DATA STRUCTURE (DEQUE & SET) */
    set<pair<int, int>> visitedCellsSet;    // Keep track the visited cells
    deque<pair<pair<int, int>, int>> toVisitCellsDeque;   // ((i,j), number of moves taken to reach it)
    toVisitCellsDeque.push_back({{0,0}, 0}); // start from initial cell [(0,0),0)]
    /* STEP 3: IMPLEMENT BFS */
    while(!toVisitCellsDeque.empty()){  // Run while there are cells to process
        // deque the currentCell | (i, j) = currentCell | Front of the queue = starting cell info = ((0,0), 0)
        auto frontElement = toVisitCellsDeque.front(); // Get the front element of the deque
        toVisitCellsDeque.pop_front(); // // Remove the front element of the deque (since we stored it)
        // Decompose the frontElement into (i, j)
        pair<int, int> currentCell = frontElement.first;
        int i = currentCell.first;
        int j = currentCell.second;
        // Decompose the frontElement to get the moves
        int moves = frontElement.second;
        /* STEP 5: ENDING CONDITION 1 */
        if((i == n-1 && j == m-1)){
            return moves;
        }
        /////////////
        bfsSearch(maze, n, m, i, j, k, visitedCellsSet, toVisitCellsDeque, moves);
    }
    /* STEP 5: ENDING CONDITION 2 */
    return -1;  // Target is unreachable (deque is emptied without reaching the destination)
}

int getMinimumMoves(vector<vector<int>>& maze, int& k){
    return bfs(maze, k);
}

int main() {
    // Define the maze
    vector<vector<int>> maze = {
            {0, 0},  // First row
            {1, 0}   // Second row
    };

    // Define the jump parameter k
    int k = 2;

    // Call the function and print the result
    cout << "Minimum number of moves: " << getMinimumMoves(maze, k) << endl;

    return 0;
}
