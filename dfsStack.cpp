#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <stack>

using namespace std;

// Here we get the final collection of each region and compare to find a match
int increaseCount(vector<set<pair<int, int>>> &regions1, vector<set<pair<int, int>>> &regions2){
    int count = 0;
    for(auto region1 :regions1){
        for(auto region2 : regions2){
            if(region1 == region2){
                ++count;
                break; // Once a match is found, we move to the next region in grid1
            }
        }
    }
    return count;
}

void markVisited(vector<string> &grid, int row, int col, set<pair<int, int>> &region){
    grid[row][col] = '2'; // Mark the given cell as visited
    region.insert(make_pair(row, col)); // Add the given cell to the region that we are currently working on
}

/**
 * This DFS function uses an iterative approach with a stack to identify regions in the grid.
 * Instead of using recursion to explore the neighbours of the (neighbour) cell, we use stack
 * where we can push, pop and explore each neighbour through its top.
 * This method is beneficial in environments where recursion depth is limited
 * or where a large number of recursive calls may lead to a stack overflow. The logic of exploration remains
 * the same, but the control over the process is more explicit and handled manually using a stack.
 *
 * @param grid Reference to the grid being explored. It's modified in-place to mark visited cells.
 * @param startRow The starting row index for DFS exploration.
 * @param startCol The starting column index for DFS exploration.
 * @param region Reference to the set where the coordinates of the region's cells are stored.
 */
void dfs(vector<string> &grid, int startRow, int startCol, set<pair<int, int>> &region) {
    stack<pair<int, int>> stack; // Initialize a stack to manage the cells to be explored in a DFS manner
    stack.push(make_pair(startRow, startCol)); // Add initial cell to the Stack
    // Continue exploring as long as there are cells in the stack
    while (!stack.empty()) {
        // Step 1: get the cell to start the exploration (top of the stack)
        // and remove it from the stack (since we will eventually need the next cell of it)
        auto [row, col] = stack.top();
        stack.pop();
        // Step 2: Check boundaries and whether the cell is already visited
        //         If so, skip the current iteration and continue with the next
        if (row < 0 || row >= grid.size() || col < 0 || col >= grid[0].size() || grid[row][col] != '1') {
            continue;
        }
        // Step 3: Mark the current cell as visited and add to the current region
        markVisited(grid, row, col, region);
        // Step 4: Push adjacent cells to the stack
        stack.push({row + 1, col}); // Down
        stack.push({row - 1, col}); // Up
        stack.push({row, col + 1}); // Right
        stack.push({row, col - 1}); // Left
    }
}


/**
* This method creates a region based on the first cell=1 it finds and add it to a collection
* Since we iterate the whole grid, it might add more than one region to the collection
* Note: we initialize a region inside the if-statement of the nested loop
*       and not inside the DFS() because the region should not 'restart'
*       during the whole DFS process, it has to get empty again when we
*       check all explore all the adjacent/neighbours of the cell == 1
*/
vector<set<pair<int, int>>> createRegions(vector<string> &grid){
    // Create a collection of regions of each grid, so that
    // each element/region of both collections can be compared at the end
    vector<set<pair<int, int>>> regions;
    // Start the exploration
    for(int row{0}; row<grid.size(); ++row){
        for(int col{0}; col<grid[0].size(); ++col){ // grid[0].size() is the 1st row size (or the number of columns)
            if(grid[row][col] == '1'){
                set<pair<int, int>> region; // Create the region start form this cell
                dfs(grid, row, col, region); // Implement dfs to create (fills the region)
                regions.push_back(region);
            }
        }
    }
    return regions;
}

/**
 * The main function "countMatches" finds the regions in both grids
 * and then compares these regions to count the number of matches.
 * This function counts the total number of regions in grid2 that exactly match with any region in grid1.
 * @param grid1
 * @param grid2
 * @return the final count of matching regions
 */
int countMatches(vector<string> grid1, vector<string> grid2) {
    // For each grid, explore using DFS and return the regions of 1s
    auto regions1 = createRegions(grid1); // For Grid1
    auto regions2 = createRegions(grid2); // For Grid2
    // Counting matches by comparing the two collections
    int countOfMatches = increaseCount(regions1, regions2);

    return countOfMatches;
}

/*
 * This solution assumes that the regions in each grid are distinct
 * and that a region in grid1 can match with only one region in grid2.
 * If there are overlapping or identical regions within a single grid,
 * the algorithm would need to be adjusted accordingly.
 */
int main() {
    // Sample input from image
    vector<string> grid1 = {"0100", "1001", "0011", "0011"};
    vector<string> grid2 = {"0101", "1001", "0011", "0011"};
    // Calculate and print the result
    cout << "Number of matching regions: " << countMatches(grid1, grid2) << endl;

    return 0;
}
