#include <iostream>
#include <vector>
#include <set>
#include <string>

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

bool validBoundaries(vector<string> &grid, int row, int col){
    // upper bound || down bound || left bound || right bound  || (after recursion) current cell is not an obstacle || or visited
    if(row < 0 || row >= grid.size() || col < 0 || col >= grid.size() || grid[row][col] == '0' || grid[row][col] == '2'){
        return false;
    }
    return true;
}

/**
 * This method is used to identify a region, starting from the given cell and marks all the connected cells with '2'
 * @param grid : * We need to check the boundaries and modify it so that to mark the visited cells
 *               * Represents the grid (either grid1 or grid2) in which regions are being identified.
 * @param currentCellPair:
 *               * The coordinates (row and column) of the cell == 1
 *               * They are essential for navigating the grid, allowing the function to explore adjacent cells and
 *                 identify the full extent of a region.
 * @param region: * This parameter is used to keep track of all the visited cells (that form the current region)
 * Note: Since we do not need a backups, we passing by reference avoids copying the entire grid
 *       However, we are not using reference in row,col because they are different every time
 */
void dfs(vector<string> &grid, int row, int col, set<pair<int, int>> &region){
    // Step 1: Check if the given cell is out of the boundaries
    bool isValid = validBoundaries(grid, row, col);
    if(!isValid){ return; } // "break" the function
    // Step 2: Mark the current cell as visited
    markVisited(grid, row, col, region);
    // Step 3: Explore all the adjacent/neighbours of the current cell
    dfs(grid, row + 1, col, region);  // increase row, thus, down
    dfs(grid, row - 1, col, region);  // decrease row, thus. up
    dfs(grid, row, col + 1, region);   // increase col, thus, right
    dfs(grid, row, col - 1, region);   // decrease col, thus, left
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
