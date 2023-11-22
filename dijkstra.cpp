#include <iostream>
#include <vector>
#include <limits>

/*
 * Note: unordered map/set are preferred when sorting is not necessary
 *       because you do not need to rearrange elements when inserting or deleting
 */
#include <unordered_map>
#include <unordered_set>

using namespace std;

/*
 * A. Instead of writing every time 'unordered_map<int, vector<pair<int,int>>>',
 *    we can use the key 'using' so that we can write, e.g., Graph myGraph.
 * B. 'pair' keyword works like a tuple but can hold only two variables,
 *    (but it works with brackets). If you want to hold more variables,
 *    you can use the 'tuple' data structure.
 */
using Graph = unordered_map<char, vector<pair<char, int>>>;

unordered_map<char, int> dijkstra(const Graph& graph, const char& start) {
    // 1. Create a dictionary to keep track of the shortest distance from the start node to each node
    unordered_map<char, int> distances;
    /* 2. A. Set the distances of all nodes from the start node initially to infinity (see the above age table)
          B. Using a reference (&) you work with the original elements without copying.
          C. 'numeric_limits<int>' gives specializations access for arithmetic type 'int'.
          D. 'numeric_limits<int>::max()' accesses the STATIC method max() which gives the maximum number that an integer can hold.
    */
    for (const auto& entry : graph) { // entry.first = key = node name
        distances[entry.first] = numeric_limits<int>::max(); // set high so that it can be replaced
    }
    distances[start] = 0;
    /* Now it is like (see the first table page above)
     * distances['A'] = {0};
       distances['B'] = {∞};  // 2147483647
       distances['C'] = {∞};
       distances['D'] = {∞};
       distances['E'] = {∞};
    */

    // 3. Create a set to keep track of visited nodes
    unordered_set<char> visited;
    // 4. Start the process until we complete our visited table
    while (visited.size() < graph.size()) {
        /* Find the node with the shortest distance from the start that has not been visited yet */
        // 4A. By default, we do not know yet which one is the current node ('\0' = None = null)
        char current_node = '\0';
        // 4B. Choose which one is the current_node; the current_node is the one with the shortest distance
        //     The rest ones might not have been visited yet because they still might have a distance of ∞
        for (auto &graph_node: graph) {
            const char &node = graph_node.first; // each time access the (original) nodeName (e.g., 1st loop 'A')
            /*
               "If node not in visited," however, '.find' does not return a boolean,
                but the position of the matching character (from 0 to size()-1) or string.
                A standard way to say "If node not in visited" is to use "visited.find(node) == visited.end()",
                where "visited.end()" (hypothetically) points to a position after the last element.
            */
            if (visited.find(node) == visited.end() &&
                (current_node == '\0' || distances[node] < distances[current_node])) {
                /* In the 1st for-loop, our current node is null; this will be replaced by 'A'
                  * but in the next 'for-loop' iteration 'distance[A] < distance[B]', thus, skip, etc.
                  */
                current_node = node;
            }
        }

        // 4C. Mark the current node as visited
        visited.insert(current_node); // For the 1st 'while-loop' only 'A' has been added

        // 4D. Now, explore the graph and UPDATE the distance map
        for (auto &graph_node: graph.at(current_node)) {

            // E.g., When we access graph.at('A'), we retrieve the neighbors/vector of pairs {{'B', 2}, {'D', 1}}
            char graph_node_neighbor_name = graph_node.first; // 'B' and then 'D'
            int graph_node_neighbor_distance = graph_node.second; // 2 and then 1

            /* If the (total of the distance that we have traveled so far + distance to travel for the new graph_node)
               is smaller than the current distance value existing in the distance_map for the current_node, replace it.
               Note: In the 1st 'while-loop' iteration, the '∞' will be replaced.
            */
            if (distances[current_node] != numeric_limits<int>::max() &&
                distances[current_node] + graph_node_neighbor_distance < distances[graph_node_neighbor_name]) {
                distances[graph_node_neighbor_name] = distances[current_node] + graph_node_neighbor_distance;
            }
        }
    }

    // 5. Return the final table
    return distances;
}

int main() {
    Graph graph;
    /* We actually have here a map/dictionary declaring:
     * unordered_map<'start_node', vector<pair<'where can it go', its value/distance>>>
    */
    graph['A'] = {{'B', 2}, {'D', 1}};
    graph['B'] = {{'D', 3}, {'E', 2}};
    graph['C'] = {};
    graph['D'] = {{'E', 5}, {'F', 4}};
    graph['E'] = {{'F', 6}, {'C', 3}};
    graph['F'] = {{'E', 1}, {'C', 2}};

    char start_node = 'A';
    unordered_map<char, int> table = dijkstra(graph, start_node);

    for (const auto& entry : table) {
        cout << start_node << " to node " << entry.first << ": " << entry.second << endl;
    }

    return 0;
}

