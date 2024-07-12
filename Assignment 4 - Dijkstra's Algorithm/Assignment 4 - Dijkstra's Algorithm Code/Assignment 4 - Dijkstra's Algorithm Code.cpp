#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm> // Include this header for std::reverse
#include <stack>

// Custom comparator for the priority queue
class Compare {
public:
    bool operator() (std::pair<int, int> a, std::pair<int, int> b) {
        return a.second > b.second;
    }
};

// Dijkstra's Algorithm implementation
std::pair<int, std::vector<int>> dijkstra(std::unordered_map<int, std::unordered_map<int, int>>& graph, int start, int end) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, Compare> pq;
    std::unordered_map<int, int> distances;
    std::unordered_map<int, int> previous;
    std::vector<int> path;
    std::unordered_map<int, bool> visited;

    // Initialize distances to infinity
    for (const auto& node : graph) {
        distances[node.first] = std::numeric_limits<int>::max();
        visited[node.first] = false;
    }
    distances[start] = 0;
    pq.push({ start, 0 });

    while (!pq.empty()) {
        int current = pq.top().first;
        pq.pop();

        if (visited[current]) continue;
        visited[current] = true;

        if (current == end) break;

        for (const auto& neighbor : graph[current]) {
            int distance = distances[current] + neighbor.second;
            if (distance < distances[neighbor.first]) {
                distances[neighbor.first] = distance;
                previous[neighbor.first] = current;
                pq.push({ neighbor.first, distance });
            }
        }
    }

    int total_cost = distances[end];
    for (int at = end; at != start; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order

    return { total_cost, path };
}

int main() {
    // Define the graph using an adjacency list
    std::unordered_map<int, std::unordered_map<int, int>> graph;
    graph[1] = { {2, 1}, {3, 4} };
    graph[2] = { {1, 1}, {3, 2}, {4, 5} };
    graph[3] = { {1, 4}, {2, 2}, {4, 1} };
    graph[4] = { {2, 5}, {3, 1} };

    int start_node, end_node;
    std::cout << "Enter the starting node: ";
    std::cin >> start_node;
    std::cout << "Enter the ending node: ";
    std::cin >> end_node;

    auto result = dijkstra(graph, start_node, end_node);
    int cost = result.first;
    std::vector<int> path = result.second;

    std::cout << "The cost of the shortest path from " << start_node << " to " << end_node << " is " << cost << std::endl;
    std::cout << "The path of the lowest cost is: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    return 0;
}