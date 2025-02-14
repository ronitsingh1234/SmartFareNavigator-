#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <climits>

using namespace std;

#define INF INT_MAX

class RoadMap {
public:
    unordered_map<int, vector<pair<int, int>>> connections;

    void insertRoad(int source, int destination, int cost) {
        connections[source].push_back({destination, cost});
        connections[destination].push_back({source, cost});
    }
};

// Dijkstra's Algorithm for shortest path
vector<int> computeShortestPath(RoadMap &map, int start, int totalNodes) {
    vector<int> minDistance(totalNodes, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> priorityQ;

    minDistance[start] = 0;
    priorityQ.push({0, start});

    while (!priorityQ.empty()) {
        int currentNode = priorityQ.top().second;
        priorityQ.pop();

        for (auto &neighbor : map.connections[currentNode]) {
            int nextNode = neighbor.first;
            int weight = neighbor.second;

            if (minDistance[currentNode] + weight < minDistance[nextNode]) {
                minDistance[nextNode] = minDistance[currentNode] + weight;
                priorityQ.push({minDistance[nextNode], nextNode});
            }
        }
    }
    return minDistance;
}

// Manhattan distance heuristic for A* Algorithm
int estimateCost(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<int> findOptimalRoute(RoadMap &map, unordered_map<int, pair<int, int>> &locations, int start, int end, int totalNodes) {
    vector<int> travelCost(totalNodes, INF);
    unordered_map<int, int> previousNode;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> priorityQ;

    travelCost[start] = 0;
    priorityQ.push({0, start});

    while (!priorityQ.empty()) {
        int currentNode = priorityQ.top().second;
        priorityQ.pop();

        if (currentNode == end) break;

        for (auto &neighbor : map.connections[currentNode]) {
            int nextNode = neighbor.first;
            int weight = neighbor.second;

            int estimatedCost = estimateCost(locations[nextNode].first, locations[nextNode].second, locations[end].first, locations[end].second);
            int totalCost = travelCost[currentNode] + weight + estimatedCost;

            if (totalCost < travelCost[nextNode]) {
                travelCost[nextNode] = totalCost;
                priorityQ.push({totalCost, nextNode});
                previousNode[nextNode] = currentNode;
            }
        }
    }

    vector<int> route;
    int node = end;
    while (node != start) {
        route.push_back(node);
        node = previousNode[node];
    }
    route.push_back(start);
    reverse(route.begin(), route.end());

    return route;
}

// Compute fare based on distance
double estimateFare(int distance) {
    double baseCharge = 50.0;
    double perKmRate = 10.0;
    return baseCharge + (distance * perKmRate);
}

int main() {
    RoadMap cityMap;
    unordered_map<int, pair<int, int>> coordinates = {
        {0, {0, 0}}, {1, {2, 3}}, {2, {4, 5}}, {3, {7, 8}}, {4, {10, 12}}
    };

    cityMap.insertRoad(0, 1, 5);
    cityMap.insertRoad(1, 2, 7);
    cityMap.insertRoad(2, 3, 6);
    cityMap.insertRoad(3, 4, 8);
    cityMap.insertRoad(0, 2, 9);
    cityMap.insertRoad(1, 3, 4);

    int totalNodes = 5, startPoint = 0, destination = 4;

    vector<int> shortestDistances = computeShortestPath(cityMap, startPoint, totalNodes);
    cout << "Shortest Distance (Dijkstra): " << shortestDistances[destination] << " km" << endl;

    vector<int> bestRoute = findOptimalRoute(cityMap, coordinates, startPoint, destination, totalNodes);
    cout << "Optimized Route (A*): ";
    for (int node : bestRoute) cout << node << " ";
    cout << endl;

    double fare = estimateFare(shortestDistances[destination]);
    cout << "Estimated Fare: " << fare << endl;

    return 0;
}
