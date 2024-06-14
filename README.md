# Drone_path
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include "matplotlibcpp"
using namespace std;
namespace plt = matplotlibcpp;

struct Point {
    double x, y;
};

double distance(const Point& p1, const Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

vector<int> bruteForceTSP(const vector<Point>& points, double& minDist) {
    vector<int> perm(points.size());
    vector<int> bestPath;
    minDist = numeric_limits<double>::max();

    for (size_t i = 0; i < points.size(); ++i) {
        perm[i] = i;
    }

    do {
        double currentDist = 0;
        for (size_t i = 0; i < points.size() - 1; ++i) {
            currentDist += distance(points[perm[i]], points[perm[i + 1]]);
        }
        currentDist += distance(points[perm.back()], points[perm[0]]); // Return to start

        if (currentDist < minDist) {
            minDist = currentDist;
            bestPath = perm;
        }
    } while (next_permutation(perm.begin(), perm.end()));

    bestPath.push_back(bestPath[0]); 
    return bestPath;
}

vector<int> divideAndConquerTSP(vector<Point>& points) {
    vector<int> path;
    vector<bool> visited(points.size(), false);
    path.push_back(0);
    visited[0] = true;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        int last = path.back();
        int next = -1;
        double minDist = numeric_limits<double>::max();
        for (size_t j = 0; j < points.size(); ++j) {
            if (!visited[j] && distance(points[last], points[j]) < minDist) {
                next = j;
                minDist = distance(points[last], points[j]);
            }
        }
        path.push_back(next);
        visited[next] = true;
    }
    path.push_back(0); 
    return path;
}

double calculateTotalDistance(const vector<Point>& points, const vector<int>& path) {
    double totalDistance = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        totalDistance += distance(points[path[i]], points[path[i + 1]]);
    }
    return totalDistance;
}

void visualizePath(const vector<Point>& points, const vector<int>& path) {
    vector<double> x, y;
    for (const auto& point : points) {
        x.push_back(point.x);
        y.push_back(point.y);
    }

    plt::plot(x, y, "ro");

    for (size_t i = 0; i < path.size() - 1; ++i) {
        vector<double> line_x = {points[path[i]].x, points[path[i + 1]].x};
        vector<double> line_y = {points[path[i]].y, points[path[i + 1]].y};
        plt::plot(line_x, line_y, "b-");
    }

    plt::show();
}

int main() {
    vector<Point> deliveryPoints = {{0, 0}, {1, 2}, {3, 1}, {6, 5}, {7, 8}, {2, 4}, {5, 3}, {8, 7}};
    double bruteForceDist, divideAndConquerDist;

    // Brute-Force Approach
    auto start = chrono::high_resolution_clock::now();
    auto bruteForcePath = bruteForceTSP(deliveryPoints, bruteForceDist);
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> bruteForceTime = end - start;

    // Divide-and-Conquer Approach
    start = chrono::high_resolution_clock::now();
    auto divideAndConquerPath = divideAndConquerTSP(deliveryPoints);
    end = chrono::high_resolution_clock::now();
    chrono::duration<double> divideAndConquerTime = end - start;
    divideAndConquerDist = calculateTotalDistance(deliveryPoints, divideAndConquerPath);

    // Output Results
    cout << "Brute-Force Approach:\n";
    cout << "Total Distance: " << bruteForceDist << "\n";
    cout << "Time Taken: " << bruteForceTime.count() << " seconds\n";

    cout << "Divide-and-Conquer Approach:\n";
    cout << "Total Distance: " << divideAndConquerDist << "\n";
    cout << "Time Taken: " << divideAndConquerTime.count() << " seconds\n";

    // Visualize Paths
    cout << "\nVisualizing Brute-Force Path:\n";
    visualizePath(deliveryPoints, bruteForcePath);

    cout << "\nVisualizing Divide-and-Conquer Path:\n";
    visualizePath(deliveryPoints, divideAndConquerPath);

    return 0;
}
