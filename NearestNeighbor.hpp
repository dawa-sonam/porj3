/*
CSCI335 Fall 2023
Assignment 3- Nearest Neighbor 
Name: Dawa Sonam
Date: December 22, 2023
*/

// NearestNeighbor.hpp
#ifndef NEAREST_NEIGHBOR_HPP
#define NEAREST_NEIGHBOR_HPP

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <chrono>
#include <limits>
#include <algorithm>
#include <sstream>

class NODE
{
public:
    NODE(int id, double lat, double lon) : id_(id), lat_(lat), lon_(lon) {}

    int getId() const
    {
        return id_;
    }
    double getLat() const
    {
        return lat_;
    }
    double getLon() const
    {
        return lon_;
    }
    // calculating Euclidean distance between two nodes
   // Euclidean distance formula
    static double distance(const NODE &a, const NODE &b)
    {
        double dx = b.lat_ - a.lat_;
        double dy = b.lon_ - a.lon_;
        return std::sqrt(dx * dx + dy * dy);
    }


private:
    int id_;
    double lat_;
    double lon_;
};

// Function to read nodes from a file and return a vector of NODEs
std::vector<NODE> read_nodes_from_file(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};  // Return an empty vector on error
    }

    // Declare variables with clear names
    std::vector<NODE> parsed_nodes;
    std::string current_line;
    int node_id;
    double node_x, node_y;

    while (std::getline(file, current_line)) {
        std::istringstream line_stream(current_line);
        if (line_stream >> node_id >> node_x >> node_y) {
            parsed_nodes.emplace_back(node_id, node_x, node_y);
        } else {
            // Handle potential parsing errors if needed
        }
    }

    return parsed_nodes;
}

// Function to perform the nearest neighbor algorithm on a set of nodes
void nearestNeighbor(const std::string &filename)
{
    auto nodes = read_nodes_from_file(filename);
    if (nodes.empty()) {
        return;
    }

    std::vector<bool> visited(nodes.size(), false);
    std::vector<int> path;
    double total_distance = 0.0;

    auto current_node_it = nodes.begin();
    visited[current_node_it - nodes.begin()] = true;
    path.push_back(current_node_it->getId());

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 1; i < nodes.size(); ++i) {
        auto nearest_node_it = std::min_element(
            nodes.begin(), nodes.end(),
            [&](const NODE& node1, const NODE& node2) {
                return !visited[node1.getId() - 1] &&
                       NODE::distance(*current_node_it, node1) <
                       NODE::distance(*current_node_it, node2);
            }
        );

        total_distance += NODE::distance(*current_node_it, *nearest_node_it);
        path.push_back(nearest_node_it->getId());
        visited[nearest_node_it - nodes.begin()] = true;
        current_node_it = nearest_node_it;
    }


    // Complete the path by adding the distance to the starting node
    total_distance += NODE::distance(*current_node_it, nodes.front());
    path.push_back(nodes.front().getId());

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Output the path, total distance, and execution time
    for (int id : path)
    {
        std::cout << id << " ";
    }
    std::cout << "\nTotal Distance: " << total_distance << "\nTime in ms: " << duration.count() << std::endl;
}

// Total Distance: 1.03496e+06
// Time in ms: 5917

#endif // NEAREST_NEIGHBOR_HPP
