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
        return std::hypot(a.lat_ - b.lat_, a.lon_ - b.lon_);
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

    std::vector<NODE> parsed_nodes;
    std::string current_line;
    int id;
    double lat, lon;

    while (std::getline(file, current_line))
    {
        std::istringstream iss(current_line);
        if (iss >> id >> lat >> lon)
        {
            parsed_nodes.emplace_back(id, lat, lon);
        }
    }

    return parsed_nodes;
}

// Function to perform the nearest neighbor algorithm on a set of nodes
void nearestNeighbor(const std::string &filename)
{
    auto parsed_nodes = read_nodes_from_file(filename);
    if (parsed_nodes.empty()) {
        return;
    }

    std::vector<bool> visited(parsed_nodes.size(), false);
    std::vector<int> path;
    double total_distance = 0.0;

    auto current_node_it = parsed_nodes.begin();
    visited[current_node_it - parsed_nodes.begin()] = true;
    path.push_back(current_node_it->getId());

    auto start_time = std::chrono::high_resolution_clock::now();

    // for (size_t i = 1; i < parsed_nodes.size(); ++i) {
    //     auto nearest_node_it = std::min_element(
    //         parsed_nodes.begin(), parsed_nodes.end(),
    //         [&](const NODE& node1, const NODE& node2) {
    //             return !visited[node1.getId() - 1] &&
    //                    NODE::distance(*current_node_it, node1) <
    //                    NODE::distance(*current_node_it, node2);
    //         }
    //     );
    
        for (size_t i = 1; i < parsed_nodes.size(); ++i)
        {
            auto nearest_node_it = parsed_nodes.end();
            double minDistance = std::numeric_limits<double>::max();

            // Find the nearest unvisited node
            for (auto it = parsed_nodes.begin(); it != parsed_nodes.end(); ++it)
            {
                if (!visited[it - parsed_nodes.begin()])
                {
                    double distance = NODE::distance(*current_node_it, *it);
                    if (distance < minDistance)
                    {
                        nearest_node_it = it;
                        minDistance = distance;
                    }
                }
            }

        total_distance += NODE::distance(*current_node_it, *nearest_node_it);
        path.push_back(nearest_node_it->getId());
        visited[nearest_node_it - parsed_nodes.begin()] = true; // Fix the index here
        current_node_it = nearest_node_it;
    }

    // Complete the path by adding the distance to the starting node
    total_distance += NODE::distance(*current_node_it, parsed_nodes.front());
    path.push_back(parsed_nodes.front().getId());

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Output the path, total distance, and execution time
    std::cout << "Path: ";
    for (int id : path) {
        std::cout << id << " ";
    }

    std::cout << "\nTotal Distance: " << total_distance << "\nExecution time in ms: " << duration.count() << std::endl;
}


#endif // NEAREST_NEIGHBOR_HPP
