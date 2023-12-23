/*
CSCI335 Fall 2023
Assignment 3- Nearest Neighbor 
Name: Dawa Sonam
Date: December 22, 2023
*/

// NEAREST_NEIGHBOR_HPP
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

// Class representing a node with an ID, latitude, and longitude
class NODE
{
public:
    // Constructor to initialize NODE with an ID, latitude, and longitude
    NODE(int id, double lat, double lon) : id_(id), lat_(lat), lon_(lon) {}

    // Getter functions to retrieve the ID, latitude, and longitude of the NODE
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
    
    // Static method to calculate Euclidean distance between two NODEs
    static double distance(const NODE &a, const NODE &b)
    {
        return std::hypot(a.lat_ - b.lat_, a.lon_ - b.lon_);
    }

private:
    // Private members for node ID, latitude, and longitude
    int id_;
    double lat_;
    double lon_;
};

// Function to read nodes from a file and return a vector of NODEs
std::vector<NODE> read_nodes_from_file(const std::string& filename)
{   
    // Open the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};  // Return an empty vector on error
    }

    std::vector<NODE> parsed_nodes;
    std::string current_line;
    int id;
    double lat, lon;

    // Parse each line in the file to extract ID, latitude, and longitude
    while (std::getline(file, current_line))
    {
        std::istringstream iss(current_line);
        if (iss >> id >> lat >> lon)
        {
            parsed_nodes.emplace_back(id, lat, lon);
        }
    }
    // Return the vector of parsed nodes
    return parsed_nodes;
}

// Function to perform the nearest neighbor algorithm on a set of nodes
void nearestNeighbor(const std::string &filename)
{
    // Read nodes from the file
    auto parsed_nodes = read_nodes_from_file(filename);
    if (parsed_nodes.empty()) {
        return;
    }

    // Initialize data structures to track visited nodes, path, and total distance
    std::vector<bool> visited(parsed_nodes.size(), false);
    std::vector<int> path;
    double total_distance = 0.0;

    auto current_node_it = parsed_nodes.begin();
    path.push_back(current_node_it->getId());
    visited[current_node_it - parsed_nodes.begin()] = true;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Nearest neighbor algorithm loop
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

        // Update visited nodes, total distance, and path
        visited[nearest_node_it - parsed_nodes.begin()] = true; 
        total_distance += minDistance;
        path.push_back(nearest_node_it->getId());
        current_node_it = nearest_node_it;
    }

    // Complete the path by adding the distance to the starting node
    total_distance += NODE::distance(*current_node_it, parsed_nodes.front());
    path.push_back(parsed_nodes.front().getId());

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Output the path, total distance, and execution time
    for (int id : path)
    {
        std::cout << id << " ";
    }

    std::cout << "\nTotal Distance: " << total_distance << "\nTime in ms: " << duration.count() << std::endl;
}

#endif // NEAREST_NEIGHBOR_HPP
