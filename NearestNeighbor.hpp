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

class Node
{
public:
    Node(int id, double x, double y) : id_(id), x_(x), y_(y) {}

    int getId() const
    {
        return id_;
    }
    double getX() const
    {
        return x_;
    }
    double getY() const
    {
        return y_;
    }

    // Calculating distance
   static double distance(const NODE &a, const NODE &b)
    {
        return std::hypot(a.x_ - b.x_, a.y_ - b.y_);
    }

private:
    int id_;
    double x_;
    double y_;
};

// Function to read nodes from a file and return a vector of Nodes
std::vector<Node> readNodes(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open the input file: " << filename << std::endl;
        return {};
    }

    std::vector<Node> nodes;
    std::string line;
    int id;
    double x, y;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        if (iss >> id >> x >> y)
        {
            nodes.emplace_back(id, x, y);
        }
    }

    return nodes;
}

// Function to calculate total distance given a path
double calculateTotalDistance(const std::vector<Node> &nodes, const std::vector<int> &path)
{
    double totalDistance = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        totalDistance += Node::distance(nodes[path[i] - 1], nodes[path[i + 1] - 1]);
    }
    return totalDistance;
}

// Function to perform the nearest neighbor algorithm on a set of nodes
void nearestNeighbor(const std::string &filename)
{
    auto nodes = readNodes(filename);
    if (nodes.empty())
    {
        return;
    }

    std::vector<bool> visited(nodes.size(), false);
    std::vector<int> path;
    path.reserve(nodes.size());
    double totalDistance = 0.0;

    auto current = nodes.begin();
    path.push_back(current->getId());
    visited[current - nodes.begin()] = true;

    auto startTime = std::chrono::high_resolution_clock::now();

    for (size_t i = 1; i < nodes.size(); ++i)
    {
        auto nearest = nodes.end();
        double minDistance = std::numeric_limits<double>::max();

        // Find the nearest unvisited node
        for (const auto &node : nodes)
        {
            if (!visited[node.getId() - 1])
            {
                double distance = Node::distance(*current, node);
                if (distance < minDistance)
                {
                    nearest = nodes.begin() + (node.getId() - 1);
                    minDistance = distance;
                }
            }
        }

        // Mark the nearest node as visited and update the total distance
        visited[nearest - nodes.begin()] = true;
        totalDistance += minDistance;
        path.push_back(nearest->getId());
        current = nearest;
    }

    // Complete the path by adding the distance to the starting node
    totalDistance += Node::distance(*current, nodes.front());
    path.push_back(nodes.front().getId());

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // Output the path, total distance, and execution time
    for (int id : path)
    {
        std::cout << id << " ";
    }

    double calculatedDistance = calculateTotalDistance(nodes, path);

    std::cout << "\nCalculated Distance: " << calculatedDistance << "\nExecution Time: " << duration.count() << " ms\n";
}

#endif // NEAREST_NEIGHBOR_HPP

