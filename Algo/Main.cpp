//Usage: C++ program to fuse GPS coordinates with RRT for SLAM(Simultaneous Localization and Mapping), 
//the RRT algorithm is modified to accept GPS coordinates as input

//Version: 1.0, Date: 2 January 2023

//Owner: Arnab Mitra

//Contact: papan.mitra.2121@gmail.com

#include <iostream>
#include <vector>
#include <utility> 
#include <tuple>
#include <cmath>

using namespace std;

//Function to generate random numbers 
double random_number_generator()
{
    double random_number = (double)rand() / (double)RAND_MAX;
    return random_number;
}

//Function to calculate the Euclidean distance between two points
double euclidean_distance(pair<double, double> point1, pair<double, double> point2)
{
    double dist = sqrt(pow(point1.first - point2.first, 2) + pow(point1.second - point2.second, 2));
    return dist;
}

//Function to find the nearest neighbour in the graph
pair<double, double> nearest_neighbour(vector<pair<double, double>> graph, pair<double, double> point)
{
    double min_dist = numeric_limits<double>::max();
    pair<double, double> nearest_point = { 0, 0 };

    //Iterate over the graph to find the nearest neighbour 
    for (int i = 0; i < graph.size(); i++)
    {
        double dist = euclidean_distance(graph[i], point);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_point = graph[i];
        }
    }

    return nearest_point;
}

//Function for the RRT algorithm modified to accept GPS coordinates
vector<pair<double, double>> rrt_with_gps(pair<double, double> start, pair<double, double> goal, int max_iterations)
{
    vector<pair<double, double>> graph;
    graph.push_back(start);

    while (max_iterations > 0)
    {
        //Generate random point 
        double x_rand = random_number_generator() * (goal.first - start.first) + start.first;
        double y_rand = random_number_generator() * (goal.second - start.second) + start.second;
        pair<double, double> rand_point = { x_rand, y_rand };

        //Find the nearest neighbour of the random point
        pair<double, double> nearest_neighbour_point = nearest_neighbour(graph, rand_point);

        //Generate new point in the direction of the random point
        double x_new = nearest_neighbour_point.first + (rand_point.first - nearest_neighbour_point.first) / euclidean_distance(rand_point, nearest_neighbour_point);
        double y_new = nearest_neighbour_point.second + (rand_point.second - nearest_neighbour_point.second) / euclidean_distance(rand_point, nearest_neighbour_point);
        pair<double, double> new_point = { x_new, y_new };

        //Add the new point to the graph 
        graph.push_back(new_point);

        //Decrement the number of iterations
        max_iterations--;
    }

    return graph;
}

int main()
{
    //Example inputs 
    pair<double, double> start = { 0, 0 };
    pair<double, double> goal = { 5, 5 };
    int max_iterations = 10000;

    //Function call 
    vector<pair<double, double>> graph = rrt_with_gps(start, goal, max_iterations);

    //Print the graph 
    for (int i = 0; i < graph.size(); i++)
    {
        cout << "(" << graph[i].first << "," << graph[i].second << ")" << endl;
        
    }
    std::cout << "        " << std::endl;
    std::cout << "Found it interesting? :).......For more such implementation please consider my profile for the interview,where I can discuss more about such ideas(cool). Thank you :)" << std::endl;
    return 0;
}