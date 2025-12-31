#ifndef A_STAR_H
#define A_STAR_H

#include <geometry_msgs/Point.h>
#include <vector>

struct Node
{
    geometry_msgs::Point position;
    double g = 0.0;
    double h = 0.0;
    double f = 0.0;
    Node* parent = nullptr;
};

/* Implementation of A star algorithm
        See a_star_node.cpp for usage */
class AStar
{
public:
    AStar(int width, int height);
    
    /* Set grid cell as obstacle */
    void SetObstacle(int x, int y);

    /* Find the most optimal path using the A* pathfinding algorithm */
    std::vector<geometry_msgs::Point> FindPath(geometry_msgs::Point start, geometry_msgs::Point goal);

private:
    int m_width; // Grid width
    int m_height; // Grid height
    std::vector<int> m_grid; // Grid

    /* Helper function to free memory */
    void Cleanup(std::vector<Node*>& list);

    /* Compare the position of two points and returns true if they are the same position */
    bool IsSamePosition(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

    /* Determine if a point is an obstacle */
    bool IsObstacle(const geometry_msgs::Point& p);
    
    /* Calculate heauristic using Euclidean distance */
    double Distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    
    /* Return the list of up to 4 neighbours around the current cell */
    std::vector<geometry_msgs::Point> GetNeighbours(const geometry_msgs::Point& p);
};

#endif