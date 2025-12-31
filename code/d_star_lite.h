#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <geometry_msgs/Point.h>
#include <vector>
#include <set>
#include <limits>
#include <utility>

class DStarLite
{
public:
    DStarLite(int width = 1, int height = 1);

    void SetObstacle(int x, int y, bool obstacle = true);

    std::vector<geometry_msgs::Point> FindPath(geometry_msgs::Point start, geometry_msgs::Point goal);

    static const double INF;

private:
    struct Node 
    {
        double g = INF;
        double rhs = INF;
        double h = 0.0; 
        bool obstacle = false;
    };

    using Key = std::pair<double,double>;
    using OpenEntry = std::pair<Key,int>;

    int m_width;
    int m_height;
    std::vector<Node> m_nodes;

    std::set<OpenEntry> m_open;
    std::vector<Key> m_key_of;

    int m_start_idx;
    int m_goal_idx;
    int m_last_start_idx;
    double m_km; 

    /* Helpers */
    int ToIndex(int x, int y) const;
    void FromIndex(int idx, int &x, int &y) const;
    bool InBounds(int x, int y) const;
    std::vector<int> Successors(int idx) const;
    std::vector<int> Predecessors(int idx) const;
    Key CalculateKey(int idx) const;
    void InsertOrUpdateOpen(int idx);
    void RemoveFromOpen(int idx);
    void UpdateVertex(int idx);
    void ComputeShortestPath();
    double Cost(int a_idx, int b_idx) const;
    double Heuristic(int a_idx, int b_idx) const;
    void InitializeForNewSearch();
};

#endif // DSTAR_LITE_H
