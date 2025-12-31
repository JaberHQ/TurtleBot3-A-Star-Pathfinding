#include "d_star_lite.h"
#include <cmath>
#include <algorithm>
#include <cassert>

const double DStarLite::INF = 1e9;

DStarLite::DStarLite(int width, int height)
    : m_width(width), m_height(height),
      m_nodes(width * height),
      m_key_of(width * height, {INF, INF}),
      m_start_idx(-1), m_goal_idx(-1),
      m_last_start_idx(-1), m_km(0.0)
{
    
}

int DStarLite::ToIndex(int x, int y) const
{
    return y * m_width + x;
}

void DStarLite::FromIndex(int idx, int &x, int &y) const
{
    x = idx % m_width;
    y = idx / m_width;
}

bool DStarLite::InBounds(int x, int y) const
{
    return (x >= 0 && y >= 0 && x < m_width && y < m_height);
}

std::vector<int> DStarLite::Successors(int idx) const
{
    int x, y;
    FromIndex(idx, x, y);

    std::vector<int> result;
    const int directions[8][2] =
    {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},
        {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
    };

    for (int i = 0; i < 8; ++i)
    {
        int nx = x + directions[i][0];
        int ny = y + directions[i][1];

        if (InBounds(nx, ny))
        {
            result.push_back(ToIndex(nx, ny));
        }
    }
    return result;
}

std::vector<int> DStarLite::Predecessors(int idx) const
{
    return Successors(idx);
}

double DStarLite::Cost(int a_idx, int b_idx) const
{
    if (a_idx < 0 || b_idx < 0) 
        return INF;

    if (m_nodes[a_idx].obstacle || m_nodes[b_idx].obstacle) 
        return INF;

    int ax, ay, bx, by;
    FromIndex(a_idx, ax, ay);
    FromIndex(b_idx, bx, by);

    int dx = std::abs(ax - bx);
    int dy = std::abs(ay - by);

    if (dx + dy == 1) 
        return 1.0;
    if (dx == 1 && dy == 1) 
        return std::sqrt(2.0);

    return INF;
}

double DStarLite::Heuristic(int a_idx, int b_idx) const
{
    int ax, ay, bx, by;
    FromIndex(a_idx, ax, ay);
    FromIndex(b_idx, bx, by);

    int dx = std::abs(bx - ax);
    int dy = std::abs(by - ay);

    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

DStarLite::Key DStarLite::CalculateKey(int idx) const
{
    double g = m_nodes[idx].g;
    double rhs = m_nodes[idx].rhs;
    double h = m_nodes[idx].h;

    double minCost = std::min(g, rhs);
    return std::make_pair(minCost + h + m_km, minCost);
}

void DStarLite::InsertOrUpdateOpen(int idx)
{
    Key newKey = CalculateKey(idx);
    Key oldKey = m_key_of[idx];

    if (oldKey.first != INF || oldKey.second != INF)
    {
        m_open.erase(std::make_pair(oldKey, idx));
    }

    m_open.insert(std::make_pair(newKey, idx));
    m_key_of[idx] = newKey;
}

void DStarLite::RemoveFromOpen(int idx)
{
    Key oldKey = m_key_of[idx];
    if (oldKey.first == INF && oldKey.second == INF) 
        return;

    m_open.erase(std::make_pair(oldKey, idx));
    m_key_of[idx] = std::make_pair(INF, INF);
}

void DStarLite::UpdateVertex(int nodeIndex)
{
    if (nodeIndex == m_goal_idx) 
        return;

    double best = INF;
    std::vector<int> successors = Successors(nodeIndex);
    for (size_t i = 0; i < successors.size(); ++i)
    {
        int succ = successors[i];
        double c = Cost(nodeIndex, succ);
        if (c < INF)
        {
            best = std::min(best, c + m_nodes[succ].g);
        }
    }

    m_nodes[nodeIndex].rhs = best;

    if (m_nodes[nodeIndex].g != m_nodes[nodeIndex].rhs)
    {
        InsertOrUpdateOpen(nodeIndex);
    }
    else
    {
        RemoveFromOpen(nodeIndex);
    }
}

void DStarLite::ComputeShortestPath()
{
    while (!m_open.empty())
    {
        std::pair<Key, int> top = *m_open.begin();
        Key kTop = top.first;
        int u = top.second;

        Key kStart = CalculateKey(m_start_idx);

        bool shouldStop =
            (kTop.first > kStart.first) ||
            (kTop.first == kStart.first && kTop.second >= kStart.second);

        if (shouldStop && m_nodes[m_start_idx].rhs == m_nodes[m_start_idx].g)
        {
            break;
        }

        RemoveFromOpen(u);

        if (m_nodes[u].g > m_nodes[u].rhs)
        {
            m_nodes[u].g = m_nodes[u].rhs;
            std::vector<int> preds = Predecessors(u);
            for (size_t i = 0; i < preds.size(); i++)
            {
                UpdateVertex(preds[i]);
            }
        }
        else
        {
            m_nodes[u].g = INF;
            UpdateVertex(u);
            std::vector<int> preds = Predecessors(u);
            for (size_t i = 0; i < preds.size(); i++)
            {
                UpdateVertex(preds[i]);
            }
        }
    }
}

void DStarLite::InitializeForNewSearch()
{
    for (size_t i = 0; i < m_nodes.size(); i++)
    {
        m_nodes[i].g = INF;
        m_nodes[i].rhs = INF;
        m_nodes[i].h = 0.0;
        m_key_of[i] = std::make_pair(INF, INF);
    }

    m_open.clear();
    m_km = 0.0;
    m_last_start_idx = -1;
}

std::vector<geometry_msgs::Point> DStarLite::FindPath(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    if (!InBounds(sx, sy) || !InBounds(gx, gy)) 
        return std::vector<geometry_msgs::Point>();

    InitializeForNewSearch();

    m_start_idx = ToIndex(sx, sy);
    m_goal_idx = ToIndex(gx, gy);
    m_last_start_idx = m_start_idx;
    m_km = 0.0;

    for (size_t i = 0; i < m_nodes.size(); ++i)
    {
        m_nodes[i].h = Heuristic(i, m_start_idx);
    }

    m_nodes[m_goal_idx].rhs = 0.0;
    InsertOrUpdateOpen(m_goal_idx);
    ComputeShortestPath();

    std::vector<geometry_msgs::Point> path;
    if (m_nodes[m_start_idx].g >= INF && m_nodes[m_start_idx].rhs >= INF)
    {
        return path;
    }

    int current = m_start_idx;
    int safetyCounter = m_width * m_height + 10;

    while (current != m_goal_idx && safetyCounter-- > 0)
    {
        int cx, cy;
        FromIndex(current, cx, cy);

        geometry_msgs::Point p;
        p.x = cx;
        p.y = cy;
        path.push_back(p);

        int bestIdx = -1;
        double bestVal = INF;

        std::vector<int> successors = Successors(current);
        for (size_t i = 0; i < successors.size(); i++)
        {
            int succ = successors[i];
            double c = Cost(current, succ);
            if (c < INF)
            {
                double val = c + m_nodes[succ].g;
                if (val < bestVal)
                {
                    bestVal = val;
                    bestIdx = succ;
                }
            }
        }

        if (bestIdx == -1) 
            break;

        current = bestIdx;
    }

    if (current == m_goal_idx)
    {
        int gx_, gy_;
        FromIndex(current, gx_, gy_);
        geometry_msgs::Point goalPoint;
        goalPoint.x = gx_;
        goalPoint.y = gy_;
        path.push_back(goalPoint);
    }
    else
    {
        path.clear();
    }

    return path;
}

void DStarLite::SetObstacle(int x, int y, bool obstacle)
{
    if (!InBounds(x, y)) 
        return;

    int idx = ToIndex(x, y);
    if (m_nodes[idx].obstacle == obstacle) 
        return;

    m_nodes[idx].obstacle = obstacle;

    std::vector<int> affected;
    affected.push_back(idx);

    std::vector<int> preds = Predecessors(idx);
    for (size_t i = 0; i < preds.size(); i++)
    {
        affected.push_back(preds[i]);
    }

    for (size_t i = 0; i < affected.size(); i++)
    {
        UpdateVertex(affected[i]);
    }

    ComputeShortestPath();
}
