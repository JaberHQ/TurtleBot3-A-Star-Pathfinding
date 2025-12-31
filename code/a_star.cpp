#include "a_star.h"

AStar::AStar(int width, int height) 
    : m_width(width), m_height(height)
{
    // Initialise grid with all cells as 0
    m_grid.resize(m_width * m_height, 0);
}

void AStar::SetObstacle(int x, int y)
{
    // Set obstacle cells to 1
    m_grid[y * m_width + x] = 1;
}

std::vector<geometry_msgs::Point> AStar::FindPath(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    std::vector<Node*> openList; // Nodes to be explored
    std::vector<Node*> closedList; // Already explored nodes

    /* Define start node and goal node */
    Node* startNode = new Node;
    startNode->position = start;

    Node* goalNode = new Node;
    goalNode->position = goal;

    openList.push_back(startNode);

    /* While there are nodes to explore */
    while (!openList.empty())
    {
        // Find lowest f
        std::vector<Node*>::iterator currentIt =
            std::min_element(openList.begin(), openList.end(),
                [](Node* a, Node* b) { return a->f < b->f; });

        Node* currentNode = *currentIt; // Eexplore the current node with the lowest f value
        openList.erase(currentIt);
        closedList.push_back(currentNode);

        // If the current node is the goal, work backwards to reconstruct the path
        if (IsSamePosition(currentNode->position, goalNode->position))
        {
            std::vector<geometry_msgs::Point> path;
            while (currentNode != NULL)
            {
                path.push_back(currentNode->position);
                currentNode = currentNode->parent;
            }
            std::reverse(path.begin(), path.end());
            Cleanup(openList);
            Cleanup(closedList);
            delete goalNode;
            return path;
        }

        // Else expand neighbours
        std::vector<geometry_msgs::Point> neighbours = GetNeighbours(currentNode->position);
        for (size_t i = 0; i < neighbours.size(); i++)
        {
            geometry_msgs::Point newPos = neighbours[i];
            if (IsObstacle(newPos)) // Skip if obstacle
                continue;

            /* Skip if node is in closed list (has already been explored) */
            bool inClosedList = false;
            for (size_t j = 0; j < closedList.size(); j++)
            {
                if (IsSamePosition(closedList[j]->position, newPos))
                {
                    inClosedList = true; 
                    break;
                }
            }
            if (inClosedList)
                continue;

            /* Calculate g, h and f for each neighbour */
            Node* child = new Node;
            child->position = newPos;
            child->g = currentNode->g + Distance(currentNode->position, newPos);
            child->h = Distance(newPos, goalNode->position);
            child->f = child->g + child->h;
            child->parent = currentNode;

            /* If node is in open list (to be explored) with a lower cost (g), skip nide */
            bool skipChild = false;
            for (size_t k = 0; k < openList.size(); k++)
            {
                if (IsSamePosition(openList[k]->position, newPos) &&
                    child->g >= openList[k]->g)
                {
                    skipChild = true;
                    break;
                }
            }
            if (skipChild)
            {
                delete child;
                continue;
            }

            openList.push_back(child); // Else, add child to openlist to be explored 
        }
    }

    Cleanup(openList);
    Cleanup(closedList);
    delete goalNode;
    return std::vector<geometry_msgs::Point>();
}

void AStar::Cleanup(std::vector<Node*>& list)
{
    for (size_t i = 0; i < list.size(); i++)
        delete list[i];
    list.clear();
}

bool AStar::IsSamePosition(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    return static_cast<int>(a.x) == static_cast<int>(b.x) &&
            static_cast<int>(a.y) == static_cast<int>(b.y);
}

bool AStar::IsObstacle(const geometry_msgs::Point& p)
{
    if (p.x < 0 || p.y < 0 || p.x >= m_width || p.y >= m_height)
        return true;
    return m_grid[static_cast<int>(p.y) * m_width + static_cast<int>(p.x)] == 1;
}

double AStar::Distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

std::vector<geometry_msgs::Point> AStar::GetNeighbours(const geometry_msgs::Point& p)
{
    std::vector<geometry_msgs::Point> neighbours;
    int dirs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
    for (int i = 0; i < 4; i++)
    {
        geometry_msgs::Point np;
        np.x = p.x + dirs[i][0];
        np.y = p.y + dirs[i][1];
        neighbours.push_back(np);
    }
    return neighbours;
}