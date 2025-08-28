#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>

using namespace std;

struct Node {
  int x, y;
  double f, g, h; // f = g + h
  Node *parent;

  Node(int x, int y, double g, double h, Node *p = nullptr)
      : x(x), y(y), g(g), h(h), f(g + h), parent(p) {}
};

// Comparator for priority queue (min-heap on f)
struct CompareNode {
  bool operator()(const Node *a, const Node *b) const {
    return a->f > b->f;
  }
};

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path;

  // Directions (4-neighbors). If you want diagonal moves, add 8 directions.
  vector<pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  // Open list (priority queue by lowest f)
  priority_queue<Node *, vector<Node *>, CompareNode> open;

  // Visited map to store best g cost seen for each cell
  vector<vector<double>> g_cost(rows, vector<double>(cols, 1e9));

  // Start node
  Node *startNode = new Node(start.first, start.second, 0,
                             heuristic(start.first, start.second, goal.first, goal.second));
  open.push(startNode);
  g_cost[start.first][start.second] = 0;

  Node *goalNode = nullptr;

  while (!open.empty()) {
    Node *current = open.top();
    open.pop();

    // Check if reached goal
    if (current->x == goal.first && current->y == goal.second) {
      goalNode = current;
      break;
    }

    for (auto d : directions) {
      int nx = current->x + d.first;
      int ny = current->y + d.second;

      if (!isvalid(nx, ny))
        continue;

      double new_g = current->g + 1; // cost to move to neighbor (grid uniform cost)

      if (new_g < g_cost[nx][ny]) {
        g_cost[nx][ny] = new_g;
        double h = heuristic(nx, ny, goal.first, goal.second);
        Node *neighbor = new Node(nx, ny, new_g, h, current);
        open.push(neighbor);
      }
    }
  }

  // Reconstruct path if found
  if (goalNode) {
    Node *curr = goalNode;
    while (curr) {
      path.push_back({curr->x, curr->y});
      curr = curr->parent;
    }
    reverse(path.begin(), path.end());
  }

  return path;
}

