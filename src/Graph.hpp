/* This hpp file declares a graph is constructed
from the roadmap built in robotic scenarios. */

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <uniform_object_rearrangement/Edge.h>

class Graph_t
{
    // the size of the graph
    int m_nNodes;

    // specify neighbors(edges) and edge cost of the graph
    std::vector<std::vector<int>> m_nodeNeighbors;
    std::vector<std::vector<float>> m_edgeCosts;
    std::vector<std::vector<float>> m_nodeStates;
    std::vector<std::vector<int>> m_edgeStatus;

    // // specify start and goal
    // int m_start;
    // std::vector<float> m_startNode;
    // int m_goal;
    // std::vector<float> m_goalNode;

    // file reader
    std::ifstream m_inFile_;


public:
    // constructor
    Graph_t() {}
    Graph_t(std::string samples_file, std::string connections_file);
    void constructGraph(std::string samples_file, std::string connections_file);
    void specify_nodeStates(std::string samples_file);
    void specify_neighborCosts(std::string connections_file);
    void specify_edgeStatus();
    // void connectStartAndGoal(std::string task_file);
    float computeDist(std::vector<float> n1, std::vector<float> n2);

    // getter
    int getnNodes() { return m_nNodes; }
    std::vector<float> getState(int idx) { return m_nodeStates[idx]; }
    // int getStart() { return m_start; }
    // int getGoal() { return m_goal; }
    // std::vector<float> getStartState() { return m_startNode; }
    // std::vector<float> getGoalState() { return m_goalNode; }
    std::vector<int> getNodeNeighbors(int id) { return m_nodeNeighbors[id]; }
    float getEdgeCost(int id1, int id2) { return m_edgeCosts[id1][id2]; }
    int getEdgeStatus(int id1, int id2) { return m_edgeStatus[id1][id2]; }
    void modifyEdge(std::vector<uniform_object_rearrangement::Edge> &violated_edges, int query_idx);

    // printer
    void printStates();
    void printNeighbors();
    void printEdgeCosts();

    // destructor
    ~Graph_t() {}

};



#endif