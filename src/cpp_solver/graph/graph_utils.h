#ifndef __GRAPH_UTILS_H__
#define __GRAPH_UTILS_H__

#include "graph.h"
#include <map>
#include <stack>
#include <vector>

namespace graph_utils {

    enum GraphType { DIRECTED, UNDIRECTED, MIXED };

    float circuitLength(Graph graph, std::vector<int> circuit);

    float eulerianCost(Graph graph, std::set<int> skipEdges = std::set<int>());

    GraphType detectGraphType(Graph graph);

    void refineEdges(Graph* graph, std::set<int> edges);

    std::vector<int> pathEdgesToVertices(std::vector<int> edgesPath, Graph g, int startVertexId);


    std::vector<std::vector<int>> tarjanConnectedComponents(Graph graph);
    void tarjanConnectedComponentsRecursion(Graph graph, int vertexID, std::map<int, int>* discoveryStep, std::map<int, int>* lowIndices, std::stack<int>* connectedAncestors, std::map<int, bool>* markedVertices, int* step, std::vector<std::vector<int>>* connectedComponents);


    void printVerticesInfo(Graph graph);
    void printEdgesInfo(Graph graph);










}




#endif
