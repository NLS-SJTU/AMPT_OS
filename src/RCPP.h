#ifndef RCPP_H
#define RCPP_H

#include "TopoMetric.h"
#include <opencv2/core/core.hpp>

#include "graph/graph.h"
#include "routing/routing_problem.h"
#include "graph/graph_factory.h"
#include "graph/graph_utils.h"
#include "utilities.h"

using namespace std;

class RCPP
{
public:
    RCPP(){}
    ~RCPP(){}

    // read parameter
    void readParam(string paramfile);

    // calculate RCPP and store path in _path_vec
    // if end_id < 0, end node is not set and all nodes can be end node to get shortest path
    void calRCPPPath(TopoMetric_c* _graph, int start_id, int end_id, vector<int>& _path_node_vec);

private:
    // calculate circuit length
    float calCircuitLen(vector<int> edge_id_vec);

    //
    Graph graph_h;

    // whether graph is undirected
    bool undirected;
};


#endif
