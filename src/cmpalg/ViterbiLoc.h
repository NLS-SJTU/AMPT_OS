#ifndef VITERBILOC_H
#define VITERBILOC_H

#include "../TopoMetric.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>
#include <fstream>

using namespace std;

class ViterbiLoc
{
public:
    ViterbiLoc(){}
    ~ViterbiLoc(){}

    void init(TopoMetric_c prior_graph);

    void reset(string graph_path);

    Eigen::VectorXd update(Eigen::VectorXd prob_obs, Eigen::Vector3d action);

    Eigen::VectorXd predict(Eigen::Vector3d action);

    Eigen::VectorXi getBestPath(int endid);

    vector<Eigen::VectorXd> _prob_t_vec;
    Eigen::VectorXd _prob_t;
    TopoMetric_c _prior_graph;
    vector<Eigen::VectorXi> _last_node_vec;

    string _save_path;
};



#endif
