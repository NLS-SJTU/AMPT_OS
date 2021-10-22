#ifndef ACTLOCOVERLAY_H
#define ACTLOCOVERLAY_H

#include "../TopoMetric.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>

using namespace std;

class ActLocOverlay
{
public:
    ActLocOverlay(){}
    ~ActLocOverlay(){}

    void setPriorGraph(TopoMetric_c _pg){prior_graph = _pg;}
    void setVisitedGraph(TopoMetric_c _vg){visited_graph = _vg;}

    void overlayMap(vector<int> _node_id_prior_vec, vector<Eigen::Quaterniond> _q_vec, vector<Eigen::Vector3d> _t_vec);

    // return next target pose in visited graph relative to the first estimation in queue _node_id_prior_vec
    Eigen::Vector3d action();
private:
    TopoMetric_c prior_graph, visited_graph;
    TopoMetric_c overlay_graph;

    // this is to store how many estimation in this node of the overlay map
    Eigen::VectorXd num_of_exist_estimation;

    // remember the base map
    Eigen::Quaterniond qr;
    Eigen::Vector3d tr;

    int start_id;

    double max_overlay;
};

#endif

