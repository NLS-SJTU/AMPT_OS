/// a similator to test pure topo actloc
/// no direction of robot, only position
/// zero of visited frame is at start node

#ifndef TOPOSIM_H
#define TOPOSIM_H

#include "TopoMetric.h"
#include <vector>
#include <opencv2/core/core.hpp>

using namespace std;

class TopoSim{
public:
    TopoSim(){}
    ~TopoSim(){}

    void readParam(const string& paramfile);

    void reset();

    // move to a target point in visited frame
    bool moveTo(Eigen::Vector3d target_visited);
    // move to a node in prior frame
    bool moveTo(int nextid_prior);
    // move to a relative target from current position in absolute direction
    bool moveToRela(Eigen::Vector3d target_relative);

    // move along a default path, return false when path is over
    Eigen::Vector3d moveNext();

    // set which node sim robot is actually at
    void setRobotNode(int node_id_prior);

    // get observation at current node
    vector<Eigen::Vector3d> getObservation(int node_id);

    // get visited node position in visited frame
    vector<Eigen::Vector3d> getVisitedNodePosition();

    // get actual path nodes
    vector<int> getActualPath();
    vector<int> getActualPathVisited();
    vector<int> getPredefPath();

    // compare estimation path with gt path
    double compareEstPathWithGT(Eigen::VectorXi estpath);

    // get sim visited graph. return current id in visited graph
    int getSimVisitedGraph(TopoMetric_c &graph);

    // get current position in visited frame
    Eigen::Vector3d getCurrentPos()
    {return _simenv_graph.getNode(_current_actual_node_id)->xyz - _simenv_graph.getNode(_start_node_id)->xyz;}

    TopoMetric_c _simenv_graph;
    TopoMetric_c _visited_graph;

    Eigen::VectorXi _is_node_visited;

    int _current_actual_node_id;

    int _start_node_id;

    int _path_id;

    vector<int> _actual_path;

    vector<int> _sim_path_to_go;

    Eigen::VectorXi _id_prior_to_visted;

    double _visited_graph_covariance;
};


#endif
