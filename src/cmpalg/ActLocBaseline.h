#ifndef ACTLOCBASELINE_H
#define ACTLOCBASELINE_H

#include "../TopoMetric.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>

using namespace std;

class ActLocBaseline
{
public:
    ActLocBaseline(){}
    ~ActLocBaseline(){}

    void init(TopoMetric_c* prior_graph, double first_action);

    Eigen::Vector3d randomAction(vector<Eigen::Vector3d> obs);

    Eigen::Vector3d furthestAction(Eigen::Vector3d current_pose_visited);

    TopoMetric_c* _prior_graph;
    Eigen::Vector3d _first_action;
};


#endif
