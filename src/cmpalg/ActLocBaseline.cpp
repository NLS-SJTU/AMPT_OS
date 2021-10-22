#include "ActLocBaseline.h"

void ActLocBaseline::init(TopoMetric_c* prior_graph, double first_action)
{
    _prior_graph = prior_graph;
    srand((unsigned)time(NULL));
    cout<<"randomeseed "<<(unsigned)time(NULL)<<endl;
    if(first_action < 0){
        double theta = M_PI * (rand() % 36) / 18;
        _first_action = Eigen::Vector3d(cos(theta),sin(theta),0);
    }
    else{
        _first_action = Eigen::Vector3d(cos(first_action),sin(first_action),0);
    }
}


Eigen::Vector3d ActLocBaseline::randomAction(vector<Eigen::Vector3d> obs)
{
    int chosen_dir = rand() % obs.size();
    Eigen::Vector3d action = obs[chosen_dir];
    cout<<"[randomact]choose dir "<<chosen_dir<<" in "<<obs.size()<<" directions"<<endl;
    return action;
}

Eigen::Vector3d ActLocBaseline::furthestAction(Eigen::Vector3d current_pose_visited)
{
    if(current_pose_visited.norm() < 0.1){
        return _first_action;
    }
    return current_pose_visited;
}
