#include "ActLocOverlay.h"



void ActLocOverlay::overlayMap(vector<int> _node_id_prior_vec, vector<Eigen::Quaterniond> _q_vec, vector<Eigen::Vector3d> _t_vec)
{
    overlay_graph.clear();
    num_of_exist_estimation.setOnes(prior_graph.size());
    max_overlay = 1;

    // use the first estimation as base map, the base map is in visited graph frame
    overlay_graph = prior_graph;
    overlay_graph.transform(_q_vec[0], _t_vec[0]);
    start_id = _node_id_prior_vec[0];
    qr = _q_vec[0];
    tr = _t_vec[0];

    // slightly different from the original paper
    // we use a vector to store how many estimations are in this node,
    // the less estimations in a node, the more informative
    for(int i=1; i<_node_id_prior_vec.size(); ++i){
        TopoMetric_c tmpgraph = prior_graph;
        tmpgraph.transform(_q_vec[i], _t_vec[i]);
        for(int j=0; j<tmpgraph.size(); ++j){
            double nearest_dist;
            int nearest_node_id = overlay_graph.searchNearestNode(tmpgraph.getNode(j)->xyz, nearest_dist);
            if(nearest_dist < 0.5){
                num_of_exist_estimation(nearest_node_id) += 1;
                if(num_of_exist_estimation(nearest_node_id) > max_overlay){
                    max_overlay = num_of_exist_estimation(nearest_node_id);
                }
            }
        }
    }
    cout<<"number of top estimations:"<<_node_id_prior_vec.size()<<", overlay nodes:"<<num_of_exist_estimation<<endl;
}

Eigen::Vector3d ActLocOverlay::action()
{
    vector<int> node_id_vec;
    overlay_graph.Dijkstra(start_id, node_id_vec);
    double maxdist = overlay_graph.dist_to_start(overlay_graph.size()-1);
    Eigen::VectorXd score;
    score.setZero(prior_graph.size());
    double alpha = 0.5;
    for(int i=0; i<prior_graph.size(); ++i)
    {
        double dist_to_start = overlay_graph.dist_to_start(i);
        if(dist_to_start < 0.01) continue;
        // distance is the shorter the better, discarded nodes are the fewer the better
        score(i) = alpha * (max_overlay - num_of_exist_estimation(i)) / max_overlay + (1 - alpha) * (1 - dist_to_start / maxdist);
    }
    Eigen::Index node_of_maxscore;
    score.maxCoeff(&node_of_maxscore);
    vector<int> path_out_puzzle_zone;
    overlay_graph.getPathFromDijkstra(node_of_maxscore, path_out_puzzle_zone);
    int id_tar_prior = path_out_puzzle_zone[path_out_puzzle_zone.size()-2];
    Eigen::Vector3d action = qr.inverse() * (prior_graph.getNode(id_tar_prior)->xyz - tr);
    return action;
}
