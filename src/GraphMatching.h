
/// p_prior = q_prior_visited * p_visited + t_prior_visited

#ifndef GRAPHMATCHING_H
#define GRAPHMATCHING_H

#include "TopoMetric.h"

#include <iostream>
#include <fstream>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <opencv2/core/core.hpp>

using namespace std;

template <typename GraphT, typename NodeT>
class GraphMatching
{
public:
    GraphMatching();
    ~GraphMatching(){}
    // read parameter,  should be done after prior graph is set
    void readParam(string paramfile);

    // init method before any matching
    // tm1 is prior graph, tm2 is visited graph
    void initialize(GraphT* tm1);
    void setGraph2(GraphT* tm2){visited_graph = tm2;}

    // match tm1 and tm1 to find the indistinguishable subgraph
    void matchSelf();

    // calculate the matching of two topometric graph tm1 and tm2 using vote tensor
    void matchTwoTopoMetric(GraphT* tm2);

    // match by first-order approach
    void matchTwoTopoMetricFirstOrder(GraphT* tm2);

    // match by second-order approach
    void matchTwoTopoMetricSecondOrder(GraphT* tm2);

    // calculate the matching of two topometric graph tm1 and tm2 assuming one match(known start node)
    void matchTwoTopoMetricWithFixTrans(GraphT *tm2, Eigen::Vector3d t_prior_visited);

    // match two graph with only one node and its direction to neibor node assuming known start node
    void initMatchWithFixTrans(GraphT *tm2, Eigen::Vector3d t_prior_visited);

    // match tm2 with only one node and its direction, the id of the only one node should be 0
    void initMatch(GraphT *tm2);

    // get probability of current state(at which node)
    vector<double> getProbOfAllStates(){return prob_vec;}

    // calculate prob of all nodes under this observation (for viterbi loc)
    Eigen::VectorXd calProbOfObs(vector<Eigen::Vector3d> obs);

    // input: id of a node in prior graph
    // return: corresponding xyz position in visited frame with the best match
    Eigen::Vector3d getCoordInVisitedFromIDPrior(int id_prior);

    // (should be used after matching)
    int getIDPriorFromIDVisited(int id_visited);

    // recover prior path
    Eigen::VectorXi recoverPriorPath(vector<int> actual_path_visited);

    // calculate yaw (-pi, pi) from q
    double calYaw(Eigen::Quaterniond _q);

    // calculate err of two yaw (-pi, pi), reurn (-pi, pi)
    double errYaw(double yaw1, double yaw2);

    // get puzzle zone id by node id; choose_graph=0, node is in prior graph, =1, in visited graph
    //int getPuzzleZoneID(int node_id, int choose_graph);
    bool inTheSamePuzzleZone(vector<int> node_id, int choose_graph, vector<int> &id_puzzle_zone_vec);
    // judge a node is in the puzzle zone
    bool inThisPuzzleZone(int node_id, int choose_graph, int id_puzzle_zone);

    //
    void getBestTpriorvisited(Eigen::Vector3d &t_prior_visited, Eigen::Quaterniond &q_prior_visited);

    //
    Eigen::VectorXi getBestMatchRes();

    //
    void saveBestMatchRes(string fname);

    // save sim_mat to file
    void saveSimMat(Eigen::MatrixXd simmat, string filename);

    vector<int> getPuzzleZoneNodes(int id_puzzle_zone);

    // just for testing some function
    void test();

    // calculate affinity of two nodes
//    double calAffinityOfTwoNode(NodeT* nc1, NodeT* nc2);

    // read puzzle zone from file because calculating it before navigation is too long somtimes
    void readPuzzleZone(const string &filepath);

    // clear some storage
    void clear();

    // calculate the whole vote tensor
    void calculateVoteTensor();

    // calculate the whole vote tensor by center node matching first instead of t
    void calculateVoteTensorNodeTow();

    // calculate prior node to node similarity mat
    void calSelfNodeSimMat();

    // calculate the value of one element in the vote tensor with unknow q_1_2
    void calculateValueOfVoteTensor(Eigen::Vector3d t_prior_visited);
    // calculate center similarity first for q_1_2, this can not be used in self match
    void calculateValueOfVoteTensorZeroFirst(Eigen::Vector3d t_prior_visited);
    // calculate vote tensor with known q_1_2
    void calculateValueOfVoteTensor(Eigen::Quaterniond q_1_2, Eigen::Vector3d t_prior_visited);

    // calculate similarity with q and t
    // p1 = q * p2 + t
    double calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi& retmatch);

    // match two node (nc2 is near 0) with only t, it will return q
    double matchTwoNodeAtZero(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond& q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi& retmatch);

    // match two node directly to get q and t
    double matchTwoNodeDirectly(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond& q_1_2, Eigen::Vector3d& t_1_2, Eigen::VectorXi& retmatch);

    // store vote match from only one node match(for init match)
    void storeResultFromOneNodeMatch(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi best_retmatch_node, double max_match_score);

    // calculate distance of two points
    double distanceOfTwoPoint(Eigen::Vector3d p1, Eigen::Vector3d p2);

    // calculate error of two points with direction and distance
    double errorOfTwoPoint(Eigen::Vector3d ep1, Eigen::Vector3d ep2);

    // get to the nearest node in visited graph which has a matching node in prior graph
    double getNearestNodeInVisitedWithMatchingInPrior(int id_visited, int id_neibor_visited, int& id_prior, string keystr);

    // check whether the two vote tensor is the same matching
    bool checkTheSameMatchResult(Eigen::Vector3i index0, Eigen::Vector3i index1);

    // find puzzle zones from the result of matchSelf()
    void findPuzzleZone(vector<Eigen::Vector3i> _index_of_votetensor);

    // remove puzzles that does not appear in both vector when find puzzle zone
    void removeErrPuzzleZone(vector<vector<int> > &zone_vec, vector<vector<int> > &match_zone_vec);

    // get best match
    // return a new err/sim matrix which error match is removed
    Eigen::MatrixXd getBestMatchFromErrMat(Eigen::MatrixXd err_mat, Eigen::VectorXi& retmatch);
    Eigen::MatrixXd getBestMatchFromSimMat(Eigen::MatrixXd sim_mat, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse);
    Eigen::MatrixXd getBestMatchFromSimTensor(Eigen::MatrixXd sim_ten, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse);

    // sinkhorn
    Eigen::MatrixXd sinkhorn(Eigen::MatrixXd sim_mat);

    // get best n result
    void getBestResultFromVoteTensor(Eigen::Tensor<double, 3> _vote_tensor, int topN, vector<Eigen::Vector3i>& _index_of_votetensor);

    void getTopSimilarEstimation(int _id_visited, vector<int>& _node_id_vec, vector<Eigen::Quaterniond>& _q_vec, vector<Eigen::Vector3d>& _t_vec);

    // cal prob with simmat and match result
//    double calProbWithSimmatMatch(Eigen::MatrixXd sim_mat, Eigen::VectorXd retmatch);

    //turn int a,b,c into a_b_c all abc are with 5 string number
    string turntoString(int a, int b, int c);
    //turn a,b,c into t_prior_visited
    Eigen::Vector3d turntot(int a, int b, int c);
    //turn t_prior_visited into a,b,c
    void turntoabc(Eigen::Vector3d t_prior_visited, int& a, int& b, int& c);

    // show result
    void showResult(vector<Eigen::Vector3i> _index_of_votetensor);

    // judge whether the same zone is already added
    bool existSameZone(vector<vector<int> > zone_vec, vector<vector<int> > match_zone_vec, vector<int> nodes, vector<int> match_nodes);

    //graphs
    GraphT *prior_graph, *visited_graph;

    //vote related
    //tensor for storing vote value
    Eigen::Tensor<double, 3> vote_tensor;
//    Eigen::Tensor<double, 3> vote_tensor_prob;
    //a storage for matched pairs in this vote, '' is from prior to visited, 'inverse' is from visited to prior
    map<string, Eigen::VectorXi> vote_match_map;
    map<string, Eigen::VectorXi> vote_match_inverse_map;
    //a storage for q_prior_visited and t_prior_visited in this vote
    map<string, Eigen::Quaterniond> vote_qpv_map;
//    map<string, Eigen::Vector3d> vote_tpv_map;
    //
    int voteT_len_x, voteT_len_y, voteT_len_z;
    // store index of best match results in vote tensor
    vector<Eigen::Vector3i> index_of_votetensor;
    // this defines the t_prior_visited of (0,0,0) of vote tensor
    Eigen::Vector3d start_of_vote_tensor;

    // store affinity matrix with size N(tm1)*N(tm2), N() is the number of nodes
//    Eigen::MatrixXd affinity_mat;

    // map from probability vector to states
    map<int, string> probid_state_map;
    map<string, int> state_probid_map;

    // puzzle zone
    // fast from node id to puzzle id in puzzle_zone_vec
    vector<vector<int> > node_to_puzzle_zone;
    // store puzzle zone
    vector<vector<int> > puzzle_zone_vec;
    vector<vector<int> > cores_puzzle_zone_vec;
    // have match self?
    bool self_match_done;

    // probability of current state
    vector<double> prob_vec;

    // store a best q of last match in case no good q is generate at this time
    Eigen::Quaterniond q_prior_visited_last;
    // 
    bool is_last_q_lock;

    Eigen::MatrixXd best_sim_mat;

    //
    bool is_matched;

    //
    bool using_selfmatch;

    // highest score in all matches
    double best_score_of_all;

    // parameters
    string parameter_filepath;
    double vote_resolution;
    double vote_threshold;
    double vote_threshold_selfmatch;
    double scale_factor;
    double dist_factor;
    double max_sinkhorn_iter;
    double permitted_center_dist;
    double permitted_center_dist_rate;
    double err_threshold;
    double err_factor_dist;
    double err_factor_cos;
    double permitted_rotation;
    double permitted_rotation_norm_factor;
    string graph_path;
    int max_show_result;
};


template <typename GraphT, typename NodeT>
GraphMatching<GraphT, NodeT>::GraphMatching():
    is_matched(false), is_last_q_lock(false), self_match_done(false)
  , using_selfmatch(false)
{}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::readParam(string paramfile)
{
    parameter_filepath = paramfile;
    cv::FileStorage fsSettings(parameter_filepath.c_str(), cv::FileStorage::READ);
    fsSettings["ampt"]["graphmatching"]["vote_resolution"] >> vote_resolution;
    fsSettings["ampt"]["graphmatching"]["vote_threshold"] >> vote_threshold;
    fsSettings["ampt"]["graphmatching"]["scale_factor"] >> scale_factor;
    fsSettings["ampt"]["graphmatching"]["dist_factor"] >> dist_factor;
    fsSettings["ampt"]["graphmatching"]["max_sinkhorn_iter"] >> max_sinkhorn_iter;
    fsSettings["ampt"]["graphmatching"]["permitted_center_dist"] >> permitted_center_dist;
    fsSettings["ampt"]["graphmatching"]["permitted_center_dist_rate"] >> permitted_center_dist_rate;
    fsSettings["ampt"]["graphmatching"]["permitted_rotation"] >> permitted_rotation;
    fsSettings["ampt"]["graphmatching"]["permitted_rotation_norm_factor"] >> permitted_rotation_norm_factor;
    fsSettings["ampt"]["graphmatching"]["err_threshold"] >> err_threshold;
    fsSettings["ampt"]["graphmatching"]["err_factor_cos"] >> err_factor_cos;
    fsSettings["ampt"]["graphmatching"]["err_factor_dist"] >> err_factor_dist;
    fsSettings["ampt"]["graphmatching"]["max_show_result"] >> max_show_result;
    fsSettings["ampt"]["graphmatching"]["vote_threshold_selfmatch"] >> vote_threshold_selfmatch;
    fsSettings["ampt"]["graph_path"] >> graph_path;
    cout<<"[GM]graph matching param: "<<vote_resolution<<" "<<vote_threshold<<" "<<scale_factor<<" "<<dist_factor<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::readPuzzleZone(const string& filepath)
{
    string pzfile = filepath+"puzzle_zone.txt";
    ifstream in(pzfile);
    if(!in){
        printf("[GM]no puzzle_zone.txt file!\n");
        return;
    }

    string line;
    boost::char_separator<char> sep(" ");
    vector<int> tmppz, tmpcorpz;
    int pzid=0;
    puzzle_zone_vec.clear();
    cores_puzzle_zone_vec.clear();
    node_to_puzzle_zone.clear();
    node_to_puzzle_zone.resize(prior_graph->size());
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        int size_puzzle_zone = (tokens.size() - 1) / 2,
            cores_node_ind = (tokens.size() + 1) / 2;
        tmppz.clear();
        tmpcorpz.clear();
        cout<<"[GM]read puzzle zone "<<pzid<<endl;
        for(int i=0; i<size_puzzle_zone; ++i)
        {
            int a = boost::lexical_cast<int>(tokens[i]),
                b = boost::lexical_cast<float>(tokens[cores_node_ind+i]);
            node_to_puzzle_zone[a].push_back(pzid);
            tmppz.push_back(a);
            tmpcorpz.push_back(b);
            cout<<a<<"-"<<b<<" ";
        }
        puzzle_zone_vec.push_back(tmppz);
        cores_puzzle_zone_vec.push_back(tmpcorpz);
        ++pzid;
        cout<<endl;
    }
    cout<<"[GM]read puzzle done!"<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::initialize(GraphT* tm1)
{
    prior_graph = tm1;
    start_of_vote_tensor << prior_graph->x_lowb, prior_graph->y_lowb, prior_graph->z_lowb;
    voteT_len_x = int((prior_graph->x_upb - prior_graph->x_lowb) / vote_resolution)+1;
    voteT_len_y = int((prior_graph->y_upb - prior_graph->y_lowb) / vote_resolution)+1;
    voteT_len_z = int((prior_graph->z_upb - prior_graph->z_lowb) / vote_resolution)+1;
    vote_tensor = Eigen::Tensor<double, 3>(voteT_len_x, voteT_len_y, voteT_len_z);
//    vote_tensor_prob = Eigen::Tensor<double, 3>(voteT_len_x, voteT_len_y, voteT_len_z);
    vote_tensor.setZero();
//    vote_tensor_prob.setZero();
    cout<<"[GM]vote tensor size: "<<voteT_len_x<<", "<<voteT_len_y<<", "<<voteT_len_z
    <<". vote [0,0,0] is at ("<<start_of_vote_tensor(0)<<", "<<start_of_vote_tensor(1)<<", "
    <<start_of_vote_tensor(2)<<")"<<endl;

    cv::FileStorage fsSettings(parameter_filepath.c_str(), cv::FileStorage::READ);
    string graph_path;
    fsSettings["ampt"]["prior_graph"] >> graph_path;
    readPuzzleZone(graph_path);
    cout<<"[GM]initialize done!"<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::clear()
{
    vote_match_map.clear();
    vote_qpv_map.clear();
    index_of_votetensor.clear();
    vote_tensor.setZero();
//    vote_tensor_prob.setZero();
    is_matched = false;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchTwoTopoMetric(GraphT* tm2)
{
    cout<<"[GM]match"<<endl;
    visited_graph = tm2;
    clear();
//    double larger_edge_of_graph;
//    cout<<visited_graph->x_upb<<" "<<visited_graph->x_lowb<<" "<<
//          visited_graph->y_upb<<" "<<visited_graph->y_lowb<<endl;
//    if((visited_graph->x_upb - visited_graph->x_lowb) > (visited_graph->y_upb - visited_graph->y_lowb)){
//        larger_edge_of_graph = visited_graph->x_upb - visited_graph->x_lowb;
//    }
//    else{
//        larger_edge_of_graph = visited_graph->y_upb - visited_graph->y_lowb;
//    }
//    start_of_vote_tensor(0) = prior_graph->x_lowb - larger_edge_of_graph;
//    start_of_vote_tensor(1) = prior_graph->y_lowb - larger_edge_of_graph;
//    start_of_vote_tensor(2) = prior_graph->z_lowb - prior_graph->z_upb;
//    voteT_len_x = int((prior_graph->x_upb - prior_graph->x_lowb + 2*larger_edge_of_graph) / vote_resolution)+1;
//    voteT_len_y = int((prior_graph->y_upb - prior_graph->y_lowb + 2*larger_edge_of_graph) / vote_resolution)+1;
//    voteT_len_z = int(2*(prior_graph->z_upb - prior_graph->z_lowb) / vote_resolution)+1;
//    cout<<"111"<<voteT_len_x<<" "<<voteT_len_y<<" "<<voteT_len_z<<endl;
//    vote_tensor = Eigen::Tensor<double, 3>(voteT_len_x, voteT_len_y, voteT_len_z);
//    vote_tensor.setZero();
//    cout<<"vote tensor ("<<voteT_len_x<<", "<<voteT_len_y<<", "<<voteT_len_z<<")"<<endl;

    // this is a faster computation method
    calculateVoteTensor();
    // this is the original computation method
//    calculateVoteTensorNodeTow();

    //get posible votes
    //test: get best n result
    getBestResultFromVoteTensor(vote_tensor, 10, index_of_votetensor);
    showResult(index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchSelf()
{
    cout<<"[GM]match self"<<endl;
    visited_graph = prior_graph;
    clear();
    // change some parameter to self match mode
    double tmppermitted_center_dist = permitted_center_dist,
            tmppermitted_center_dist_rate = permitted_center_dist_rate,
            tmpvote_threshold = vote_threshold,
            larger_edge_of_graph;
    if((prior_graph->x_upb - prior_graph->x_lowb) > (prior_graph->y_upb - prior_graph->y_lowb)){
        larger_edge_of_graph = prior_graph->x_upb - prior_graph->x_lowb;
    }
    else{
        larger_edge_of_graph = prior_graph->y_upb - prior_graph->y_lowb;
    }
    start_of_vote_tensor(0) = prior_graph->x_lowb - larger_edge_of_graph;
    start_of_vote_tensor(1) = prior_graph->y_lowb - larger_edge_of_graph;
    start_of_vote_tensor(2) = prior_graph->z_lowb - prior_graph->z_upb;
    voteT_len_x = int((prior_graph->x_upb - prior_graph->x_lowb + 2*larger_edge_of_graph) / vote_resolution)+1;
    voteT_len_y = int((prior_graph->y_upb - prior_graph->y_lowb + 2*larger_edge_of_graph) / vote_resolution)+1;
    voteT_len_z = int(2*(prior_graph->z_upb - prior_graph->z_lowb) / vote_resolution)+1;
    vote_tensor = Eigen::Tensor<double, 3>(voteT_len_x, voteT_len_y, voteT_len_z);
    vote_tensor.setZero();
    permitted_center_dist = vote_resolution/2;
    permitted_center_dist_rate = 0.0;
    vote_threshold = vote_threshold_selfmatch;
    cout<<"vote tensor ("<<voteT_len_x<<", "<<voteT_len_y<<", "<<voteT_len_z<<")"<<endl;

    using_selfmatch = true;
    calculateVoteTensorNodeTow();
    calSelfNodeSimMat();
    using_selfmatch = false;

    getBestResultFromVoteTensor(vote_tensor, -1, index_of_votetensor);
    showResult(index_of_votetensor);

    // find similar zone by finding neibor nodes which are not in the similar zone
    findPuzzleZone(index_of_votetensor);
    self_match_done = true;

    // change parameter back
    permitted_center_dist = tmppermitted_center_dist;
    permitted_center_dist_rate = tmppermitted_center_dist_rate;
    vote_threshold = tmpvote_threshold;
    initialize(prior_graph);
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::initMatchWithFixTrans(GraphT *tm2, Eigen::Vector3d t_prior_visited)
{
    cout<<"[GM]init match with fix trans"<<endl;
    visited_graph = tm2;
    clear();
    int id_prior = prior_graph->searchNearestNode(t_prior_visited);
    NodeT *nc1 = &prior_graph->nodes_vec[id_prior]
        , *nc2 = &visited_graph->nodes_vec[visited_graph->id_near_zero_node];
    Eigen::Quaterniond q_1_2;
    Eigen::VectorXi best_retmatch_node;
    double max_match_score = matchTwoNodeAtZero(nc1, nc2, q_1_2, t_prior_visited, best_retmatch_node);
    storeResultFromOneNodeMatch(nc1, nc2, q_1_2, t_prior_visited, best_retmatch_node, max_match_score);
    // because only one match is done, all match res come from this best_retmatch_node
    
    q_prior_visited_last = q_1_2;
    is_last_q_lock = true;

    getBestResultFromVoteTensor(vote_tensor, 5, index_of_votetensor);
    showResult(index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::initMatch(GraphT *tm2)
{
    cout<<"[GM]init match"<<endl;
    visited_graph = tm2;
    clear();
    NodeT *nc1, *nc2 = &visited_graph->nodes_vec[visited_graph->id_near_zero_node];
    Eigen::Quaterniond q_1_2;
    Eigen::VectorXi best_retmatch_node;
    for(int i=0; i<prior_graph->size(); ++i)
    {
        // cout<<"match prior "<<i<<endl;
        nc1 = &prior_graph->nodes_vec[i];
        Eigen::Vector3d t_prior_visited = prior_graph->nodes_vec[i].xyz;
        double max_match_score = matchTwoNodeAtZero(nc1, nc2, q_1_2, t_prior_visited, best_retmatch_node);
        storeResultFromOneNodeMatch(nc1, nc2, q_1_2, t_prior_visited, best_retmatch_node, max_match_score);
    }

    getBestResultFromVoteTensor(vote_tensor, 10, index_of_votetensor);
    showResult(index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchTwoTopoMetricSecondOrder(GraphT* tm2)
{
    cout<<"[GM]match with second order method"<<endl;
    visited_graph = tm2;
    clear();

    Eigen::Vector3d t_prior_visited;
    Eigen::Quaterniond q_prior_visited;
    Eigen::VectorXi retmatch, retmatchinverse;
    int simmat_size = prior_graph->size() * visited_graph->size_confirm_nodes;
    Eigen::MatrixXd sim_ten = Eigen::MatrixXd::Zero(simmat_size, simmat_size);
    // compute affinity tensor by node pair
    for(int id1_p=0; id1_p<prior_graph->size(); ++id1_p){
        NodeT* nc1_p = prior_graph->getNode(id1_p);
        for(int id1_v=0; id1_v<visited_graph->size_confirm_nodes; ++id1_v){
            NodeT* nc1_v = visited_graph->getNode(id1_v);
            double sim1pv = matchTwoNodeDirectly(nc1_p, nc1_v,
                        q_prior_visited, t_prior_visited, retmatch);
            int ind1 = id1_p * visited_graph->size_confirm_nodes + id1_v;
            for(int id1_neighbor_p=0; id1_neighbor_p<prior_graph->nodes_vec[id1_p].id_neibor.size(); ++id1_neighbor_p){
                int id2_p = prior_graph->nodes_vec[id1_p].id_neibor[id1_neighbor_p];
                if(id2_p < id1_p) continue;
                NodeT* nc2_p = prior_graph->getNode(id2_p);
                for(int id1_neighbor_v=0; id1_neighbor_v<visited_graph->nodes_vec[id1_v].id_neibor.size(); ++id1_neighbor_v){
                    int id2_v = visited_graph->nodes_vec[id1_v].id_neibor[id1_neighbor_v];
                    if(id2_v < id1_v) continue;
                    if(id2_v >= visited_graph->size_confirm_nodes) continue;
                    NodeT* nc2_v = visited_graph->getNode(id2_v);
                    double sim2pv = matchTwoNodeDirectly(nc2_p, nc2_v,
                                q_prior_visited, t_prior_visited, retmatch);
                    int ind2 = id2_p * visited_graph->size_confirm_nodes + id2_v;
                    sim_ten(ind1, ind2) = (sim1pv + sim2pv) / 2;
                }
            }
        }
    }
    //get best matching from affinity tensor
    retmatch.resize(prior_graph->size());
    retmatchinverse.resize(visited_graph->size_confirm_nodes);
    Eigen::MatrixXd matchmat = getBestMatchFromSimTensor(sim_ten, retmatch, retmatchinverse);
    int i_x=0, i_y=0, i_z=0;
    string keystr = turntoString(i_x, i_y, i_z);
    vote_tensor(i_x, i_y, i_z) = matchmat.sum();
    vote_match_map[keystr] = retmatch;
    vote_match_inverse_map[keystr] = retmatchinverse;
    vote_qpv_map[keystr] = q_prior_visited; // this q and t are of no use
    getBestResultFromVoteTensor(vote_tensor, 10, index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchTwoTopoMetricFirstOrder(GraphT* tm2)
{
    cout<<"[GM]match with first order method"<<endl;
    visited_graph = tm2;
    clear();

    Eigen::Vector3d t_prior_visited;
    Eigen::Quaterniond q_prior_visited;
    Eigen::VectorXi retmatch, retmatchinverse;
    Eigen::MatrixXd sim_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size_confirm_nodes);
    cout<<"rows:"<<sim_mat.rows()<<",cols:"<<sim_mat.cols()<<endl;
    for(int id1=0; id1<prior_graph->size(); ++id1)
    {
        NodeT* nc1 = prior_graph->getNode(id1);
        for(int id2=0; id2<visited_graph->size_confirm_nodes; ++id2)
        {
            NodeT* nc2 = visited_graph->getNode(id2);
            sim_mat(id1,id2) = matchTwoNodeDirectly(nc1, nc2,
                      q_prior_visited, t_prior_visited, retmatch);
        }
    }
    saveSimMat(sim_mat, "Gloc_tra");
    retmatch.resize(prior_graph->size());
    retmatchinverse.resize(visited_graph->size_confirm_nodes);
    Eigen::MatrixXd matchmat = getBestMatchFromSimMat(sim_mat, retmatch, retmatchinverse);
    // just use 0,0,0 as index
    int i_x=0, i_y=0, i_z=0;
    string keystr = turntoString(i_x, i_y, i_z);
    vote_tensor(i_x, i_y, i_z) = matchmat.sum();
    vote_match_map[keystr] = retmatch;
    vote_match_inverse_map[keystr] = retmatchinverse;
    vote_qpv_map[keystr] = q_prior_visited; // this q and t are of no use
    getBestResultFromVoteTensor(vote_tensor, 10, index_of_votetensor);
//    showResult(index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchTwoTopoMetricWithFixTrans(GraphT *tm2, Eigen::Vector3d t_prior_visited)
{
    cout<<"[GM]match with fix trans"<<endl;
    visited_graph = tm2;
    clear();
    calculateValueOfVoteTensorZeroFirst(t_prior_visited);

    //get posible votes
    //test: get best n result
    getBestResultFromVoteTensor(vote_tensor, 5, index_of_votetensor);
    showResult(index_of_votetensor);
    is_matched = true;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateVoteTensorNodeTow()
{
    Eigen::Vector3d t_prior_visited;
    Eigen::Quaterniond q_prior_visited;
    Eigen::VectorXi retmatch;
    double score;
    for(int i_1=0; i_1<prior_graph->size(); ++i_1)
    {
        for(int i_2=0; i_2<visited_graph->size(); ++i_2)
        {
            // match two node to get q and t
            score = matchTwoNodeDirectly(prior_graph->getNode(i_1), visited_graph->getNode(i_2),
                                 q_prior_visited, t_prior_visited, retmatch);
            cout<<i_1<<", "<<i_2<<", score"<<endl;
            if(score > vote_threshold){
                calculateValueOfVoteTensor(q_prior_visited, t_prior_visited);
            }
        }
    }
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateVoteTensor()
{
    best_score_of_all = 0;
    Eigen::Vector3d t_prior_visited;
    for(int i_x=0; i_x<voteT_len_x; ++i_x)
    {
        t_prior_visited(0) =  vote_resolution * i_x + start_of_vote_tensor(0);//prior_graph->x_lowb;
        for(int i_y=0; i_y<voteT_len_y; ++i_y)
        {
            t_prior_visited(1) =  vote_resolution * i_y + start_of_vote_tensor(1);//prior_graph->y_lowb;
            for(int i_z=0; i_z<voteT_len_z; ++i_z)
            {
                // cout<<"-------------------------------\\"<<endl<<"vote grid:"<<i_x<<", "<<i_y<<", "<<i_z<<endl;
//                Eigen::MatrixXd tmpSim_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size());
                t_prior_visited(2) =  vote_resolution * i_z + start_of_vote_tensor(2);//prior_graph->z_lowb;
                if(using_selfmatch){
                    calculateValueOfVoteTensor(t_prior_visited);
                }
                else{
                    calculateValueOfVoteTensorZeroFirst(t_prior_visited);
                }
            }
        }
    }
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateValueOfVoteTensor(Eigen::Quaterniond q_1_2, Eigen::Vector3d t_prior_visited)
{
    int i_x, i_y, i_z;
    turntoabc(t_prior_visited, i_x, i_y, i_z);
//    cout<<"index vote tensor: "<<i_x<<", "<<i_y<<","<<i_z<<endl;
    string keystr = turntoString(i_x, i_y, i_z);
    Eigen::Vector3d p_prior, p_visited;
    Eigen::Vector3d p_visited_tran;
    Eigen::MatrixXd tmpSim_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size());
    Eigen::Quaterniond q_prior_visited, q_prior_visited_fix;
    double best_sim_score = 0;
    int best_match_visited_id = -1;
    Eigen::VectorXi retmatch_node;
    double last_lock_yaw;
    double simscore = 0;

    q_prior_visited_fix = q_1_2;
    last_lock_yaw = calYaw(q_prior_visited_fix);

    // match all nodes
    for(int i_pri=0; i_pri<prior_graph->size(); ++i_pri)
    {
        p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;

        for(int i_visi=0; i_visi<visited_graph->size_confirm_nodes; ++i_visi)
        {
            simscore = 0;
            p_visited = visited_graph->getNode(i_visi)->xyz;
            if(i_visi == visited_graph->id_near_zero_node){
                q_prior_visited = q_prior_visited_fix;
            }
            else{
                q_prior_visited = Eigen::Quaterniond::FromTwoVectors(p_visited, p_prior);
            }
            p_visited_tran = q_prior_visited * p_visited;
            float center_dist = (p_prior - p_visited_tran).norm();
            float center_dist_rate = center_dist / (visited_graph->getNode(i_visi)->norm);
            if(center_dist < permitted_center_dist || center_dist_rate < permitted_center_dist_rate){
                double thisyaw = calYaw(q_prior_visited);
                double erryaw = thisyaw - last_lock_yaw;
                if(erryaw > M_PI){
                    erryaw -= M_PI*2;
                }
                else if(erryaw < -M_PI){
                    erryaw += M_PI*2;
                }

                if(i_visi == visited_graph->id_near_zero_node){
                    simscore = calSimilarityOfTwoNode(
                        prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                      , q_prior_visited, t_prior_visited, retmatch_node);
                }
                else if(fabs(erryaw) < permitted_rotation*exp(-visited_graph->getNode(i_visi)->norm/permitted_rotation_norm_factor)){
                    // similar to lock yaw
                    simscore = calSimilarityOfTwoNode(
                        prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                      , q_prior_visited, t_prior_visited, retmatch_node) * exp(-center_dist_rate);
                }

                if(simscore > vote_threshold ){
                    tmpSim_mat(i_pri, i_visi) = simscore;//&& simscore > tmpSim_mat(i_pri, i_visi)
                    if(simscore > best_sim_score){
                        q_prior_visited_fix = q_prior_visited;
                        best_sim_score = simscore;
                        best_match_visited_id = i_visi;
                    }
                }
            }
        }
    }
    Eigen::VectorXi retmatch(prior_graph->size())
            , retmatchinverse(visited_graph->size());
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);
    Eigen::MatrixXd matchmat = getBestMatchFromSimMat(tmpSim_mat, retmatch, retmatchinverse);
    simscore = matchmat.sum();
    if(simscore > vote_tensor(i_x, i_y, i_z)){
        vote_tensor(i_x, i_y, i_z) = simscore;
        vote_match_map[keystr] = retmatch;
        vote_match_inverse_map[keystr] = retmatchinverse;
        vote_qpv_map[keystr] = q_prior_visited_fix;
    }
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateValueOfVoteTensorZeroFirst(Eigen::Vector3d t_prior_visited)
{
    int i_x, i_y, i_z;
    turntoabc(t_prior_visited, i_x, i_y, i_z);
    string keystr = turntoString(i_x, i_y, i_z);
    Eigen::Vector3d p_prior, p_visited;
    Eigen::Vector3d p_visited_tran;
    Eigen::MatrixXd tmpSim_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size());
    Eigen::Quaterniond q_prior_visited, q_prior_visited_fix;
    double best_sim_score = 0;
    int best_match_visited_id = -1;
    Eigen::VectorXi retmatch_node;
    double last_lock_yaw;

    double simscore = 0;
    // calculate near zero node first
    int id_prior_near_t = prior_graph->searchNearestNode(t_prior_visited);
    float center_dist = (prior_graph->getNode(id_prior_near_t)->xyz - t_prior_visited).norm();
    if(center_dist > vote_threshold/1.4){
        // zero node is not matched, return
        Eigen::VectorXi retmatch(prior_graph->size())
                , retmatchinverse(visited_graph->size());
        retmatch.setConstant(-1);
        retmatchinverse.setConstant(-1);
        vote_tensor(i_x, i_y, i_z) = 0;
        vote_match_map[keystr] = retmatch;
        vote_match_inverse_map[keystr] = retmatchinverse;
        vote_qpv_map[keystr] = q_prior_visited_fix;
        return;
    }
    // match zero node and fix q
    if(is_last_q_lock){
        last_lock_yaw = calYaw(q_prior_visited_last);
        q_prior_visited_fix = q_prior_visited_last;
        simscore = calSimilarityOfTwoNode(prior_graph->getNode(id_prior_near_t)
          , visited_graph->getNode(visited_graph->id_near_zero_node)
          , q_prior_visited_fix, t_prior_visited, retmatch_node);
    }
    else{
        simscore = matchTwoNodeAtZero(prior_graph->getNode(id_prior_near_t), visited_graph->getNode(visited_graph->id_near_zero_node)
                      , q_prior_visited, t_prior_visited, retmatch_node);
        q_prior_visited_fix = q_prior_visited;
        last_lock_yaw = calYaw(q_prior_visited_fix);
    }
    tmpSim_mat(id_prior_near_t, visited_graph->id_near_zero_node) = simscore;
    // do not put neibor of zero node into sim_mat
    /*for(int i=0; i<prior_graph->getNode(id_prior_near_t)->id_neibor.size(); ++i)
    {
        if(retmatch_node(i) < 0) continue;
        else{
            int id_pri = prior_graph->getNode(id_prior_near_t)->id_neibor[i],
                id_visi = visited_graph->getNode(visited_graph->id_near_zero_node)->id_neibor[retmatch_node(i)];
            tmpSim_mat(id_pri, id_visi) = simscore / prior_graph->getNode(id_prior_near_t)->id_neibor.size();
        }
    }*/

    // match other nodes
    for(int i_pri=0; i_pri<prior_graph->size(); ++i_pri)
    {
        if(i_pri == id_prior_near_t) continue;
        p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;

        for(int i_visi=0; i_visi<visited_graph->size_confirm_nodes; ++i_visi)
        {
            if(i_visi == visited_graph->id_near_zero_node) continue;
            simscore = 0;
            p_visited = visited_graph->getNode(i_visi)->xyz;
            q_prior_visited = Eigen::Quaterniond::FromTwoVectors(p_visited, p_prior);
            p_visited_tran = q_prior_visited * p_visited;
            center_dist = (p_prior - p_visited_tran).norm();
            float center_dist_rate = center_dist / (visited_graph->getNode(i_visi)->norm);
            if(center_dist < permitted_center_dist || center_dist_rate < permitted_center_dist_rate){
                double thisyaw = calYaw(q_prior_visited);
                double erryaw = thisyaw - last_lock_yaw;
                if(erryaw > M_PI){
                    erryaw -= M_PI*2;
                }
                else if(erryaw < -M_PI){
                    erryaw += M_PI*2;
                }
                if(fabs(erryaw) < permitted_rotation*exp(-visited_graph->getNode(i_visi)->norm/permitted_rotation_norm_factor)){
                    // similar to lock yaw
                    simscore = calSimilarityOfTwoNode(
                        prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                      , q_prior_visited, t_prior_visited, retmatch_node) * exp(-center_dist_rate);
                }

                if(simscore > vote_threshold ){
                    tmpSim_mat(i_pri, i_visi) = simscore;//&& simscore > tmpSim_mat(i_pri, i_visi)
                    if(simscore > best_sim_score){
                        q_prior_visited_fix = q_prior_visited;
                        best_sim_score = simscore;
                        best_match_visited_id = i_visi;
                    }
                }
            }
        }
    }
    Eigen::VectorXi retmatch(prior_graph->size())
            , retmatchinverse(visited_graph->size());
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);
    vote_tensor(i_x, i_y, i_z) = getBestMatchFromSimMat(tmpSim_mat, retmatch, retmatchinverse).sum();
    vote_match_map[keystr] = retmatch;
    vote_match_inverse_map[keystr] = retmatchinverse;
    vote_qpv_map[keystr] = q_prior_visited_fix;
    // save simmat
    if(vote_tensor(i_x, i_y, i_z) > best_score_of_all){
        best_score_of_all = vote_tensor(i_x, i_y, i_z);

        best_sim_mat = tmpSim_mat;
        saveSimMat(tmpSim_mat, "Gloc_our");
    }
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::saveBestMatchRes(string fname)
{
    saveSimMat(best_sim_mat, fname);
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::saveSimMat(Eigen::MatrixXd simmat, string filename)
{
    ofstream ost(graph_path+filename+"_simmat.txt");
    ofstream ost2(graph_path+filename+"_sinksimmat.txt");
    Eigen::MatrixXd simmatsink = sinkhorn(simmat);
    for(int i=0; i<prior_graph->size(); ++i)
    {
        for(int j=0; j<visited_graph->size_confirm_nodes; ++j)
        {
            ost << simmat(i,j) <<" ";
            ost2 << simmatsink(i,j) <<" ";
        }
        ost << endl;
        ost2 << endl;
    }
    ost.close();
    ost2.close();
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateValueOfVoteTensor(Eigen::Vector3d t_prior_visited)
{
    int i_x, i_y, i_z;
    turntoabc(t_prior_visited, i_x, i_y, i_z);
    string keystr = turntoString(i_x, i_y, i_z);
    Eigen::Vector3d p_prior, p_visited;
    Eigen::Vector3d p_visited_tran;
    Eigen::MatrixXd tmpSim_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size());
    Eigen::Quaterniond q_prior_visited, q_prior_visited_fix;
    double best_sim_score = 0;
    vector<int> nodezero_vec;
    bool record_nodezero_flag = false;
    int best_match_visited_id = -1;
    Eigen::VectorXi retmatch_node;
    double last_lock_yaw;
    if(is_last_q_lock){
        last_lock_yaw = calYaw(q_prior_visited_last);
        q_prior_visited_fix = q_prior_visited_last;
    }
    for(int i_pri=0; i_pri<prior_graph->size(); ++i_pri)
    {
        p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;
        for(int i_visi=0; i_visi<visited_graph->size_confirm_nodes; ++i_visi)
        {
            double simscore = 0;
            if(visited_graph->getNode(i_visi)->norm > 1){
                p_visited = visited_graph->getNode(i_visi)->xyz;
                q_prior_visited = Eigen::Quaterniond::FromTwoVectors(p_visited, p_prior);
                p_visited_tran = q_prior_visited * p_visited;
                float center_dist = (p_prior - p_visited_tran).norm();
                float center_dist_rate = center_dist / (visited_graph->getNode(i_visi)->norm);
                if(center_dist < permitted_center_dist || center_dist_rate < permitted_center_dist_rate){//
                    /*cout<<"id_pri:"<<i_pri<<", id_visi:"<<i_visi<<", centerdist:"<<center_dist<<", center_dist_rate:"<<center_dist_rate
                        <<", q:"<<q_prior_visited.w()<<","<<q_prior_visited.x()<<","
                        <<q_prior_visited.y()<<","<<q_prior_visited.z()
                        <<endl;*/
                    if(is_last_q_lock){
                        double thisyaw = calYaw(q_prior_visited);
                        double erryaw = thisyaw - last_lock_yaw;
                        if(erryaw > M_PI){
                            erryaw -= M_PI*2;
                        }
                        else if(erryaw < -M_PI){
                            erryaw += M_PI*2;
                        }
                        if(fabs(erryaw) < permitted_rotation*exp(-visited_graph->getNode(i_visi)->norm/permitted_rotation_norm_factor)){
                            // similar to last lock yaw
                            simscore = calSimilarityOfTwoNode(
                                prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                              , q_prior_visited, t_prior_visited, retmatch_node) * exp(-center_dist_rate);
                        }
                    }
                    else{
                        simscore = calSimilarityOfTwoNode(
                                prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                              , q_prior_visited, t_prior_visited, retmatch_node) * exp(-center_dist_rate);
                    }
                    
                    /*cout<<"similarity between G1 node "<<i_pri<<" and G2 node "<<i_visi
                        <<" is:"<<simscore<<endl;*/
                    if(simscore > vote_threshold ){
                        tmpSim_mat(i_pri, i_visi) = simscore;//&& simscore > tmpSim_mat(i_pri, i_visi)
                        if(simscore > best_sim_score){
                            q_prior_visited_fix = q_prior_visited;
                            best_sim_score = simscore;
                            best_match_visited_id = i_visi;
                        }
                    }
                }
            }
            else{
                simscore = matchTwoNodeAtZero(prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                              , q_prior_visited, t_prior_visited, retmatch_node);
                if(simscore > vote_threshold ){
                    tmpSim_mat(i_pri, i_visi) = simscore;
                    if(simscore > best_sim_score){
                        q_prior_visited_fix = q_prior_visited;
                        best_sim_score = simscore;
                        best_match_visited_id = i_visi;
                    }
                }
            }
            /*else if(!record_nodezero_flag){
                //p_visited is near 0, only record when first see this node
                nodezero_vec.push_back(i_visi);
            }*/
        }
        record_nodezero_flag = true;
    }
    /// a hiding bug: q_prior_visited_fix might be wrong(to improve)
    //deal with nodes near zero
    for(int i_pri=0; i_pri<prior_graph->size(); ++i_pri)
    {
        p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;
        // all nodes method 1--still distance
        for(int i_visi=0; i_visi<visited_graph->size(); ++i_visi)
        {
            if(tmpSim_mat(i_pri, i_visi) < vote_threshold) continue;
            p_visited = visited_graph->getNode(i_visi)->xyz;
            p_visited_tran = q_prior_visited_fix * p_visited;
            float center_dist = (p_prior - p_visited_tran).norm();
            float center_dist_rate = center_dist / ((visited_graph->getNode(i_visi)->norm)+0.1);
            if(center_dist < permitted_center_dist || center_dist_rate < permitted_center_dist_rate){
                /*cout<<"id_pri:"<<i_pri<<", id_visi:"<<i_visi<<", centerdist:"<<center_dist<<", center_dist_rate:"<<center_dist_rate
                    <<", q:"<<q_prior_visited_fix.w()<<","<<q_prior_visited_fix.x()<<","
                    <<q_prior_visited_fix.y()<<","<<q_prior_visited_fix.z()
                    <<endl;*/
                double simscore = calSimilarityOfTwoNode(
                            prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                          , q_prior_visited_fix, t_prior_visited, retmatch_node) * exp(-center_dist_rate);
                /*cout<<"similarity between G1 node "<<i_pri<<" and G2 node "<<i_visi
                   <<" is:"<<simscore<<endl;*/
                if(simscore > vote_threshold){// && simscore > tmpSim_mat(i_pri, i_visi)
                    tmpSim_mat(i_pri, i_visi) = simscore;
                }
            }
            else{
                tmpSim_mat(i_pri, i_visi) = 0;
            }
        }
    }
    Eigen::VectorXi retmatch(prior_graph->size())
            , retmatchinverse(visited_graph->size());
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);
    vote_tensor(i_x, i_y, i_z) = getBestMatchFromSimMat(tmpSim_mat, retmatch, retmatchinverse).sum();
    vote_match_map[keystr] = retmatch;
    vote_match_inverse_map[keystr] = retmatchinverse;
    vote_qpv_map[keystr] = q_prior_visited_fix;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi& retmatch)
{
    if(nc1->id_neibor.size() == 0){
        return 0.0;
    }
    double ret = 0.0;
    vector<Eigen::Vector3d> xyz_neibor_2_vec;
    Eigen::Vector3d nc2_xyz_rotate = q_1_2 * nc2->xyz + t_1_2, nc_neibor_xyz;
    xyz_neibor_2_vec.resize(nc2->id_neibor.size());
    //transform nc2 points to nc1 frame
    for(int i_2=0; i_2<nc2->id_neibor.size(); ++i_2)
    {
        nc_neibor_xyz = visited_graph->getNode(nc2->id_neibor[i_2])->xyz;
        xyz_neibor_2_vec[i_2] = q_1_2 * nc_neibor_xyz + t_1_2;//nc2->node_neibor[i_2]->xyz
    }
//    if(nc1->get_id_self() == 0){
//        cout<<"q:"<<q_1_2.w()<<","<<q_1_2.x()<<","
//           <<q_1_2.y()<<","<<q_1_2.z()<<",t_1_2:"<<t_1_2<<endl;
//    }
    //calculate error of any two neibor nodes of two nodes
    Eigen::MatrixXd err_mat, sim_mat;
    err_mat.resize(nc1->id_neibor.size(), nc2->id_neibor.size());
    err_mat.setConstant(10000);
    sim_mat.resize(nc1->id_neibor.size(), nc2->id_neibor.size());
    sim_mat.setConstant(0);
    for(int i_1=0; i_1<nc1->id_neibor.size(); ++i_1)
    {
        nc_neibor_xyz = prior_graph->getNode(nc1->id_neibor[i_1])->xyz;
        double errnorm = (nc_neibor_xyz - nc1->xyz).norm();//nc1->node_neibor[i_1]->xyz
        for(int i_2=0; i_2<nc2->id_neibor.size(); ++i_2)
        {
            //use distance of two neibor node as error
            //err_mat(i_1, i_2) = distanceOfTwoPoint(nc1->node_neibor[i_1]->xyz, xyz_neibor_2_vec[i_2]) / errnorm;
            //use difference of direction and distance of neibor nodes as error
            err_mat(i_1, i_2) = errorOfTwoPoint(nc_neibor_xyz - nc1->xyz, xyz_neibor_2_vec[i_2] - nc2_xyz_rotate);//nc1->node_neibor[i_1]->xyz
            if(err_mat(i_1, i_2) < err_threshold){
                sim_mat(i_1, i_2) = exp(-err_mat(i_1, i_2));
            }
        }
    }

    //get most near pairs
//    Eigen::VectorXi retmatch(nc1->id_neibor.size());
    retmatch.resize(nc1->id_neibor.size());
    retmatch.setConstant(-1);
//    getBestMatchFromErrMat(err_mat, retmatch); ///maybe here can use similarity mat with sinkhorn
    Eigen::VectorXi tmpretmatch;
    tmpretmatch.resize(nc2->id_neibor.size());
    getBestMatchFromSimMat(sim_mat, retmatch, tmpretmatch);

    //calculate similarity
    for(int i=0; i<retmatch.size(); ++i)
    {
        if(retmatch[i] < 0 || err_mat(i, retmatch[i]) > err_threshold){
            continue;
        }
        else{
            ret += exp(-err_mat(i, retmatch[i]));
            /// maybe gausian model is better?
        }
    }
    int larger_size;
    if(nc1->id_neibor.size() > nc2->id_neibor.size()){
        larger_size = nc1->id_neibor.size();
    }
    else{
        larger_size = nc2->id_neibor.size();
    }
    ret = ret / larger_size;
    return ret;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::storeResultFromOneNodeMatch(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi best_retmatch_node, double max_match_score)
{
    Eigen::VectorXi best_retmatch_inverse(visited_graph->size()),
            best_retmatch(prior_graph->size());
    best_retmatch_inverse.setConstant(-1);
    best_retmatch.setConstant(-1);
    for(int i=0; i<nc1->id_neibor.size(); ++i)
    {
        if(best_retmatch_node(i) < 0) continue;
        else{
            best_retmatch(nc1->id_neibor[i]) = nc2->id_neibor[best_retmatch_node(i)];
            best_retmatch_inverse(nc2->id_neibor[best_retmatch_node(i)]) = nc1->id_neibor[i];
        }
    }
    best_retmatch(nc1->get_id_self()) = nc2->get_id_self();
    best_retmatch_inverse(nc2->get_id_self()) = nc1->get_id_self();
    int i_x, i_y, i_z;
    turntoabc(t_1_2, i_x, i_y, i_z);
    string keystr = turntoString(i_x, i_y, i_z);
    vote_tensor(i_x, i_y, i_z) = max_match_score;
    vote_match_map[keystr] = best_retmatch;
    vote_match_inverse_map[keystr] = best_retmatch_inverse;
    vote_qpv_map[keystr] = q_1_2;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calSelfNodeSimMat()
{
    Eigen::MatrixXd selfsimmat;
    Eigen::Quaterniond tmpq;
    Eigen::Vector3d tmpt;
    Eigen::VectorXi tmpm;
    selfsimmat.resize(prior_graph->size_confirm_nodes,prior_graph->size_confirm_nodes);
    for(int i=0; i<prior_graph->size_confirm_nodes; ++i)
    {
        NodeT* nc1 = prior_graph->getNode(i);
        for(int j=i; j<prior_graph->size_confirm_nodes; ++j)
        {
            NodeT* nc2 = visited_graph->getNode(j);
            double score = matchTwoNodeDirectly(nc1,nc2,tmpq,tmpt,tmpm);
            selfsimmat(i,j) = score;
            selfsimmat(j,i) = score;
        }
    }
    ofstream outfile(graph_path+"selfsimmat.txt");
    for(int i=0; i<prior_graph->size_confirm_nodes; ++i)
    {
        for(int j=0; j<prior_graph->size_confirm_nodes; ++j)
        {
            outfile << selfsimmat(i,j) << " ";
        }
        outfile << endl;
    }
    outfile.close();
}

template <typename GraphT, typename NodeT>
Eigen::VectorXd GraphMatching<GraphT, NodeT>::calProbOfObs(vector<Eigen::Vector3d> obs)
{
    // make a topometric
    GraphT vgraph;
    visited_graph = &vgraph;
    NodeT ntmp(0,0,0,0);
    vgraph.nodes_vec.push_back(ntmp);
    for(int i=0; i<obs.size(); ++i)
    {
        ntmp.setxyzid(obs[i], i+1);
        vgraph.nodes_vec.push_back(ntmp);
        vgraph.getNode(i+1)->addLink(0, obs[i].norm());
        vgraph.getNode(0)->addLink(i+1, obs[i].norm());
    }
    vgraph.showGraphData();

    // cal prob
    Eigen::VectorXd prob_obs;
    prob_obs.resize(prior_graph->size());
    NodeT* nc2 = visited_graph->getNode(0);
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::VectorXi retmatch;
    for(int i=0; i<prior_graph->size(); ++i)
    {
        NodeT* nc1 = prior_graph->getNode(i);
        prob_obs(i) = matchTwoNodeDirectly(nc1, nc2, q,t,retmatch);
    }
    return prob_obs;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::matchTwoNodeDirectly(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond& q_1_2, Eigen::Vector3d& t_1_2, Eigen::VectorXi& retmatch)
{
    if(nc2->id_neibor.size() <= 0){
        return 0;
    }

    vector<Eigen::Vector3d> nc1_neibor_vec, nc2_neibor_vec;
    Eigen::Vector3d nc_neibor_xyz;
    Eigen::Quaterniond q_1_2_tmp;
    nc1_neibor_vec.resize(nc1->id_neibor.size());
    nc2_neibor_vec.resize(nc2->id_neibor.size());
    // transform, the two nodes transformation is (nc1neibor - nc1) = q_1_2 * (nc2neibor - nc2)
    // thus t_1_2 = nc1 - q_1_2 * nc2
    for(int i=0; i<nc1->id_neibor.size(); ++i)
    {
        nc_neibor_xyz = prior_graph->getNode(nc1->id_neibor[i])->xyz;
        nc1_neibor_vec[i] = nc_neibor_xyz - nc1->xyz;
    }
    for(int i=0; i<nc2->id_neibor.size(); ++i)
    {
        nc_neibor_xyz = visited_graph->getNode(nc2->id_neibor[i])->xyz;
        nc2_neibor_vec[i] = nc_neibor_xyz - nc2->xyz;
    }

    // try to match all edges
    double max_match_score = 0, tmpscore = 0;
    Eigen::VectorXi retmatch_node(nc1->id_neibor.size()),
            best_retmatch_node(nc1->id_neibor.size()),
            best_retmatch_inverse(visited_graph->size()),
            best_retmatch(prior_graph->size());
    best_retmatch_inverse.setConstant(-1);
    best_retmatch.setConstant(-1);
    for(int i=0; i<nc1_neibor_vec.size(); ++i)
    {
        for(int j=0; j<nc2_neibor_vec.size(); ++j)
        {
            q_1_2_tmp = Eigen::Quaterniond::FromTwoVectors(nc2_neibor_vec[j], nc1_neibor_vec[i]);
            t_1_2 = nc1->xyz - q_1_2_tmp * nc2->xyz;
            tmpscore = calSimilarityOfTwoNode(nc1, nc2, q_1_2_tmp, t_1_2, retmatch_node);
            if(tmpscore > max_match_score){
                max_match_score = tmpscore;
                q_1_2 = q_1_2_tmp;
                best_retmatch_node = retmatch_node;
            }
        }
    }
    retmatch = best_retmatch_node;
    //calculate t
    t_1_2 = nc1->xyz - q_1_2 * nc2->xyz;
    return max_match_score;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::matchTwoNodeAtZero(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond& q_1_2, Eigen::Vector3d t_1_2, Eigen::VectorXi& retmatch)
{
    if(nc2->id_neibor.size() <= 0){
        return 0;
    }
    Eigen::Vector3d nc1_xyz_trans = nc1->xyz - t_1_2, nc_neibor_xyz; // in fact, this is near zero
    if(nc1_xyz_trans.norm() > permitted_center_dist){
        return 0;
    }
    Eigen::Quaterniond q_1_2_tmp;
    vector<Eigen::Vector3d> nc1_neibor_vec;
    nc1_neibor_vec.resize(nc1->id_neibor.size());
    // transform
    for(int i=0; i<nc1->id_neibor.size(); ++i)
    {
        nc_neibor_xyz = prior_graph->getNode(nc1->id_neibor[i])->xyz;
        nc1_neibor_vec[i] = nc_neibor_xyz - t_1_2;//nc1->node_neibor[i]->xyz
    }
    // try to match all edges
    double max_match_score = 0, tmpscore = 0;
    Eigen::VectorXi retmatch_node(nc1->id_neibor.size()),
            best_retmatch_node(nc1->id_neibor.size()),
            best_retmatch_inverse(visited_graph->size()),
            best_retmatch(prior_graph->size());
    best_retmatch_inverse.setConstant(-1);
    best_retmatch.setConstant(-1);
    for(int i=0; i<nc1->id_neibor.size(); ++i)
    {
        for(int j=0; j<nc2->id_neibor.size(); ++j)
        {
            nc_neibor_xyz = visited_graph->getNode(nc2->id_neibor[j])->xyz;
            q_1_2_tmp = Eigen::Quaterniond::FromTwoVectors(nc_neibor_xyz, nc1_neibor_vec[i]);//nc2->node_neibor[j]->xyz
            tmpscore = calSimilarityOfTwoNode(nc1, nc2, q_1_2_tmp, t_1_2, retmatch_node);
            if(tmpscore > max_match_score){
                max_match_score = tmpscore;
                q_1_2 = q_1_2_tmp;
                best_retmatch_node = retmatch_node;
            }
        }
    }
    retmatch = best_retmatch_node;
    // cout<<"2.5"<<endl<<best_retmatch_node<<endl;
    return max_match_score;
}

template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::getBestMatchFromErrMat(Eigen::MatrixXd err_mat, Eigen::VectorXi& retmatch)
{
    Eigen::MatrixXd::Index minRow, minCol;
    Eigen::MatrixXd new_mat = Eigen::MatrixXd::Zero(err_mat.rows(), err_mat.cols());
    Eigen::VectorXd colmax = Eigen::VectorXd::Constant(err_mat.rows(), DBL_MAX)
            , rowmax = Eigen::VectorXd::Constant(err_mat.cols(), DBL_MAX);
    double minvalue;
    for(int row=0; row<err_mat.rows(); ++row)
    {
        minvalue = err_mat.minCoeff(&minRow, &minCol);
        if(minvalue > 100000000){break;}
        retmatch(minRow) = minCol;
//        cout<<"row:"<<minRow<<", col:"<<minCol<<endl;
        // set the value of matched nodes to max
        new_mat(minRow, minCol) = minvalue;
        err_mat.col(minCol) = colmax;
        err_mat.row(minRow) = rowmax;
    }

    return new_mat;
}


template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::getBestMatchFromSimMat(Eigen::MatrixXd sim_mat, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse)
{
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXd new_mat = Eigen::MatrixXd::Zero(sim_mat.rows(), sim_mat.cols());
    Eigen::VectorXd colmin = Eigen::VectorXd::Constant(sim_mat.rows(), 0)
            , rowmin = Eigen::VectorXd::Constant(sim_mat.cols(), 0);
    double maxvalue;
    Eigen::MatrixXd sinksim_mat = sim_mat;
    for(int row=0; row<sim_mat.rows(); ++row)
    {
        maxvalue = sinksim_mat.maxCoeff(&maxRow, &maxCol);
        if(maxvalue < 0.01){break;}
        retmatch(maxRow) = maxCol;
        retmatchinverse(maxCol) = maxRow;
        new_mat(maxRow, maxCol) = sim_mat(maxRow, maxCol);
        // set the values to 0 to find the next best match
        sinksim_mat.col(maxCol) = colmin;
        sinksim_mat.row(maxRow) = rowmin;
    }

    return new_mat;
}

template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::getBestMatchFromSimTensor(Eigen::MatrixXd sim_ten, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse)
{
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXd new_mat = Eigen::MatrixXd::Zero(prior_graph->size(), visited_graph->size_confirm_nodes);
    Eigen::VectorXd colmin = Eigen::VectorXd::Constant(sim_ten.rows(), 0)
            , rowmin = Eigen::VectorXd::Constant(sim_ten.cols(), 0);
    double maxvalue;
    Eigen::MatrixXd sinksim_ten = sim_ten;
    for(int row=0; row<sim_ten.rows(); ++row)
    {
        maxvalue = sinksim_ten.maxCoeff(&maxRow, &maxCol);
        if(maxvalue < 0.01){
            break;
        }
        int id1_p = int(maxRow / visited_graph->size_confirm_nodes),
            id1_v = maxRow % visited_graph->size_confirm_nodes,
            id2_p = int(maxCol / visited_graph->size_confirm_nodes),
            id2_v = maxCol % visited_graph->size_confirm_nodes;
        cout<<id1_p<<", "<<id2_p<<", "<<id1_v<<", "<<id2_v<<", "<<maxvalue<<endl;
        retmatch(id1_p) = id1_v;
        retmatchinverse(id1_v) = id1_p;
        retmatch(id2_p) = id2_v;
        retmatchinverse(id2_v) = id2_p;
        new_mat(id1_p, id1_v) = maxvalue;
        new_mat(id2_p, id2_v) = maxvalue;
        // set the values to 0 to find the next best match
        sinksim_ten(maxRow, maxCol) = 0;
        sinksim_ten(maxCol, maxRow) = 0;
        for(int i=0; i<prior_graph->size(); ++i){
            if(i != id1_p){
                int ind = i * visited_graph->size_confirm_nodes + id1_v;
                sinksim_ten.col(ind) = colmin;
                sinksim_ten.row(ind) = rowmin;
            }
            if(i != id2_p){
                int ind = i * visited_graph->size_confirm_nodes + id2_v;
                sinksim_ten.col(ind) = colmin;
                sinksim_ten.row(ind) = rowmin;
            }
        }
        for(int j=0; j<visited_graph->size_confirm_nodes; ++j){
            if(j != id1_v){
                int ind = id1_p * visited_graph->size_confirm_nodes + j;
                sinksim_ten.col(ind) = colmin;
                sinksim_ten.row(ind) = rowmin;
            }
            if(j != id2_v){
                int ind = id2_p * visited_graph->size_confirm_nodes + j;
                sinksim_ten.col(ind) = colmin;
                sinksim_ten.row(ind) = rowmin;
            }
        }
    }

    return new_mat;
}

template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::sinkhorn(Eigen::MatrixXd sim_mat)
{
    double sum_of_rc;
    for(int iter=0; iter<max_sinkhorn_iter; ++iter)
    {
        for(int row=0; row<sim_mat.rows(); ++row)
        {
            sum_of_rc = sim_mat.row(row).sum();
            if(sum_of_rc < 0.1) continue;
            sim_mat.row(row) = sim_mat.row(row) / sum_of_rc;
        }
        for(int col=0; col<sim_mat.cols(); ++col)
        {
            sum_of_rc = sim_mat.col(col).sum();
            if(sum_of_rc < 0.1) continue;
            sim_mat.col(col) = sim_mat.col(col) / sum_of_rc;
        }
    }

    return sim_mat;
}


template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::getTopSimilarEstimation(int _id_visited, vector<int>& _node_id_vec, vector<Eigen::Quaterniond>& _q_vec, vector<Eigen::Vector3d>& _t_vec)
{
    vector<double> value_vec;
    _node_id_vec.clear();
    // find the best clusters of matches
    for(int i_x=0; i_x<voteT_len_x; ++i_x)
    {
        for(int i_y=0; i_y<voteT_len_y; ++i_y)
        {
            for(int i_z=0; i_z<voteT_len_z; ++i_z)
            {
                double value = vote_tensor(i_x, i_y, i_z);
                if(value < 0.1) continue;
                Eigen::Vector3d _t = turntot(i_x, i_y, i_z);
                string keystr = turntoString(i_x, i_y, i_z);
                int id_prior = vote_match_inverse_map[keystr](_id_visited);
                if(_node_id_vec.size() == 0){
                    // the first to push
                    value_vec.push_back(value);
                    _node_id_vec.push_back(id_prior);
                    _q_vec.push_back(vote_qpv_map[keystr]);
                    _t_vec.push_back(_t);
                }
                else if(value > value_vec[0] + 0.3){
                    // clear and push better estimation
                    value_vec.clear();
                    _node_id_vec.clear();
                    _q_vec.clear();
                    _t_vec.clear();
                    value_vec.push_back(value);
                    _node_id_vec.push_back(id_prior);
                    _q_vec.push_back(vote_qpv_map[keystr]);
                    _t_vec.push_back(_t);
                }
                else if(value < value_vec[0] - 0.3){
                    // abandon worse estimation
                }
                else{
                    value_vec.push_back(value);
                    _node_id_vec.push_back(id_prior);
                    _q_vec.push_back(vote_qpv_map[keystr]);
                    _t_vec.push_back(_t);
                }
            }
        }
    }
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::getBestResultFromVoteTensor(Eigen::Tensor<double, 3> _vote_tensor, int topN, vector<Eigen::Vector3i>& _index_of_votetensor)
{
    if(topN < 0){
        topN = INT_MAX;
    }

    for(int i_x=0; i_x<voteT_len_x; ++i_x)
    {
        for(int i_y=0; i_y<voteT_len_y; ++i_y)
        {
            for(int i_z=0; i_z<voteT_len_z; ++i_z)
            {
                double value = _vote_tensor(i_x, i_y, i_z);
                // judge and insert to return vector
                if(value > 0.1){
                    if(_index_of_votetensor.size() == 0){
                        //nothing in vector
                        _index_of_votetensor.push_back(Eigen::Vector3i(i_x, i_y, i_z));
                    }
                    else{
                        if(value < _vote_tensor(_index_of_votetensor[_index_of_votetensor.size()-1](0),
                               _index_of_votetensor[_index_of_votetensor.size()-1](1),
                               _index_of_votetensor[_index_of_votetensor.size()-1](2))){
                            // smaller than last one
                            if(_index_of_votetensor.size()<topN &&
                                  !checkTheSameMatchResult(_index_of_votetensor[_index_of_votetensor.size()-1],
                                       Eigen::Vector3i(i_x, i_y, i_z))){
                                _index_of_votetensor.push_back(Eigen::Vector3i(i_x, i_y, i_z));
//                                cout<<"smaller: "<<i_x<<", "<<i_y<<", "<<i_z<<", value:"<<value<<endl;
                            }
                            continue;
                        }
                        else if(value > _vote_tensor(_index_of_votetensor[0](0),
                                       _index_of_votetensor[0](1), _index_of_votetensor[0](2))){
                            // bigger than first one
                            if(checkTheSameMatchResult(_index_of_votetensor[0], Eigen::Vector3i(i_x, i_y, i_z))){
                                _index_of_votetensor[0] = Eigen::Vector3i(i_x, i_y, i_z);
                            }
                            else{
                                _index_of_votetensor.insert(_index_of_votetensor.begin(), Eigen::Vector3i(i_x, i_y, i_z));
                            }
//                            cout<<"larger: "<<i_x<<", "<<i_y<<", "<<i_z<<", value:"<<value<<endl;
                        }
                        else{
                            // smaller than first and bigger than last one
                            for(int i_index=_index_of_votetensor.size()-1; i_index>=0; --i_index)
                            {
                                // from last on, if to a value bigger than self, insert after this
                                if(value <= _vote_tensor(_index_of_votetensor[i_index](0),
                                        _index_of_votetensor[i_index](1), _index_of_votetensor[i_index](2))){
                                    if(!checkTheSameMatchResult(_index_of_votetensor[i_index], Eigen::Vector3i(i_x, i_y, i_z)) &&
                                         !checkTheSameMatchResult(_index_of_votetensor[i_index+1], Eigen::Vector3i(i_x, i_y, i_z))){
                                        _index_of_votetensor.insert(_index_of_votetensor.begin()+1+i_index, Eigen::Vector3i(i_x, i_y, i_z));
                                    }
                                    break;
                                }
                            }
//                            cout<<"middle: "<<i_x<<", "<<i_y<<", "<<i_z<<", value:"<<value<<endl;
                        }
                        if(_index_of_votetensor.size() > topN){
                            _index_of_votetensor.pop_back();
                        }
                    }
                }
            }
        }
    }
}

template <typename GraphT, typename NodeT>
bool GraphMatching<GraphT, NodeT>::checkTheSameMatchResult(Eigen::Vector3i index0, Eigen::Vector3i index1)
{return false;
    string keystr0 = turntoString(index0(0), index0(1), index0(2)),
           keystr1 = turntoString(index1(0), index1(1), index1(2));
    Eigen::VectorXi match0 = vote_match_map[keystr0], match1 = vote_match_map[keystr1];

    return ((match0 - match1).norm() < 0.000001);
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::distanceOfTwoPoint(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return (p1 - p2).norm();
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::errorOfTwoPoint(Eigen::Vector3d ep1, Eigen::Vector3d ep2)
{
    double ep1norm = ep1.norm(), ep2norm = ep2.norm(), largernorm;
    if(ep1norm > ep2norm){
        largernorm = ep1norm;
    }
    else{
        largernorm = ep2norm;
    }
    double costheta = ep1.dot(ep2) / ep1norm / ep2norm, ret;
    if(costheta < 0){
        ret = 1;
    }
    else{
        ret = err_factor_cos*(1 - costheta) + err_factor_dist*fabs(ep1norm - ep2norm) / ep1norm;
    }
    return ret;
}

template <typename GraphT, typename NodeT>
string GraphMatching<GraphT, NodeT>::turntoString(int a, int b, int c)
{
    char tmp[16];
    sprintf(tmp, "%05d%05d%05d", a,b,c);
    string str = tmp;
//    cout<<"tostring: "<<str<<endl;
    return str;
}

template <typename GraphT, typename NodeT>
Eigen::Vector3d GraphMatching<GraphT, NodeT>::turntot(int a, int b, int c)
{
    Eigen::Vector3d ret(vote_resolution * a + start_of_vote_tensor(0),
                        vote_resolution * b + start_of_vote_tensor(1),
                        vote_resolution * c + start_of_vote_tensor(2));
    return ret;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::turntoabc(Eigen::Vector3d t_prior_visited, int& a, int& b, int& c)
{
    a = int((t_prior_visited(0) + 0.01 - start_of_vote_tensor(0)) / vote_resolution);
    b = int((t_prior_visited(1) + 0.01 - start_of_vote_tensor(1)) / vote_resolution);
    c = int((t_prior_visited(2) + 0.01 - start_of_vote_tensor(2)) / vote_resolution);
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::getBestTpriorvisited(Eigen::Vector3d &t_prior_visited, Eigen::Quaterniond &q_prior_visited)
{
    if(!is_matched){
        cout<<"[GM]cannot get best t_prior_visited because of you have not done matching"<<endl;
        return;
    }
    Eigen::Vector3i index_bestmatch = index_of_votetensor[0];
    string key_bestmatch = turntoString(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    q_prior_visited = vote_qpv_map[key_bestmatch];
    t_prior_visited = turntot(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
}

template <typename GraphT, typename NodeT>
Eigen::Vector3d GraphMatching<GraphT, NodeT>::getCoordInVisitedFromIDPrior(int id_prior)
{
    if(id_prior >= prior_graph->size()){
        cout<<"[GM]request id "<< id_prior<<" out of prior graph size [0 - "<<
                (prior_graph->size()-1)<<"]."<<endl;
        return Eigen::Vector3d(0,0,0);
    }
    Eigen::Vector3i index_bestmatch = index_of_votetensor[0];
    string keystr = turntoString(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    Eigen::Vector3d xyz_prior = prior_graph->nodes_vec[id_prior].xyz,
            t_prior_visited = turntot(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    Eigen::Quaterniond q_prior_visited = vote_qpv_map[keystr];
    Eigen::Vector3d xyz_visited = q_prior_visited.toRotationMatrix().transpose() * (xyz_prior - t_prior_visited);
    // cout<<q_prior_visited.w()<<","<<q_prior_visited.x()<<","<<q_prior_visited.y()<<","<<q_prior_visited.z()<<endl<<t_prior_visited<<endl<<xyz_prior<<endl;
    return xyz_visited;
}

template <typename GraphT, typename NodeT>
bool GraphMatching<GraphT, NodeT>::inThisPuzzleZone(int node_id, int choose_graph, int id_puzzle_zone)
{
    int id_prior = node_id;
    if(choose_graph == 1){
        id_prior = getIDPriorFromIDVisited(node_id);
    }
    for(int i=0; i<node_to_puzzle_zone[id_prior].size(); ++i)
    {
        if(node_to_puzzle_zone[id_prior][i] == id_puzzle_zone){
            cout<<"[GM]prior node "<<node_id<<" is in puzzle zone "<<id_puzzle_zone<<endl;
            return true;
        }
    }
    cout<<"[GM]prior node "<<node_id<<" is NOT in puzzle zone "<<id_puzzle_zone<<endl;
    return false;
}

template <typename GraphT, typename NodeT>
vector<int> GraphMatching<GraphT, NodeT>::getPuzzleZoneNodes(int id_puzzle_zone)
{
    return puzzle_zone_vec[id_puzzle_zone];
}

template <typename GraphT, typename NodeT>
bool GraphMatching<GraphT, NodeT>::inTheSamePuzzleZone(vector<int> node_id, int choose_graph, vector<int> &id_puzzle_zone_vec)
{
    vector<int> all_puzzle_zones;
    for(int i=0; i<puzzle_zone_vec.size(); ++i)
    {
        all_puzzle_zones.push_back(i);
    }

    for(int i=0; i<node_id.size(); ++i)
    {
        int id_prior = node_id[i];
        if(choose_graph == 1){
            id_prior = getIDPriorFromIDVisited(node_id[i]);
        }
        // cout<<"[GM]prior node "<<id_prior<<endl;

        int res_puzzle_zone_size = all_puzzle_zones.size();
        for(int j=0; j<res_puzzle_zone_size; ++j)
        {
            bool exist = false;
            // erase puzzle zones that are not with the nodes
            for(int k=0; k<node_to_puzzle_zone[id_prior].size(); ++k)
            {
                if(all_puzzle_zones[j] == node_to_puzzle_zone[id_prior][k]){
                    exist = true;
                    break;
                }
            }
            if(!exist){
                all_puzzle_zones.erase(all_puzzle_zones.begin()+j);
                --j;
                --res_puzzle_zone_size;
                if(all_puzzle_zones.size() == 0){
                    return false;
                }
            }
        }
    }

//    id_puzzle_zone = all_puzzle_zones[all_puzzle_zones.size()-1];
    id_puzzle_zone_vec = all_puzzle_zones;
    return true;
}

template <typename GraphT, typename NodeT>
Eigen::VectorXi GraphMatching<GraphT, NodeT>::recoverPriorPath(vector<int> actual_path_visited)
{
    Eigen::VectorXi path;
    path.resize(actual_path_visited.size());
    for(int i=0; i<actual_path_visited.size(); ++i)
    {
        path(i) = getIDPriorFromIDVisited(actual_path_visited[i]);
    }
    return path;
}

template <typename GraphT, typename NodeT>
int GraphMatching<GraphT, NodeT>::getIDPriorFromIDVisited(int id_visited)
{
    if(!is_matched || vote_match_inverse_map.size() == 0){
        cout<<"[GM]have not run matching function."<<endl;
        return -1;
    }
    if(id_visited >= visited_graph->size()){
        cout<<"[GM]request id "<< id_visited<<" out of visited graph size [0 - "<<
              (visited_graph->size()-1)<<"]."<<endl;
        return -1;
    }
    Eigen::Vector3i index_bestmatch = index_of_votetensor[0];
    string keystr = turntoString(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    // cout<<"[GM]index of best match: "<<index_bestmatch(0)<<","<<index_bestmatch(1)<<","<<index_bestmatch(2)<<", "<<keystr<<endl;
    int id_prior = vote_match_inverse_map[keystr](id_visited);
    // cout<<vote_match_inverse_map[keystr]<<endl;
    //(todo: if there is no matching node)
    int tmp_id_prior;
    if(id_prior < 0){
        double nearest_dist = 100000, tmp_dist;
        for(int i=0; i<visited_graph->getNode(id_visited)->id_neibor.size(); ++i)
        {
            tmp_dist = getNearestNodeInVisitedWithMatchingInPrior(id_visited, visited_graph->getNode(id_visited)->id_neibor[i], tmp_id_prior, keystr);
//            cout<<"neibor "<<i<<" to nearest match node "<<tmp_id_prior<<", distance "<<tmp_dist<<endl;
            if(tmp_dist < nearest_dist){
                nearest_dist = tmp_dist;
                id_prior = tmp_id_prior;
            }
        }
    }

    return id_prior;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::getNearestNodeInVisitedWithMatchingInPrior(int id_visited, int id_neibor_visited, int& id_prior, string keystr)
{
//    cout<<"checking "<<id_visited<<" neibor "<<id_neibor_visited<<endl;
    // check if neibor has matching
    int tmp_id_prior = vote_match_inverse_map[keystr](id_neibor_visited);
    double dist;
    if(tmp_id_prior >= 0){
        id_prior = tmp_id_prior;
        for(int i=0; i<visited_graph->getNode(id_visited)->id_neibor.size(); ++i)
        {
            if(visited_graph->getNode(id_visited)->id_neibor[i] == id_neibor_visited){
                dist = visited_graph->getNode(id_visited)->dist_neibor[i];
                break;
            }
        }
//        cout<<"matched node in prior is "<<id_prior<<", distance is "<<dist<<endl;
        return dist;
    }

    // find neibor of neibor which has visited
    double nearest_dist = 100000, adddist;
    for(int i=0; i<visited_graph->getNode(id_neibor_visited)->id_neibor.size(); ++i)
    {
        if(id_visited == visited_graph->getNode(id_neibor_visited)->id_neibor[i]){
            adddist = visited_graph->getNode(id_visited)->dist_neibor[i];
            continue;
        }
        dist = getNearestNodeInVisitedWithMatchingInPrior(id_neibor_visited,
                  visited_graph->getNode(id_neibor_visited)->id_neibor[i], tmp_id_prior, keystr);
        if(dist < nearest_dist){
            id_prior = tmp_id_prior;
            nearest_dist = dist;
        }
    }
    return nearest_dist + adddist;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::calYaw(Eigen::Quaterniond _q)
{
    return atan2(2 * (_q.w() * _q.z() + _q.x() * _q.y()) , 1 - 2 * (_q.y() * _q.y() + _q.z() * _q.z()));
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::errYaw(double yaw1, double yaw2)
{
    double err = yaw1 - yaw2, PI_M_2 = 2*M_PI;
    if(err > PI_M_2){
        err -= PI_M_2;
    }
    else if(err < -PI_M_2){
        err += PI_M_2;
    }
    return err;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::test()
{
}

template <typename GraphT, typename NodeT>
bool GraphMatching<GraphT, NodeT>::existSameZone(vector<vector<int> > zone_vec, vector<vector<int> > match_zone_vec, vector<int> nodes, vector<int> match_nodes)
{
    for(int i=0; i<zone_vec.size(); ++i)
    {
        // for all puzzle zones recorded
        bool exist = true;
        if(zone_vec[i].size() != nodes.size()){
            exist = false;
            for(int j=0; j<zone_vec[i].size(); ++j)
            {
                // remove all nodes in the smaller group
                for(int k=0; k<nodes.size(); ++k)
                {
                    if(zone_vec[i][j] == nodes[k] && match_zone_vec[i][j] == match_nodes[k]){
                        nodes.erase(nodes.begin()+k);
                        match_nodes.erase(match_nodes.begin()+k);
                        break;
                    }
                }
                if(nodes.size() == 0){
                    exist = true;
                    break;
                }
            }
        }
        else{
            for(int j=0; j<nodes.size(); ++j)
            {
                // compare one by one
                if(nodes[j] == zone_vec[i][j] && match_nodes[j] == match_zone_vec[i][j]){
                    continue;
                }
                else{
                    exist = false;
                    break;
                }
            }
        }
        if(exist){
            return true;
        }
    }
    return false;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::removeErrPuzzleZone(vector<vector<int> > &zone_vec, vector<vector<int> > &match_zone_vec)
{
    vector<vector<int> > tmpzone_vec, tmpmatch_zone_vec;
    for(int i=0; i<zone_vec.size(); ++i)
    {
        for(int j=i+1; j<match_zone_vec.size(); ++j)
        {
            if(zone_vec[i].size() != match_zone_vec[j].size()) continue;
            bool isthesame = true;
            for(int k=0; k<zone_vec[i].size(); ++k)
            {
                if(zone_vec[i][k] != match_zone_vec[j][k] || zone_vec[j][k] != match_zone_vec[i][k]){
                    isthesame = false;
                    break;
                }
            }
            if(isthesame){
                tmpzone_vec.push_back(zone_vec[i]);
                tmpmatch_zone_vec.push_back(match_zone_vec[i]);
                tmpzone_vec.push_back(zone_vec[j]);
                tmpmatch_zone_vec.push_back(match_zone_vec[j]);
            }
        }
    }

    zone_vec = tmpzone_vec;
    match_zone_vec = tmpmatch_zone_vec;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::findPuzzleZone(vector<Eigen::Vector3i> _index_of_votetensor)
{
    node_to_puzzle_zone.clear();
    node_to_puzzle_zone.resize(prior_graph->size());
    puzzle_zone_vec.clear();
    cores_puzzle_zone_vec.clear();
    string keystr;
    vector<int> node_similar_zone_vec, coresponding_node_vec;
    vector<vector<int> > split_nodes;
    Eigen::VectorXd node_dealt;
    for(int i=1; i<_index_of_votetensor.size(); ++i)
    {// skip the first match which is all nodes matched
        int cnt_matched_nodes=0, cnt_same_node_match=0;
        keystr = turntoString(_index_of_votetensor[i](0), _index_of_votetensor[i](1), _index_of_votetensor[i](2));
        Eigen::VectorXi retmatch = vote_match_map[keystr];
        node_similar_zone_vec.clear();
        coresponding_node_vec.clear();
        for(int tmpi=0; tmpi<prior_graph->size(); ++tmpi){
            if(retmatch(tmpi) < 0) continue;
            if(retmatch(tmpi) == tmpi) ++cnt_same_node_match;
            ++cnt_matched_nodes;
            node_similar_zone_vec.push_back(tmpi);
            coresponding_node_vec.push_back(retmatch(tmpi));
        }
        if(cnt_matched_nodes < 3) break;  // do not care about zones with less than 3 points
        if(cnt_same_node_match >= 2){
            continue;
        }
        node_dealt.resize(cnt_matched_nodes);
        node_dealt.setConstant(-1);
        // find bunch neibor nodes more than 3(which is puzle zone)
        split_nodes.clear();
        prior_graph->splitNodesByNeibor(node_similar_zone_vec, split_nodes, 3);
        // store split_nodes
        vector<int> tmp_cores_node_vec;
        vector<vector<int> > tmpmatch_nodes;
        if(i < max_show_result){
            cout<<"best match "<<i<<": with "<<split_nodes.size()<<" groups of nodes"<<endl;
        }
        for(int si=0; si<split_nodes.size(); ++si)
        {
            tmp_cores_node_vec.clear();
            // see if match nodes are linked together
            for(int gi=0; gi<split_nodes[si].size(); ++gi)
            {
                int cores_node = retmatch(split_nodes[si][gi]);
                tmp_cores_node_vec.push_back(cores_node);
                if(i < max_show_result){
                    cout<<split_nodes[si][gi]<<"~"<<cores_node<<", ";
                }
            }
            if(i < max_show_result){
                cout<<endl<<"total "<<split_nodes[si].size()<<" nodes"<<endl;
            }
            tmpmatch_nodes.clear();
            prior_graph->splitNodesByNeibor(tmp_cores_node_vec, tmpmatch_nodes, 1);
            // matched puzzle zone is not linked together, continue
            if(tmpmatch_nodes.size() > 1) continue;

            // judge if the same zone exist
            if(existSameZone(puzzle_zone_vec, cores_puzzle_zone_vec, split_nodes[si], tmp_cores_node_vec)) continue;

            puzzle_zone_vec.push_back(split_nodes[si]);
            cores_puzzle_zone_vec.push_back(tmp_cores_node_vec);
        }
    }

    removeErrPuzzleZone(puzzle_zone_vec, cores_puzzle_zone_vec);

    //save puzzle zone
    ofstream outfile(graph_path+"puzzle_zone.txt");
    for(int i=0; i<puzzle_zone_vec.size(); ++i)
    {
        string s1="", s2="";
        for(int j=0; j<puzzle_zone_vec[i].size(); ++j)
        {
            node_to_puzzle_zone[puzzle_zone_vec[i][j]].push_back(i);
            s1 += to_string(puzzle_zone_vec[i][j]);
            s1 += " ";
            s2 += to_string(cores_puzzle_zone_vec[i][j]);
            s2 += " ";
        }
        outfile << s1 << "> " << s2 << endl;
    }
    outfile.close();
    // show
    for(int i=0; i<puzzle_zone_vec.size(); ++i)
    {
        cout<<"puzzle zone "<<i<<endl;
        for(int j=0; j<puzzle_zone_vec[i].size(); ++j)
        {
            cout<<puzzle_zone_vec[i][j]<<" ";
        }
        cout<<"---> ";
        for(int j=0; j<cores_puzzle_zone_vec[i].size(); ++j)
        {
            cout<<cores_puzzle_zone_vec[i][j]<<" ";
        }
        cout<<endl;
    }
    for(int i=0; i<node_to_puzzle_zone.size(); ++i)
    {
        cout<<"node "<<i<<" belongs to puzzle zone: ";
        for(int j=0; j<node_to_puzzle_zone[i].size(); ++j)
        {
            cout<<node_to_puzzle_zone[i][j]<<", ";
        }
        cout<<endl;
    }
}

template <typename GraphT, typename NodeT>
Eigen::VectorXi GraphMatching<GraphT, NodeT>::getBestMatchRes()
{
    string keystr = turntoString(index_of_votetensor[0](0), index_of_votetensor[0](1), index_of_votetensor[0](2));
    return vote_match_map[keystr];
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::showResult(vector<Eigen::Vector3i> _index_of_votetensor)
{
    // cout top n match
    string keystr;
    cout<<"show best match~~~~~~~~~~~~~~~~~~~~"<<endl;
    for(int i=0; i<_index_of_votetensor.size(); ++i)
    {
        keystr = turntoString(_index_of_votetensor[i](0), _index_of_votetensor[i](1), _index_of_votetensor[i](2));
        Eigen::VectorXi retmatch = vote_match_map[keystr];
        Eigen::Quaterniond qwxyz = vote_qpv_map[keystr];
        cout<<"------top match "<<i<<endl
            <<"index of vote tensor: "<<_index_of_votetensor[i](0)<<","<<_index_of_votetensor[i](1)<<","<<_index_of_votetensor[i](2)<<endl
            <<"t:"<<(vote_resolution * _index_of_votetensor[i](0) + start_of_vote_tensor(0))<<", "
                <<(vote_resolution * _index_of_votetensor[i](1) + start_of_vote_tensor(1))<<", "
                <<(vote_resolution * _index_of_votetensor[i](2) + start_of_vote_tensor(2))<<endl
            <<"qwxyz:"<<qwxyz.w()<<", "<<qwxyz.x()<<", "<<qwxyz.y()<<", "<<qwxyz.z()<<endl
            <<"score: "<<vote_tensor(_index_of_votetensor[i](0), _index_of_votetensor[i](1), _index_of_votetensor[i](2))<<endl;
        int cnt_matched_nodes=0;
        for(int tmpi=0; tmpi<prior_graph->size(); ++tmpi){
            if(retmatch(tmpi) < 0) continue;
            cout<<tmpi<<"~"<<retmatch(tmpi)<<"==";
            ++cnt_matched_nodes;
        }
        cout<<endl;
        if((cnt_matched_nodes < 3 && i>20) || i>max_show_result) break; //do not show too many results
    }
}



#endif
