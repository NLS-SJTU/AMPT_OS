#include "ViterbiLoc.h"

void ViterbiLoc::reset(string graph_path)
{
    _prob_t.setConstant(1.0/_prior_graph.size());
    _last_node_vec.clear();
    _prob_t_vec.clear();
    _save_path = graph_path;
}

void ViterbiLoc::init(TopoMetric_c prior_graph)
{
    _prior_graph = prior_graph;
    _prob_t.resize(_prior_graph.size());
}

Eigen::VectorXi ViterbiLoc::getBestPath(int endid=-1)
{
    Eigen::VectorXi path;
    path.resize(_last_node_vec.size());
    Eigen::MatrixXd::Index index;
    _prob_t.maxCoeff(&index);
    int lastid;
    if(endid < 0){
        lastid = index;
    }
    else{
        lastid = endid;
    }
    cout<<"[viterbi]show path:"<<endl;
    path(_last_node_vec.size()-1) = lastid;
    ofstream ost(_save_path+"Vloc.txt");
    for(int j=0; j<_prior_graph.size(); ++j)
    {
        ost << _prob_t_vec[_last_node_vec.size()-1](j) <<" ";
    }
    ost << endl;
    for(int j=0; j<_prior_graph.size(); ++j)
    {
        ost << _last_node_vec[_last_node_vec.size()-1](j) <<" ";
    }
    ost << endl;
//    cout<<"node "<<lastid<<", prob "<<_prob_t_vec[_last_node_vec.size()-1]<<" <- "<<endl;
    for(int i=_last_node_vec.size()-1; i>0; --i)
    {
        lastid = _last_node_vec[i](lastid);
        path(i-1) = lastid;
//        cout<<"node "<<lastid<<", prob "<<_prob_t_vec[i-1]<<" <- "<<endl;
        for(int j=0; j<_prior_graph.size(); ++j)
        {
            ost << _prob_t_vec[i-1](j) <<" ";
        }
        ost << endl;
        for(int j=0; j<_prior_graph.size(); ++j)
        {
            ost << _last_node_vec[i-1](j) <<" ";
        }
        ost << endl;
    }
    ost.close();
    cout<<endl;

    return path;
}

Eigen::VectorXd ViterbiLoc::update(Eigen::VectorXd prob_obs, Eigen::Vector3d action)
{
    Eigen::VectorXd prob_pred = predict(action);
    for(int i=0; i<prob_obs.size(); ++i)
    {
        _prob_t(i) = prob_obs(i) * prob_pred(i);
    }
    _prob_t = _prob_t / _prob_t.sum();
    _prob_t_vec.push_back(_prob_t);
//    cout<<"pred: "<<prob_pred<<endl<<"obs: "<<prob_obs<<endl<<"prob: "<<_prob_t<<endl;
    return _prob_t;
}

Eigen::VectorXd ViterbiLoc::predict(Eigen::Vector3d action)
{
    Eigen::VectorXd prob_pred = _prob_t;
    Eigen::VectorXi last_node;
    last_node.resize(_prior_graph.size());
    for(int i=0; i<_prior_graph.size(); ++i)
    {
        // find max nearby prob
        double prob = 0;
        for(int j=0; j<_prior_graph.getNode(i)->id_neibor.size(); ++j)
        {
            int neibor_id = _prior_graph.getNode(i)->id_neibor[j];
            if(_prob_t(neibor_id) > prob){
                prob = _prob_t(neibor_id);
                last_node(i) = neibor_id;
            }
        }
        prob_pred(i) = prob;
    }
    _last_node_vec.push_back(last_node);

    return prob_pred;
}
