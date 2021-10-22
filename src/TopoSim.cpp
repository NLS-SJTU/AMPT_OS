#include "TopoSim.h"

void TopoSim::readParam(const string& paramfile)
{
    cout<<"[TOPOSIM]read param"<<endl;
    cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
    string graph_path;
    fsSettings["ampt"]["simenv_graph"] >> graph_path;
    _simenv_graph.readAndContructTopoMetric(graph_path);
    _is_node_visited.resize(_simenv_graph.size());
    cv::Mat tmp;
    fsSettings["ampt"]["TopoSim"]["sim_path_to_go"] >> tmp;
    // cout<<"row "<<tmp.rows<<" col:"<<tmp.cols<<" data:"<<tmp<<endl;
    for(int i=0; i<tmp.cols; ++i)
    {
        _sim_path_to_go.push_back(int(tmp.at<int>(0,i)));
        // cout<<"weofasfo "<<(tmp.at<int>(0,i))<<" "<<_sim_path_to_go[i]<<endl;
    }
    _start_node_id = _sim_path_to_go[0];
    setRobotNode(_start_node_id);
    fsSettings["ampt"]["TopoSim"]["visited_graph_covariance"] >> _visited_graph_covariance;
}

void TopoSim::reset(){
    _actual_path.clear();
    _path_id = 1;
    _is_node_visited.setConstant(0);
    _simenv_graph.resetEdgeStatus();
    setRobotNode(_start_node_id);
}

void TopoSim::setRobotNode(int node_id_prior)
{
    _actual_path.clear();
    if(node_id_prior > _simenv_graph.size_confirm_nodes){
        _current_actual_node_id = 0;
    }
    else{
        _current_actual_node_id = node_id_prior;
    }
    _actual_path.push_back(_current_actual_node_id);
    _is_node_visited.setConstant(0);
    _is_node_visited(_current_actual_node_id) = 1;
    cout<<"[TOPOSIM]set start node to "<<_current_actual_node_id<<endl;
}

Eigen::Vector3d TopoSim::moveNext()
{
    Eigen::Vector3d action(0,0,0);
    if(_path_id >= _sim_path_to_go.size()) return action;
    moveTo(_sim_path_to_go[_path_id]);
    action = _simenv_graph.getNode(_sim_path_to_go[_path_id])->xyz -
            _simenv_graph.getNode(_sim_path_to_go[_path_id-1])->xyz;
    ++_path_id;
    return action;
}

bool TopoSim::moveTo(int nextid_prior)
{
    if(!_simenv_graph.edgeExist(_current_actual_node_id, nextid_prior)){
        cout<<"[TOPOSIM]cannot move to "<<nextid_prior<<" from "<<_current_actual_node_id<<endl;
        return false;
    }
    int last_id = _current_actual_node_id;
    _current_actual_node_id = nextid_prior;
    _is_node_visited(_current_actual_node_id) = 1;
    _simenv_graph.setVisitedEdge(last_id, _current_actual_node_id);

    _actual_path.push_back(_current_actual_node_id);
    cout<<"[TOPOSIM](GT)move to node "<<_current_actual_node_id<<" from node "<<last_id<<endl;
    return true;
}

bool TopoSim::moveToRela(Eigen::Vector3d target_relative)
{
    Node_c *curnode = _simenv_graph.getNode(_current_actual_node_id);
    target_relative = target_relative / target_relative.norm();
    double max_norm=-1, tmpnorm;
    int min_neibor;
    for(int i=0; i<curnode->id_neibor.size(); ++i)
    {
        Eigen::Vector3d rela_gt = _simenv_graph.getNode(curnode->id_neibor[i])->xyz - curnode->xyz;
        rela_gt /= rela_gt.norm();
        tmpnorm = rela_gt.dot(target_relative);
        if(tmpnorm > max_norm){
            max_norm = tmpnorm;
            min_neibor = i;
        }
    }
    if(max_norm <=0 ){
        cout<<"[TOPOSIM]cannot move to direction "<<target_relative<<" from "
           <<_current_actual_node_id<<", move to another direction."<<endl;
//        return false;
    }
    // set to dentination
    int nextid_prior = _simenv_graph.getNode(curnode->id_neibor[min_neibor])->get_id_self();

    return moveTo(nextid_prior);
}

bool TopoSim::moveTo(Eigen::Vector3d target_visited)
{
    Node_c *curnode = _simenv_graph.getNode(_current_actual_node_id);
    Eigen::Vector3d target_relative = target_visited + _simenv_graph.getNode(_start_node_id)->xyz - curnode->xyz;
    return moveToRela(target_relative);
}

vector<Eigen::Vector3d> TopoSim::getObservation(int node_id){
    if(node_id < 0) node_id = _current_actual_node_id;
    vector<Eigen::Vector3d> ret_obs;
    Node_c *curnode = _simenv_graph.getNode(node_id);
    for(int i=0; i<curnode->id_neibor.size(); ++i)
    {
        ret_obs.push_back(_simenv_graph.getNode(curnode->id_neibor[i])->xyz - curnode->xyz);
    }
    return ret_obs;
}

double TopoSim::compareEstPathWithGT(Eigen::VectorXi estpath)
{
    if(estpath.size() != _actual_path.size()){
        return false;
    }
    double errcnt;
    cout<<"[TOPOSIM]est path: ";
    for(int i=0; i<_actual_path.size(); ++i)
    {
        cout<< estpath(i) << " ,";
        if(_actual_path[i]!=estpath(i)){
            errcnt += 1;
        }
    }
    errcnt /= _actual_path.size();
    cout<<endl<<"[TOPOSIM]average path error is "<<errcnt<<endl;
    return errcnt;
}

vector<int> TopoSim::getActualPath()
{
    cout<<"[TOPOSIM]actual path: "<<_actual_path[0]<<", ";
    double length=0;
    for(int i=1; i<_actual_path.size(); ++i)
    {
        cout<<_actual_path[i]<<", ";
        length += (_simenv_graph.getNode(_actual_path[i])->xyz-_simenv_graph.getNode(_actual_path[i-1])->xyz).norm();
    }
    cout<<"length "<<length<<endl;
    return _actual_path;
}

vector<int> TopoSim::getPredefPath()
{
    return _sim_path_to_go;
}


vector<int> TopoSim::getActualPathVisited()
{
    vector<int> actual_path_visited;
    double length=0;
    actual_path_visited.push_back(_id_prior_to_visted(_actual_path[0]));
    cout<<"[TOPOSIM]path of visited graph: "<<actual_path_visited[0]<<", ";
    for(int i=1; i<_actual_path.size(); ++i)
    {
        length += (_simenv_graph.getNode(_actual_path[i])->xyz-_simenv_graph.getNode(_actual_path[i-1])->xyz).norm();
        actual_path_visited.push_back(_id_prior_to_visted(_actual_path[i]));
        cout<<actual_path_visited[i]<<", ";
    }
    cout<<"length "<<length<<endl;
    return actual_path_visited;
}

vector<Eigen::Vector3d> TopoSim::getVisitedNodePosition()
{
    vector<Eigen::Vector3d> node_pos;
    Eigen::Vector3d startpos = _simenv_graph.getNode(_start_node_id)->xyz;
    for(int i=0; i<_actual_path.size(); ++i)
    {
        Eigen::Vector3d pathpos = _simenv_graph.getNode(_actual_path[i])->xyz;
        node_pos.push_back(pathpos - startpos);
    }
    return node_pos;
}

int TopoSim::getSimVisitedGraph(TopoMetric_c &graph)
{
    Node_c tmpn;
    int num_visi=0;
    graph.clear();
    vector<int> inserted_node_id_prior;
    Eigen::VectorXi id_prior_to_new, tmpis_node_visited = _is_node_visited;
    id_prior_to_new.resize(_simenv_graph.size());
    id_prior_to_new.setConstant(-1);
    // add visited nodes
    double max_x=-10000, min_x=10000, max_y=-10000, min_y=10000;
    for(int i=0; i<_simenv_graph.size(); ++i)
    {
        if(tmpis_node_visited(i) > 0 && tmpis_node_visited(i) < 2){
            inserted_node_id_prior.push_back(i);
//            cout<<"push to visited node (new)"<<num_visi<<" (old)"<<i<<endl;
            id_prior_to_new(i) = num_visi;
            Eigen::Vector3d xyz = _simenv_graph.getNode(i)->xyz - _simenv_graph.getNode(_start_node_id)->xyz;
            double rand1 = _visited_graph_covariance * (rand() % (99 + 1) / (double)(99 + 1) * 2 - 1);
            double rand2 = _visited_graph_covariance * (rand() % (99 + 1) / (double)(99 + 1) * 2 - 1);
            tmpn.setxyzid(xyz(0)+rand1, xyz(1)+rand2, xyz(2), num_visi);
            graph.nodes_vec.push_back(tmpn);
            tmpis_node_visited(i) += 1;
            ++num_visi;
            if(xyz(0) > max_x){max_x = xyz(0);}
            if(xyz(0) < min_x){min_x = xyz(0);}
            if(xyz(1) > max_y){max_y = xyz(1);}
            if(xyz(1) < min_y){min_y = xyz(1);}
        }
    }
    graph.x_lowb = min_x;
    graph.x_upb = max_x;
    graph.y_lowb = min_y;
    graph.y_upb = max_y;
    graph.id_near_zero_node = id_prior_to_new(_start_node_id);
    graph.size_confirm_nodes = num_visi;
    graph.visited_edge_mat.resize(graph.nodes_vec.size(), graph.nodes_vec.size());
    graph.visited_edge_mat.setConstant(-1);
    // add virtual nodes and links
    for(int j=0; j<inserted_node_id_prior.size(); ++j)
    {
        int oldid = inserted_node_id_prior[j];
        int newid = id_prior_to_new(oldid);
        Node_c *oldnode = _simenv_graph.getNode(oldid);
        for(int i=0; i<oldnode->id_neibor.size(); ++i)
        {
            Node_c *oldneibor = _simenv_graph.getNode(oldnode->id_neibor[i]);
            int old_neiborid = oldneibor->get_id_self();
//            cout<<"neibor node (old)"<<old_neiborid<<endl;
            if(_simenv_graph.isEdgeVisited(oldid, old_neiborid)){
                //add link
                int new_neiborid = id_prior_to_new(old_neiborid);
                if(newid > new_neiborid) continue; // add only once
                float dist = (oldneibor->xyz - oldnode->xyz).norm();
                graph.nodes_vec[newid].addLink(new_neiborid, dist);
                graph.nodes_vec[new_neiborid].addLink(newid, dist);
//                cout<<"add link between "<<newid<<" and "<<new_neiborid<<endl;
            }
            else{
                //add virtual node
                Eigen::Vector3d xyz = oldneibor->xyz - oldnode->xyz;
                xyz /= xyz.norm();
                xyz = xyz + graph.getNode(newid)->xyz;
                int new_neiborid = graph.size();
                tmpn.setxyzid(xyz(0), xyz(1), xyz(2), new_neiborid);
                graph.nodes_vec.push_back(tmpn);
                graph.nodes_vec[newid].addLink(new_neiborid, 1);
                graph.nodes_vec[new_neiborid].addLink(newid, 1);
//                cout<<"add virtual link between "<<newid<<" and "<<new_neiborid<<endl;
            }
        }
    }
    _visited_graph = graph;
    _id_prior_to_visted = id_prior_to_new;

    return id_prior_to_new(_current_actual_node_id);
}
