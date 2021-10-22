#include "TopoMetric.h"

double calDistance(Eigen::Vector3d po1, Eigen::Vector3d po2){
    Eigen::Vector3d err = po1 - po2;
    return err.norm();
}

Node_c::Node_c(float _x, float _y, float _z, int _id_self):
    xyz(_x,_y,_z), id_self(_id_self)
{
    norm = xyz.norm();
}

Node_c::~Node_c(){}

void Node_c::setxyzid(float _x, float _y, float _z, int _id_self)
{
    xyz << _x, _y, _z;
    norm = xyz.norm();
    id_self = _id_self;
}

void Node_c::setxyzid(Eigen::Vector3d _xyz, int _id_self)
{
    xyz = _xyz;
    norm = xyz.norm();
    id_self = _id_self;
}

void Node_c::addLink(int _id_neibor, float _dist)//, Node_c* _node_neibor
{
    id_neibor.push_back(_id_neibor);
    // node_neibor.push_back(_node_neibor);
    visited_neibor.push_back(false);
    dist_neibor.push_back(_dist);
}

void Node_c::removeLink(int _id_neibor)
{
    for(int i=0; i<id_neibor.size(); ++i){
        if(_id_neibor == id_neibor[i]){
            id_neibor.erase(id_neibor.begin()+i);
            // node_neibor.erase(node_neibor.begin()+i);
            visited_neibor.erase(visited_neibor.begin()+i);
            break;
        }
    }
}


TopoMetric_c::TopoMetric_c(string _frame_id):
    id_near_zero_node(0), size_confirm_nodes(1),
    last_add_vertice(-1),
    x_upb(FLT_MIN), x_lowb(FLT_MAX),
    y_upb(FLT_MIN), y_lowb(FLT_MAX),
    z_upb(FLT_MIN), z_lowb(FLT_MAX)
{
    frame_id = _frame_id;
}

Node_c* TopoMetric_c::getNode(int _i)
{
    if(_i >= nodes_vec.size() || _i < 0){
        return &(nodes_vec[0]);
    }
    return &(nodes_vec[_i]);
}

void TopoMetric_c::saveGraph(string fname)
{
    ofstream outnode(fname+"_node.txt"), outedge(fname+"_edge.txt");

    for(int i=0; i<nodes_vec.size(); ++i)
    {
        if(i == size_confirm_nodes){
            outnode <<endl;
            // separate confirm nodes
        }
        outnode << nodes_vec[i].xyz(0) <<" "<< nodes_vec[i].xyz(1)
                <<" "<< nodes_vec[i].xyz(2) <<endl;
        for(int j=0; j<nodes_vec[i].id_neibor.size(); ++j)
        {
            int idneibor = nodes_vec[i].id_neibor[j];
            if(idneibor > i){
                outedge << i << " "  << idneibor <<endl;
            }
        }
    }

    outnode.close();
    outedge.close();
}

bool TopoMetric_c::readAndContructTopoMetric(const string& filepath)
{
    nodes_vec.clear();
    file_path = filepath;
    string nodefile = filepath+"node.txt",
           edgefile = filepath+"edge.txt";
    Node_c nc(0,0,0,0);

    // read nodes
    ifstream in(nodefile);
    if(!in){
        nodes_vec.push_back(nc);
        printf("[topometric]no node file!\n");
        return false;
    }
    string line;
    boost::char_separator<char> sep(" ");
    float x,y,z;
    int i=0,j=0;
    double min_zero_norm = 5.0;
    Eigen::VectorXd min_norms = Eigen::VectorXd::Constant(bcnode_xyz_vec.size(), 10.0);
    Eigen::VectorXi id_near_backup_node = Eigen::VectorXi::Constant(bcnode_xyz_vec.size(), -1);
    // id_near_zero_node = 0; //near zero node id
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        x = boost::lexical_cast<float>(tokens[0]);
        y = boost::lexical_cast<float>(tokens[1]);
        z = boost::lexical_cast<float>(tokens[2]);

        nc.setxyzid(x, y, z, i);
        for(int bc_i=0; bc_i<bcnode_xyz_vec.size(); ++bc_i)
        {
            double dist_center = (nc.xyz - bcnode_xyz_vec[bc_i]).norm();
            if(dist_center < min_norms(bc_i)){
                min_norms(bc_i) = dist_center;
                id_near_backup_node(bc_i) = nodes_vec.size();
            }
        }
        if(nc.norm < min_zero_norm){
            min_zero_norm = nc.norm;
            id_near_zero_node = i;
        }
        
        nodes_vec.push_back(nc);
        if(x > x_upb){
            x_upb = x;
        }
        else if(x < x_lowb){
            x_lowb = x;
        }
        if(y > y_upb){
            y_upb = y;
        }
        else if(y < y_lowb){
            y_lowb = y;
        }
        if(z > z_upb){
            z_upb = z;
        }
        else if(z < z_lowb){
            z_lowb = z;
        }
//        cout<<"[TM]read node:"<<x<<" "<<y<<" "<<z<<" "<<i<<endl;
        ++i;
    }
    if(nodes_vec.size() == 0){
        nodes_vec.push_back(nc);
    }
    in.close();
    size_confirm_nodes = nodes_vec.size();

    //read edges
    ifstream in1(edgefile);
    if(!in1){
        printf("[topometric]no edge file!\n");
        return false;
    }
    adjacent_mat.resize(nodes_vec.size(), nodes_vec.size());
    adjacent_mat.setConstant(-1);
    visited_edge_mat.resize(nodes_vec.size(), nodes_vec.size());
    visited_edge_mat.setConstant(-1);
    while (!in1.eof())
    {
        std::getline(in1, line);
        in1.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 2) continue;
        i = boost::lexical_cast<float>(tokens[0]);
        j = boost::lexical_cast<float>(tokens[1]);
        float _dist = (nodes_vec[i].xyz - nodes_vec[j].xyz).norm();

        nodes_vec[i].addLink(j, _dist);//, &(nodes_vec[j])
        nodes_vec[j].addLink(i, _dist);//, &(nodes_vec[i])
        adjacent_mat(i,j) = _dist;
        adjacent_mat(j,i) = _dist;
        visited_edge_mat(i, j) = 1;
        visited_edge_mat(j, i) = 1;
//        cout<<"[TM]read edge:"<<i<<" "<<j<<" "<<_dist<<endl;
    }
    in1.close();
    cout<<"[topometric]read "<<filepath<<" node and edge file done!"
    <<" number of nodes is " << nodes_vec.size() <<endl;
    if(bcnode_xyz_vec.size() == 0) return true;

    // replace with back up nodes
    nodes_vec.clear();
    for(int i=0; i<bcnode_xyz_vec.size(); ++i)
    {
        nc.setxyzid(bcnode_xyz_vec[i](0), bcnode_xyz_vec[i](1)
                    , bcnode_xyz_vec[i](2), i);
        nodes_vec.push_back(nc);
    }
    id_near_zero_node = 0;
    size_confirm_nodes = bcnode_xyz_vec.size();
    for(int i=0; i<bcedge_vec.size(); ++i)
    {
        float _dist = (nodes_vec[bcedge_vec[i](0)].xyz - nodes_vec[bcedge_vec[i](1)].xyz).norm();
        nodes_vec[bcedge_vec[i](0)].addLink(bcedge_vec[i](1),_dist);
        nodes_vec[bcedge_vec[i](1)].addLink(bcedge_vec[i](0),_dist);
    }
    for(int i=0; i<bcneibor_xyz_vecs.size(); ++i)
    {
        Eigen::Vector3d center_xyz = nodes_vec[i].xyz;
        for(int j=0; j<bcneibor_xyz_vecs[i].size(); ++j)
        {
            double min_angle_cos = 0.707;
            int min_neibor = -1;
            Eigen::Vector3d nei_xyz = bcneibor_xyz_vecs[i][j];
            for(int tmpi = 0; tmpi < nodes_vec[i].id_neibor.size(); ++tmpi)
            {
                Eigen::Vector3d xyz2 = nodes_vec[nodes_vec[i].id_neibor[tmpi]].xyz - center_xyz;
                double costheta = nei_xyz.dot(xyz2) / xyz2.norm() / nei_xyz.norm();
                if(costheta > min_angle_cos){
                    min_angle_cos = costheta;
                    min_neibor = tmpi;
                }
            }
            if(min_neibor >= 0){
                // remove this neibor
                // danger remove
                bcneibor_xyz_vecs[i].erase(bcneibor_xyz_vecs[i].begin()+j);
                --j;
                continue;
            }
            nei_xyz = center_xyz + bcneibor_xyz_vecs[i][j];
            nc.setxyzid(nei_xyz(0), nei_xyz(1), nei_xyz(2), i);
            nodes_vec.push_back(nc);
            float _dist = (nodes_vec[i].xyz - nodes_vec[nodes_vec.size()-1].xyz).norm();
            nodes_vec[i].addLink(nodes_vec.size()-1,_dist);
            nodes_vec[nodes_vec.size()-1].addLink(i,_dist);
        }
    }

    return true;
}

void TopoMetric_c::initZeroNode(float _x, float _y, float _z, int _id)
{
    cout<<"[topometric]init zero node."<<endl;
    Node_c nc(_x, _y, _z, _id);
    nodes_vec.push_back(nc);
}

int TopoMetric_c::searchNearestNode(Eigen::Vector3d _xyz)
{
    double nearest_dist = 100000, tmpdist;
    int nearest_id = -1;
    for(int id=0; id<size_confirm_nodes; ++id)
    {
        tmpdist = (nodes_vec[id].xyz - _xyz).norm();
        if(tmpdist < nearest_dist){
            nearest_dist = tmpdist;
            nearest_id = id;
        }
    }
    return nearest_id;
}

int TopoMetric_c::searchNearestNode(Eigen::Vector3d _xyz, double& nearest_dist)
{
    nearest_dist = 100000;
    double tmpdist;
    int nearest_id = -1;
    for(int id=0; id<size_confirm_nodes; ++id)
    {
        tmpdist = (nodes_vec[id].xyz - _xyz).norm();
        if(tmpdist < nearest_dist){
            nearest_dist = tmpdist;
            nearest_id = id;
        }
    }
    return nearest_id;
}

int TopoMetric_c::searchNearbyNode(Eigen::Vector3d _xyz, int id_initnode)
{
    if(id_initnode < 0) return -1;
    double nearest_dist = (nodes_vec[id_initnode].xyz - _xyz).norm(), tmpdist;
    int nearest_id = id_initnode;
    for(int neibor=0; neibor<nodes_vec[id_initnode].id_neibor.size(); ++neibor)
    {
        Eigen::Vector3d tmpxyz = nodes_vec[nodes_vec[id_initnode].id_neibor[neibor]].xyz;
        tmpdist = (tmpxyz - _xyz).norm();//nodes_vec[id_initnode].node_neibor[neibor]->xyz
        // to judge whether it is reaching next node, the distance should be smaller than 1m
        if(tmpdist < 1.0 && tmpdist < nearest_dist){
            nearest_dist = tmpdist;
            nearest_id = nodes_vec[id_initnode].id_neibor[neibor];
        }
    }
    return nearest_id;
}

void TopoMetric_c::splitNodesByNeibor(vector<int> nodes, vector<vector<int> > &split_nodes, int min_group_size)
{
    vector<int> one_group;
    for(int i=0; i<nodes.size(); ++i)
    {
        if(nodes[i] < 0) continue;
        one_group.clear();
        findGroupNodes(nodes, i, one_group);
        if(one_group.size() < min_group_size) continue;
        split_nodes.push_back(one_group);
    }
}

void TopoMetric_c::findGroupNodes(vector<int> &nodes, int index_in_vec, vector<int> &one_group)
{
//    cout<<"start group nodes"<<endl;
    one_group.push_back(nodes[index_in_vec]);
//    cout<<nodes[index_in_vec]<<", ";
    int ind_one_group = 0;
    while(ind_one_group < one_group.size())
    {
        for(int i=index_in_vec+1; i<nodes.size(); ++i)
        {
            if(nodes[i] < 0) continue;
            if(adjacent_mat(one_group[ind_one_group], nodes[i]) > 0){
                one_group.push_back(nodes[i]);
//                cout<<nodes[i]<<", ";
                nodes[i] = -1;
            }
        }
        ++ind_one_group;
    }
//    cout<<endl;
}

void TopoMetric_c::modifyNode(int _id, Eigen::Vector3d _new_xyz)
{
    nodes_vec[_id].xyz = _new_xyz;
    nodes_vec[_id].norm = _new_xyz.norm();
    for(int i=0; i<nodes_vec[_id].id_neibor.size(); ++i)
    {
        int id_neibor = nodes_vec[_id].id_neibor[i];
        float dist = (_new_xyz - nodes_vec[id_neibor].xyz).norm();
        nodes_vec[_id].dist_neibor[i] = dist;
//        adjacent_mat(id_neibor, _id) = dist;
//        adjacent_mat(_id, id_neibor) = dist;
        for(int j=0; j<nodes_vec[id_neibor].id_neibor.size(); ++j)
        {
            if(_id == nodes_vec[id_neibor].id_neibor[j]){
                nodes_vec[id_neibor].dist_neibor[j] = dist;
                break;
            }
        }
    }
}

void TopoMetric_c::setVisitedEdge(int id_node_1, int id_node_2, double multiple)
{
    if(visited_edge_mat(id_node_1, id_node_2) > 0){
        visited_edge_mat(id_node_1, id_node_2) = multiple;
        visited_edge_mat(id_node_2, id_node_1) = multiple;
    }
}

bool TopoMetric_c::edgeExist(int id1, int id2)
{
    return (visited_edge_mat(id1,id2) > 0);
}

double TopoMetric_c::getDistance(int id1, int id2)
{
    return (nodes_vec[id1].xyz - nodes_vec[id2].xyz).norm();
}

bool TopoMetric_c::isEdgeVisited(int id_node_1, int id_node_2)
{
    if(visited_edge_mat(id_node_1, id_node_2) > 1){
        return true;
    }
    else{
        return false;
    }
}

void TopoMetric_c::resetEdgeStatus()
{
    for(int i=0; i<nodes_vec.size(); ++i)
    {
        for(int j=i+1; j<nodes_vec.size(); ++j)
        {
            if(visited_edge_mat(i, j) > 0){
                visited_edge_mat(i, j) = 1;
                visited_edge_mat(j, i) = 1;
            }
        }
    }
}

bool TopoMetric_c::Dijkstra(int _id_start, vector<int>& near_to_far_nodes)
{
    cout<<"[TM]search path from node "<<_id_start<<endl;
    if(_id_start < 0 || _id_start >= nodes_vec.size()){
        cout<<"[TM]node id out of range [0, "<<nodes_vec.size()<<")."<<endl;
        return false;
    }
    // init
    id_start = _id_start;
    dist_to_start.resize(nodes_vec.size());
    id_from_where.resize(nodes_vec.size());
    id_from_where.setConstant(-1);
    Eigen::VectorXi id_dealed;
    id_dealed.resize(nodes_vec.size());
    id_dealed.setConstant(-1);

    vector<int> nodeQueue;
    id_from_where(id_start) = id_start;
    dist_to_start(id_start) = 0;
    id_dealed(id_start) = 1;
    nodeQueue.push_back(id_start);

    int id_now, id_next;
    float dtn_tmp;
    while(!nodeQueue.empty())
    {
        id_now = nodeQueue[nodeQueue.size()-1];

        nodeQueue.pop_back();
        near_to_far_nodes.push_back(id_now);
        // cout<<"dijk id: "<<id_now<<", dist: "<<dist_to_start(id_now)<<endl;

        for(int i=0; i<nodes_vec[id_now].id_neibor.size(); ++i)
        {
            id_next = nodes_vec[id_now].id_neibor[i];

            if(id_next == id_from_where(id_now)) continue;

            // visited_edge_mat, if this edge is visited, going back again is not recommended
            if(id_dealed(id_next) <= 0)
            {
                dist_to_start(id_next) = dist_to_start(id_now) + visited_edge_mat(id_now, id_next) * nodes_vec[id_now].dist_neibor[i];
                id_from_where(id_next) = id_now;
                id_dealed(id_next) = 1;

                insertSortByDistTotal(id_next, nodeQueue, dist_to_start);
            }
            else
            {
                dtn_tmp = dist_to_start(id_now) + visited_edge_mat(id_now, id_next) * nodes_vec[id_now].dist_neibor[i];
                if(dtn_tmp < dist_to_start(id_next))
                {
                    dist_to_start(id_next) = dtn_tmp;
                    id_from_where(id_next) = id_now;

                    int j=0;
                    //erase from queue and re-add into queue
                    for(j=0; j<nodeQueue.size(); ++j)
                    {
                        if(nodeQueue[j] == id_next)
                        {
                            nodeQueue.erase(nodeQueue.begin()+j);
                            insertSortByDistTotal(id_next, nodeQueue, dist_to_start);
                            break;
                        }
                    }
                }
            }
        }
    }
    cout<<"[TM]dijkstra done."<<endl;
    return true;
}

void TopoMetric_c::getPathFromDijkstra(int _id_end, vector<int>& path_vec)
{
    int id_current = _id_end;
    path_vec.push_back(id_current);
    cout<<"[TM]path from "<<id_start<<" to "<<_id_end<<": "<<id_current;
    while(id_current != id_start)
    {
        if(id_current == -1){
            cout<<"[TM]cannot get path."<<endl;
            path_vec.clear();
            break;
        }
        id_current = id_from_where(id_current);
        path_vec.push_back(id_current);
        cout<<"<-"<<id_current;
    }
    cout<<endl;
}

void TopoMetric_c::insertSortByDistTotal(int _id, vector<int> &_nodeQueue, Eigen::VectorXd value_source)
{
    if(_nodeQueue.empty())
    {
        _nodeQueue.push_back(_id);
    }
    else if(value_source(_id) > value_source(_nodeQueue[0]))
    {
        _nodeQueue.insert(_nodeQueue.begin(), _id);
        // cout<<"insert before begin "<<value_source(_id)<<" "<<value_source(_nodeQueue[0])<<endl;
    }
    else if(value_source(_id) < value_source(_nodeQueue[_nodeQueue.size()-1]))
    {
        _nodeQueue.push_back(_id);
        // cout<<"insert after end "<<value_source(_id)<<" "<<value_source(_nodeQueue[_nodeQueue.size()-1])<<endl;
    }
    else
    {
        int _start=0, _end=_nodeQueue.size()-1, half=0;
        while((_end-_start)>1)
        {
            half = (_end+_start) / 2;
            if(value_source(_id) > value_source(_nodeQueue[half]) )
            {
                _end = half;
            }
            else
            {
                _start = half;
            }
        }
        _nodeQueue.insert(_nodeQueue.begin()+_end, _id);
        // cout<<"insert middle"<<endl;
    }
    // test cout
    // cout<<"queue: ";
    // for(int i=0; i<_nodeQueue.size(); ++i)
    // {
    //     cout<<_nodeQueue[i]<<"("<<value_source(_nodeQueue[i])<<")###";
    // }
    // cout<<endl;
}

void TopoMetric_c::backUpNeibor(Eigen::Vector3d node_xyz, vector<Eigen::Vector3d> neibor_xyz_vec)
{
    //first check if there already exists nearby backup noedes
    for(int i=0; i<bcnode_xyz_vec.size(); ++i){
        double dist = (node_xyz - bcnode_xyz_vec[i]).norm();
        if(dist < 2){
            cout<<"[TM]node ("<<node_xyz(0)<<","<<node_xyz(1)<<","<<node_xyz(2)
               <<") will not back up."<<endl;
            bcedge_vec.push_back(Eigen::Vector2i(last_add_vertice,i));
            last_add_vertice = i;
            return;
        }
    }
    bcnode_xyz_vec.push_back(node_xyz);
    bcneibor_xyz_vecs.push_back(neibor_xyz_vec);
    if(last_add_vertice >= 0){
        bcedge_vec.push_back(Eigen::Vector2i(last_add_vertice,bcnode_xyz_vec.size()-1));
    }
    last_add_vertice = bcnode_xyz_vec.size()-1;
    ofstream write(file_path+"backup_pos.txt");
    cout<<"[TM]back up "<<bcnode_xyz_vec.size()<<" position to "<<file_path<<endl;
    for(int i=0; i<bcnode_xyz_vec.size(); ++i){
        write <<bcnode_xyz_vec[i](0)<<" "<<bcnode_xyz_vec[i](1)<<" "<<bcnode_xyz_vec[i](2)<<endl;
    }
    write.close();
}

void TopoMetric_c::showGraphData()
{
    cout<<"[topometric]show graph data:(zero node is "<<id_near_zero_node
       <<", confirmed nodes "<<size_confirm_nodes<<")"<<endl;
    for(int i=0; i<nodes_vec.size(); ++i)
    {
        cout<<"node: "<<i<<"("<<nodes_vec[i].xyz(0)<<","<<nodes_vec[i].xyz(1)<<","<<nodes_vec[i].xyz(2)<<"); neibor: ";
        for(int j=0; j<nodes_vec[i].id_neibor.size(); ++j)
        {
            cout<<nodes_vec[i].id_neibor[j]<<" ";//<<"("<<nodes_vec[i].node_neibor[j]->get_id_self()<<" addr "<<nodes_vec[i].node_neibor[j]<<") ";
        }
        cout<<endl;
    }
}


TopoMetric_c& TopoMetric_c::operator=(const TopoMetric_c& tm)
{
    id_near_zero_node = tm.id_near_zero_node;
    size_confirm_nodes = tm.size_confirm_nodes;
    bcnode_xyz_vec = tm.bcnode_xyz_vec;
    bcneibor_xyz_vecs = tm.bcneibor_xyz_vecs;
    nodes_vec = tm.nodes_vec;
    adjacent_mat = tm.adjacent_mat;
    visited_edge_mat = tm.visited_edge_mat;
    file_path = tm.file_path;
    z_lowb = tm.z_lowb;
    z_upb = tm.z_upb;
    y_lowb = tm.y_lowb;
    y_upb = tm.y_upb;
    x_lowb = tm.x_lowb;
    x_upb = tm.x_upb;
    file_path = tm.frame_id;
    return *this;
}

void TopoMetric_c::transform(Eigen::Quaterniond _q, Eigen::Vector3d _t)
{
    for(int i=0; i<nodes_vec.size(); ++i)
    {
        nodes_vec[i].xyz = _q.inverse() * (nodes_vec[i].xyz - _t);
    }
}
