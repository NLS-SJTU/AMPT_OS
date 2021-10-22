#ifndef TOPOMETRIC_H
#define TOPOMETRIC_H

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/tokenizer.hpp> 
#include <boost/lexical_cast.hpp> 


using namespace std;

class Node_c
{
public:
    Node_c(){}
    Node_c(float _x, float _y, float _z, int _id_self);
    ~Node_c();

    // set coordinate and id of node
    void setxyzid(float _x, float _y, float _z, int _id_self);
    void setxyzid(Eigen::Vector3d _xyz, int _id_self);

    // add edge with other node with _id_neibor
    void addLink(int _id_neibor, float _dist);//, Node_c* _node_neibor

    // remove edge with other node
    void removeLink(int _id_neibor);

    int get_id_self(){return id_self;}

    // coordinate of this node
    Eigen::Vector3d xyz;
    double norm;

    //edge between this node and neibor node
    vector<int> id_neibor;

    //store distance of neibor, coresponding to id_neibor
    vector<float> dist_neibor;

    //store whether this direction of edge is visited
    vector<bool> visited_neibor;
private:
    int id_self;
};

class TopoMetric_c
{
public:
    TopoMetric_c(){}
    TopoMetric_c(string _frame_id);
    ~TopoMetric_c(){}

    void clear(){nodes_vec.clear();}

    // save graph to file
    void saveGraph(string fname);

    // read graph from filepath folder, files named as node.txt and edge.txt
    bool readAndContructTopoMetric(const string& filepath);

    TopoMetric_c& operator=(const TopoMetric_c& tm);

    // transform all nodes to a new frame
    void transform(Eigen::Quaterniond _q, Eigen::Vector3d _t);

    // return number of nodes, for a visited graph use size_confirm_nodes
    int size(){return nodes_vec.size();}

    // return node with fake index _i(not same with id, but if you did not insert or delete any nodes, they are the same)
    Node_c* getNode(int _i);

    // modify the position of one node
    void modifyNode(int _id, Eigen::Vector3d _new_xyz);

    // search the nearest node to input
    int searchNearestNode(Eigen::Vector3d _xyz);
    int searchNearestNode(Eigen::Vector3d _xyz, double& nearest_dist);

    // search if the nearest node to _xyz is id_initnode
    int searchNearbyNode(Eigen::Vector3d _xyz, int id_initnode);

    void initZeroNode(float _x, float _y, float _z, int _id);

    // given id of nodes, return groups of them by thier neibor relation
    void splitNodesByNeibor(vector<int> nodes, vector<vector<int> > &split_nodes, int min_group_size);

    // find nodes linked together
    void findGroupNodes(vector<int> &nodes, int index_in_vec, vector<int> &one_group);

    // set edge as visited
    void setVisitedEdge(int id_node_1, int id_node_2, double multiple=2.0);

    //
    bool isEdgeVisited(int id_node_1, int id_node_2);

    bool edgeExist(int id1, int id2);

    double getDistance(int id1, int id2);

    // reset all edges to original unvisited status
    void resetEdgeStatus();

    // caculate distance using dijkstra, return nodes id ranked by distance to start node
    bool Dijkstra(int _id_start, vector<int>& near_to_far_nodes);
    // get path from the dijk result, 0 is end node, last is start node
    void getPathFromDijkstra(int _id_end, vector<int>& path_vec);
    // insert node to a queue for dijk
    void insertSortByDistTotal(int _id, vector<int> &_nodeQueue, Eigen::VectorXd value_source);

    // shou graph data
    void showGraphData();

    // the node that is near zero point
    int id_near_zero_node;
    // number of nodes which are not back up nodes(back up nodes should not be matched as main nodes)
    int size_confirm_nodes;
    // back up multiple nodes
    void backUpNeibor(Eigen::Vector3d node_xyz, vector<Eigen::Vector3d> neibor_xyz_vec);
    void clearBackUpNeibor(){bcnode_xyz_vec.clear(); bcneibor_xyz_vecs.clear();}
    // center nodes of back up neibor
    vector<Eigen::Vector3d> bcnode_xyz_vec;
    vector<Eigen::Vector2i> bcedge_vec;
    // coresponding back up neibor
    vector<vector<Eigen::Vector3d> > bcneibor_xyz_vecs;

    // nodes of the graph
    vector<Node_c> nodes_vec;
    // adjacent matrix
    Eigen::MatrixXd adjacent_mat;
    // label visited edge, -1 is no edge, 1 is normal edge, 2 is visited edge
    Eigen::MatrixXd visited_edge_mat;

    // Graph backup_nodegraph;
    int last_add_vertice;

    // file path
    string file_path;

    // for dijkstra
    Eigen::VectorXd dist_to_start;
    Eigen::VectorXi id_from_where;
    int id_start;

    // frame id
    string frame_id;

    // range of  nodes
    float x_upb, x_lowb, y_upb, y_lowb, z_upb, z_lowb;
};



#endif
