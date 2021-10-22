
#include "RCPP.h"

using namespace std;

void RCPP::readParam(string paramfile)
{
    cout<<"[RCPP]read param"<<endl;
    cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
    fsSettings["ampt"]["RCPP"]["undirected"] >> undirected;
    cout<<"undirected:"<<undirected<<endl;
}

void RCPP::calRCPPPath(TopoMetric_c* _graph, int start_id, int end_id, vector<int>& _path_node_vec)
{
    cout<<"[RCPP]start RCPP path calculation..."<<endl;
    // turn my class GraphT to the class Graph rcpp can use
    graph_h.clear();
    // add vertex
    for(int i=0; i<_graph->size(); ++i)
    {
        Node_c *nt = _graph->getNode(i);
        graph_h.addVertex(i, Eigen::Vector2f(nt->xyz(0), nt->xyz(1)));
    }
    // add edge
    set<int> visited_edges;
    float sum_all_edges = 0, sum_visited_edge = 0;
    for(int i=0; i<_graph->size(); ++i)
    {
        Node_c *nt = _graph->getNode(i);
        for(int j=0; j<nt->id_neibor.size(); ++j)
        {
            if((not undirected) || nt->id_neibor[j] > i){
                cout<<" edge "<<i<<", "<<nt->id_neibor[j];
                // from, to, undirected, cost
                Graph::Edge* edge_n = graph_h.addEdge(i, nt->id_neibor[j], undirected, nt->dist_neibor[j]);
                sum_all_edges += nt->dist_neibor[j];
                // visited edges
                if(_graph->visited_edge_mat(i, nt->id_neibor[j]) > 1){
                    visited_edges.insert(edge_n->id());
                    sum_visited_edge += nt->dist_neibor[j];
                }
            }
        }
    }
    cout<<"all edges length: "<<sum_all_edges<<", visited edge length: "<<sum_visited_edge
       <<", coverage edges length: "<<(sum_all_edges - sum_visited_edge)<<endl;

    // calculate path
    vector<int> circuit;
    int goalId;
    float shortest_path_len = 10000000;
    if(end_id < 0){
        //find shortest path comparing all end nodes
        RoutingProblem routing;
        float tmp_path_len;
        vector<int> tmp_circuit;
        for(int tmpgoalId=0; tmpgoalId<graph_h.vertices().size(); ++tmpgoalId)
        {
            routing.init(graph_h, start_id, tmpgoalId, visited_edges);
            tmp_circuit = routing.solve();
            tmp_path_len = calCircuitLen(tmp_circuit);
            if(tmp_path_len < shortest_path_len){
                goalId = tmpgoalId;
                shortest_path_len = tmp_path_len;
                circuit = tmp_circuit;
            }
        }
    }
    else{
        RoutingProblem routing;
        goalId = end_id;
        routing.init(graph_h, start_id, goalId, visited_edges);
        // path of edges
        circuit = routing.solve();
        shortest_path_len = calCircuitLen(circuit);
    }

    // return path of nodes
    _path_node_vec = graph_utils::pathEdgesToVertices(circuit, graph_h, start_id);
    cout<<"[RCPP]path nodes and edges (start "<<start_id<<", end "<<goalId<<", length "
       <<shortest_path_len<<"m):"<<endl;
    for(int i=0; i<circuit.size(); ++i)
    {
        cout<<"n"<<_path_node_vec[i]<<" -(e"<<circuit[i]<<")> ";
    }
    cout<<"n"<<_path_node_vec[_path_node_vec.size()-1]<<endl;

    cout<<"[RCPP]RCPP path calculation done!"<<endl;
}

float RCPP::calCircuitLen(vector<int> edge_id_vec)
{
    float ret = 0;
    Graph::EdgeIDMap eidmap = graph_h.edges();
    for(int i=0; i<edge_id_vec.size(); ++i)
    {
        ret += eidmap[edge_id_vec[i]]->cost();
    }
    return ret;
}
