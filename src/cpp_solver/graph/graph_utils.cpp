#include "graph_utils.h"
#include "iostream"
#include "paths/shortest_paths.h"



namespace graph_utils
{
    using namespace std;

    float circuitLength(Graph graph, std::vector<int> circuit)
    {
        float cost = 0;
        for(int i=0; i<circuit.size(); ++i)
        {
            int edgeId = circuit[i];
            Graph::Edge* edgeprt = graph.edge(edgeId);
            if(edgeprt != nullptr && edgeprt->cost() < 9999999){
                cost += edgeprt->cost();
            }
        }
        return cost;
    }

    float eulerianCost(Graph graph, std::set<int> skipEdges)
    {

        float cost = 0;

        for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end(); it++) {
            Graph::Edge* e = it->second;
            // do not add virtual edge because it is too large
            if (std::find(skipEdges.begin(), skipEdges.end(), e->id()) == skipEdges.end() && e->cost() < 9999999)
                cost += e->cost();
        }

        return cost;

    }


    GraphType detectGraphType(Graph graph)
    {
        bool hasEdge = false;
        bool hasArc = false;

        Graph::EdgeIDMap edges = graph.edges();
        for (Graph::EdgeIDMap::const_iterator it = edges.begin(); it != edges.end(); it++){
            Graph::Edge* e = it->second;

            if (e->undirected()){
                hasEdge = true;
            }
            else {
                hasArc = true;
            }

        }

        if (!hasArc)
            return UNDIRECTED;
        else if (hasArc && !hasEdge)
            return DIRECTED;
        else
            return MIXED;

    }


    void refineEdges(Graph* graph, std::set<int> edges)
    {
        std::vector<std::pair<int,int>> optsExtremes;

        Graph noOtpGraph = *graph;

        for (int edgeId : edges){
            Graph::Edge* edgeHere = graph->edge(edgeId);
            Graph::Edge* otp = noOtpGraph.edge(edgeId);
//            std::cout<<"otp edge: "<<edgeId<<", ehere "<<(edgeHere == nullptr)<<", otp "<<(otp == nullptr)<<endl;
            if (otp == nullptr){
                continue;
            }
            noOtpGraph.removeEdge(otp);
        }

        for (int edgeId : edges){

            Graph::Edge* edgeHere = graph->edge(edgeId);
            Graph::Edge* edgeThere = noOtpGraph.edge(edgeId);
//            std::cout<<"otp edge: "<<edgeId<<", ehere "<<(edgeHere == nullptr)<<", ethere "<<(edgeThere == nullptr)<<endl;
//            if (edgeThere == nullptr){
            if(edgeHere == nullptr){
                continue;
            }
            graph->removeEdge(edgeHere);
            
            int fromId = edgeHere->from()->id();
            int toId = edgeHere->to()->id();
            bool undirected = edgeHere->undirected();

            std::vector<int> path = shortest_paths::pathDijkstra(noOtpGraph, fromId, toId);

            for (int eId : path){
                Graph::Edge* eP = noOtpGraph.edge(eId);
                Graph::Vertex* fromV = eP->from();
                Graph::Vertex* toV = eP->to();

//                Graph::Edge* duplicatedEdge = noOtpGraph.addEdge(fromV->id(), toV->id(), eP->undirected(), eP->cost(), eP->capacity());
                Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), eP->undirected(), eP->cost(), eP->capacity());
                duplicatedEdge->setParentId(eP->parentId());
            }


        }

    }

    std::vector<int> pathEdgesToVertices(std::vector<int> edgesPath, Graph g, int startVertexId)
    {


        std::vector<int> verticesPath;

        verticesPath.push_back(startVertexId);

        for (int eId : edgesPath){
            Graph::Edge* e = g.edge(eId);
            int nextId = (e->from()->id() == verticesPath.back()) ? e->to()->id() : e->from()->id();

            verticesPath.push_back(nextId); 

        }

        return verticesPath;

    }





std::vector<std::vector<int>> tarjanConnectedComponents(Graph graph)
{

	std::map<int, int> discoveryStep;
	std::map<int, int> lowIndices;
	std::map<int, bool> markedVertices;
	std::stack<int> connectedAncestors;
	std::vector<std::vector<int>> connectedComponents;

	int step = 0;

	for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it++) {
		discoveryStep.insert(std::pair<int, int>(it->first, -1));
		lowIndices.insert(std::pair<int, int>(it->first, -1));
		markedVertices.insert(std::pair<int, bool>(it->first, false));

	}


	for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it++) {
		if (discoveryStep.at(it->first) == -1)
			tarjanConnectedComponentsRecursion(graph, it->first, &discoveryStep, &lowIndices, &connectedAncestors, &markedVertices, &step, &connectedComponents);
	}



	return connectedComponents;

}


void tarjanConnectedComponentsRecursion(Graph graph, int vertexID, std::map<int, int>* discoveryStep, std::map<int, int>* lowIndices, std::stack<int>* connectedAncestors, std::map<int, bool>* markedVertices, int* step, std::vector<std::vector<int>>* connectedComponents)
{

	discoveryStep->at(vertexID) = lowIndices->at(vertexID) = (*step)++;
	connectedAncestors->push(vertexID);
	markedVertices->at(vertexID) = true;

	std::vector<int> connectedComponent;

	Graph::EdgeSet exitingEdges = graph.vertex(vertexID)->exitingEdges();

	for (Graph::EdgeSet::iterator it = exitingEdges.begin(); it != exitingEdges.end(); it++) {
		int vertexAdjacentID;
		if ((*it)->from()->id() != vertexID)
			vertexAdjacentID = (*it)->from()->id();
		else
			vertexAdjacentID = (*it)->to()->id();

		if (discoveryStep->at(vertexAdjacentID) == -1) {

			tarjanConnectedComponentsRecursion(graph, vertexAdjacentID, discoveryStep, lowIndices, connectedAncestors, markedVertices, step, connectedComponents);
			lowIndices->at(vertexID) = std::min(lowIndices->at(vertexID), lowIndices->at(vertexAdjacentID));
		}

		else if (markedVertices->at(vertexAdjacentID) == true) {
			lowIndices->at(vertexID) = std::min(lowIndices->at(vertexID), discoveryStep->at(vertexAdjacentID));
		}
	}


	int w = 0;
	if (lowIndices->at(vertexID) == discoveryStep->at(vertexID)) {
		while (connectedAncestors->top() != vertexID) {
			w = connectedAncestors->top();
			markedVertices->at(w) = false;
			connectedComponent.push_back(w);
			connectedAncestors->pop();

		}
		w = connectedAncestors->top();
		markedVertices->at(w) = false;
		connectedComponent.push_back(w);
		connectedComponents->push_back(connectedComponent);
		connectedAncestors->pop();

	}

}





    void printVerticesInfo(Graph g)
    {

        std::cout<<"Printing vertices info ------->"<<std::endl;

        for (Graph::VertexIDMap::const_iterator it = g.vertices().begin(); it != g.vertices().end(); it++){
            Graph::Vertex* v = it->second;
            std::cout<< "Vertex " << v->id() << " at " << v->position().transpose();
            if (v->enteringEdges().size() > 0){
                std::cout <<" +1:";
                for (Graph::Edge* e : v->enteringEdges()){
                    std::cout<<" e " << e->from()->id()<< " " << e->to()->id();
                }
            }
            if (v->exitingEdges().size() > 0){
                std::cout <<" -1:";
                for (Graph::Edge* e : v->exitingEdges()){
                    std::cout<<" e " << e->from()->id()<< " " << e->to()->id();
                }
            }
            std::cout<<std::endl;
        }
    }


    void printEdgesInfo(Graph g)
    {

        std::cout<<"Printing edges info ------->"<<std::endl;

        for (Graph::EdgeIDMap::const_iterator it = g.edges().begin(); it != g.edges().end(); it++){
            Graph::Edge* e = it->second;
            std::cout<< "Edge " << e->from()->id() << " "<< e->to()->id() << (e->undirected() ? " Undirected" : " Directed") << " Cost: " << e->cost() << " Capacity: " << (e->capacity() == INT_MAX ? std::numeric_limits<float>::infinity() : e->capacity()) << " Id " << e->id();
            if (e->parentId() != e->id()){
                std::cout << " ParentId: " << e->parentId();
            }
            std::cout<<std::endl;
        }
    }







}
