#include "eulerian_extension.h"
#include "paths/shortest_paths.h"
#include "paths/network_flow.h"
#include "branch_bound.h"
#include <queue>

namespace eulerian_extension
{


        
    Graph extend(Graph& graph, graph_utils::GraphType type, std::set<int> travelEdges, int startId, int endId)
    {

        if (travelEdges.empty()){

            heuristicExtension(&graph, type, travelEdges);

        }
        else {

            if (type == graph_utils::MIXED){

            }
            else {
                // add best otps to graph
                std::set<int> otps = eulerian_extension::ruralSolver(&graph, travelEdges, type, startId, endId);
//                std::cout<<"graphsize before refine: "<<graph.vertices().size()<<", "<<graph.edges().size()<<std::endl;
//                std::cout<<"before refine:";
//                graph.show();
                // change otps back to original edges
                graph_utils::refineEdges(&graph, otps);
//                std::cout<<"after refine:";
//                graph.show();
                // now edges except travelEdges are all needed
                heuristicExtension(&graph, type, travelEdges);
                // test
//                RoutingProblem routing;
//                routing.init(graph, startId, endId, travelEdges);
//                std::cout<<"after remove:";
//                std::vector<int> circuit = routing.hierholzerSolver(graph, endId);
//                std::cout<<"graphsize after refine: "<<graph.vertices().size()<<", "<<graph.edges().size()<<std::endl;
            }
        }

        return graph;

    }


    void heuristicExtension(Graph* graph, graph_utils::GraphType type, std::set<int> travelEdges)
    {
            
         if (type == graph_utils::DIRECTED){

            eulerian_extension::simmetryHeuristic(graph, travelEdges);
        }                
        else if (type == graph_utils::UNDIRECTED){
            eulerian_extension::evenDegreeHeuristic(graph, travelEdges);

        }
        else if (type == graph_utils::MIXED){
        
        }

    }


    void evenDegreeHeuristic(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> oddDegreeVertices;
        Graph allEdgesGraph = *graph;

        for (Graph::EdgeIDMap::iterator it = allEdgesGraph.edges().begin(); it != allEdgesGraph.edges().end(); it++){
            Graph::Edge* e = it->second;
            e->setUndirected(true);
        }

        for (Graph::VertexIDMap::iterator itV = allEdgesGraph.vertices().begin(); itV != allEdgesGraph.vertices().end(); itV++){

            int degree = 0;
            Graph::Vertex* v = itV->second;
            for (Graph::EdgeSet::iterator itE = v->edges().begin(); itE != v->edges().end(); itE++){
                Graph::Edge* e = *itE;
                if (notRequiredEdges.find(e->id()) == notRequiredEdges.end()){
                    degree ++;
                }
            }

            if (degree % 2 != 0){
                oddDegreeVertices.push_back(v->id());
            }
        }

        assert (oddDegreeVertices.size() % 2 == 0);

        if (oddDegreeVertices.empty()){
            return;
        }

        std::map<std::pair<int, int>, float> D;
        std::map<std::pair<int, int>, std::vector<int>> P;


        std::vector<int> verticesVector;
        for (auto va = allEdgesGraph.vertices().begin(); va != allEdgesGraph.vertices().end(); va++){
            verticesVector.push_back(va->first);
        }

        shortest_paths::mapFloydWarshall(allEdgesGraph, verticesVector ,&D, &P);
        //shortest_paths::mapDijkstra(allEdgesGraph, oddDegreeVertices, oddDegreeVertices, &D, &P);

        for (int vId : oddDegreeVertices){
            D[std::make_pair(vId, vId)] = std::numeric_limits<float>::infinity(); 
        }

        std::vector<int> bestAssignment = network_flow::naivePairsAssignment(oddDegreeVertices, D);


        for (int j = 0; j < bestAssignment.size(); j += 2){
            
            int firstElement = bestAssignment[j];
            int firstId = oddDegreeVertices[firstElement];
            int secondElement = bestAssignment[j + 1];
            int secondId = oddDegreeVertices[secondElement];

            assert (firstId >= 0 && secondId >= 0);

            std::vector<int> path = P[std::make_pair(firstId, secondId)];

            for (int eId : path){
                Graph::Edge* e = graph->edge(eId);
                Graph::Vertex* fromV = graph->vertex(e->from()->id());
                Graph::Vertex* toV = graph->vertex(e->to()->id());
                Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
                duplicatedEdge->setParentId(e->parentId());
            }

        }

    }

    std::map<int, int> simmetryHeuristic(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> verticesID;
        std::map<int, int> addedEdges;
        
        std::map<int, int> verticesSupplyMap = network_flow::computeVerticesSupplyMap(*graph, notRequiredEdges);

        std::map<std::pair<int,int>, int> flowMatrix = network_flow::minCostMaxFlowMatching(*graph, verticesSupplyMap);

        for (std::map<std::pair<int,int>, int>::iterator itMap = flowMatrix.begin(); itMap != flowMatrix.end(); itMap++) {

            int count = itMap->second;

            assert (count >= 0);

            std::pair<int,int> element = itMap->first;
            Graph::Vertex* fromV = graph->vertex(element.first);
            Graph::Edge* e = graph->edge(element.second);
            Graph::Vertex* toV = (e->from()->id() == fromV->id()) ? e->to() : e->from();
            for (int i = 0; i < count; i ++){
                Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
                duplicatedEdge->setParentId(e->parentId());
            }

            if (count != 0){
                addedEdges[e->id()] = count;
            }		

        }

        return addedEdges;
    }



    std::tuple<std::vector<int>, std::vector<int>, std::vector<int>> fredericksonInOutDegree(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> directedElements;
        std::vector<int> undirectedElements;
        std::vector<int> addedElements;

        Graph g2 = *graph;
        Graph g3 = *graph;

        //Loop on original edges, since I'm adding edges in g2. quadruplicate edges and add arcs to directed elements
        for (Graph::EdgeIDMap::iterator itEdge = graph->edges().begin(); itEdge != graph->edges().end(); itEdge++) {
            Graph::Edge* e =itEdge->second;
            int edgeId = itEdge->first;
            
            if (!e->undirected()) {
                directedElements.push_back(edgeId);
                continue;
            }
                
            int fromId = e->from()->id();
            int toId = e->to()->id();
            //Substitute old edge with 2 pairs of arcs
            Graph::Edge* newArc;

            newArc = g3.addEdge(fromId, toId, false, e->cost(), e->capacity());
            newArc->setParentId(edgeId);
            newArc = g3.addEdge(toId, fromId, false, e->cost(), e->capacity());
            newArc->setParentId(edgeId);
            newArc = g3.addEdge(fromId, toId, false, 0.0, 1);
            newArc->setParentId(edgeId);
            newArc = g3.addEdge(toId, fromId, false, 0.0, 1);
            newArc->setParentId(edgeId);
            g3.removeEdge(g3.edge(edgeId));
        }

        
        std::map<int, int> newEdges = simmetryHeuristic(&g3, notRequiredEdges);


        for (std::map<int, int>::iterator itMap = newEdges.begin(); itMap != newEdges.end(); itMap++) {
            //Add new edges to original graph
            int thisId = itMap->first;
            int countCopies = itMap->second;
            Graph::Edge* e = g3.edge(thisId);
            //Check that I'm considering a 0 cost directed edge
            if (countCopies == 1 && e->cost() == 0 && e->capacity() == 1) {
                //Check that I have not used the opposite direction 0 cost edge
                bool oppositeDirection = false;
                for (std::map<int, int>::iterator itEdge = newEdges.begin(); itEdge != newEdges.end(); itEdge++) {
                    Graph::Edge* e2 = g3.edge(itEdge->first);

                    if (itEdge->second != 1 || e2->parentId() != e->parentId() || e2->cost() != 0 || e2->capacity() != 1)
                        continue;

                    if (e2->from()->id() == e->to()->id() && e2->to()->id() == e->from()->id()) {
                        oppositeDirection = true;
                        break;
                    }
                }

                if (!oppositeDirection) {
                    Graph::Vertex* vFrom = g2.vertex(e->from()->id());
                    Graph::Vertex* vTo = g2.vertex(e->to()->id());
                    Graph::Edge* edgeOriginal = g2.edge(e->parentId());
                    Graph::Edge* new_e = g2.addEdge(vFrom, vTo, false, edgeOriginal->cost(), edgeOriginal->capacity());
                    new_e->setParentId(e->parentId());
                    g2.removeEdge(edgeOriginal);
                    directedElements.push_back(new_e->id());

                }
                //If the opposite direction has been used, I do nothing.


            }
            else {
                Graph::Vertex* vFrom = g2.vertex(e->from()->id());
                Graph::Vertex* vTo = g2.vertex(e->to()->id());
                for (int i = 0; i < countCopies; i++) {
                    //This works as duplicateLinks
                    Graph::Edge* ed = g2.addEdge(vFrom, vTo, false, e->cost(), e->capacity());
                    ed->setParentId(e->parentId());
                    directedElements.push_back(ed->id());
                    addedElements.push_back(ed->id());
                }
            }

        }

        //Insert in undirected set the remained edges
        for (Graph::EdgeIDMap::iterator it = g2.edges().begin(); it != g2.edges().end(); it++) {
            Graph::Edge* e = it->second;
            if (e->undirected()) {
                undirectedElements.push_back(it->first);
            }

        }


        *graph = g2;

        return std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>(directedElements, undirectedElements, addedElements);

    }




    std::set<int> ruralSolver(Graph* graph, std::set<int> notRequiredEdges, graph_utils::GraphType type, int startId, int endId)
    {
        /// should fix a bug: if the graph is separated into several parts by visited edges,
        /// original method will just go in one part without going the other parts.
        float minCoverCost = graph_utils::eulerianCost(*graph, notRequiredEdges);
//        std::cout<<"min circuit length: "<<minCoverCost<<std::endl;
        std::set<int> originnotRequiredEdges = notRequiredEdges;

        //Vertices incident only to required edges
        std::vector<int> requiredVertices;
        //Vertices incident only to already visited edges
        std::vector<int> travelVertices;
        //Vertices incident to at least 1 required and 1 visited edges
        std::vector<int> borderVertices;


        for (Graph::VertexIDMap::const_iterator itVertices = graph->vertices().begin(); itVertices != graph->vertices().end(); itVertices++) {
            bool incidentToRequired = false;
            bool incidentToVisited = false;
            Graph::Vertex* v = itVertices->second;

            for (Graph::EdgeSet::iterator itEdges = v->edges().begin(); itEdges != v->edges().end(); itEdges++) {
                Graph::Edge* e = *itEdges;

                if (notRequiredEdges.find(e->id()) != notRequiredEdges.end())
                    incidentToVisited = true;
                else 
                    incidentToRequired = true;
                
            }

            if (incidentToVisited && incidentToRequired) {
                borderVertices.push_back(itVertices->first);
            }
            else if (incidentToRequired) {
                requiredVertices.push_back(itVertices->first);
            }
            else if (incidentToVisited) {
                travelVertices.push_back(itVertices->first);
            }

        }

        std::map<std::pair<int, int>, float> shortestPathsD;
        std::map<std::pair<int, int>, std::vector<int>> optimalTravelPaths;

        //Compute all shortest paths between border vertices
        shortest_paths::mapDijkstra(*graph, borderVertices, borderVertices, &shortestPathsD, &optimalTravelPaths);

        //Remove paths containing not only visited edges
        /// --> want to leave only visited edge path
        /// want path between border vertices only across visited edges
        for (int fromId : borderVertices) {
//            std::cout<<"bordervertice: "<<fromId<<std::endl;
            for (int toId : borderVertices) {

                if (fromId == toId) {
                    optimalTravelPaths.erase(std::make_pair(fromId, toId));
                    shortestPathsD.at(std::make_pair(fromId, toId)) = std::numeric_limits<float>::infinity();
                    continue;
                }
//                std::cout<<"from "<<fromId<<" to "<<toId<<", ";
                std::vector<int> path = optimalTravelPaths.at(std::make_pair(fromId, toId));
                for (unsigned i = 0; i < path.size(); i++) {
                    ///problem?? the original code means all path edges should not in visited edges(notrequirededges)
                    /// == end ---> not in this set ---> is not visited edge ---> remove
                    /// != end ---> in this set ---> is visited edge
//                    if (notRequiredEdges.find(path[i]) != notRequiredEdges.end()){
                    if (notRequiredEdges.find(path[i]) == notRequiredEdges.end()){
//                        std::cout<<"(remove for edge "<<path[i]<<" from "<<graph->edge(path[i])->from()->id()
//                           <<" to "<<graph->edge(path[i])->to()->id()<<")";
                        optimalTravelPaths.erase(std::make_pair(fromId, toId));
                        shortestPathsD.at(std::make_pair(fromId, toId)) = std::numeric_limits<float>::infinity();
                        break;
                    }
                }
//                std::cout<<std::endl;
            }
        }


        std::set<int> otpEdges;
        Graph optimalGraph = *graph;

        bool undirected = type == graph_utils::UNDIRECTED;
        //Add artificial OTPs edges
//        std::cout<<"opts edges:"<<std::endl;
        for (std::map<std::pair<int, int>, std::vector<int>>::const_iterator it = optimalTravelPaths.begin(); it != optimalTravelPaths.end(); it++) {
            
            std::vector<int> path = it->second;

            if (path.empty()){
                continue;
            }

            int fromId = it->first.first;
            int toId = it->first.second;

            Graph::Edge* optEdge = optimalGraph.addEdge(fromId, toId, undirected, 0);

            /* FOR SOME REASON I CAN'T REDUCE THE PROBLEM
            for (int edgeId : path){
                optimalGraph.removeEdge(optimalGraph.edge(edgeId));
            }
            */

            otpEdges.insert(optEdge->id());
            notRequiredEdges.insert(optEdge->id());
//            std::cout<<"edge id:"<<optEdge->id()<<", n"<<optEdge->from()->id()<<" n"<<optEdge->to()->id()<<std::endl;
        }

        std::priority_queue<BranchNBoundStruct, std::vector<BranchNBoundStruct>, std::greater<BranchNBoundStruct>> pq;

        pq.push(BranchNBoundStruct(optimalGraph, otpEdges));

        std::vector<int> circuit;

//        BranchNBoundStruct currentStruct = pq.top();
//        heuristicExtension(&currentStruct.graph, type, notRequiredEdges);
        RoutingProblem routing;
//        routing.init(currentStruct.graph, startId, endId, notRequiredEdges);
//        circuit = routing.hierholzerSolver(currentStruct.graph, endId);
//        currentStruct.cost = graph_utils::circuitLength(currentStruct.graph, circuit);
//        std::cout<<"init cost with all otp=0: "<<currentStruct.cost<<std::endl;

        while (!pq.empty()) {
            //Extract top (lower cost) element from the priority queue
            BranchNBoundStruct currentStruct = pq.top();
            pq.pop();
            //If the lower cost element has no unlabelled OTP edges -> I have found a solution
            if (currentStruct.remainedElements.empty()) {
                if(currentStruct.cost < minCoverCost) continue;

                *graph = currentStruct.graph;
                // do not extend here!!!
                // extend after refine edges!!!
//                std::cout<<"end before extend:"<<std::endl;
//                currentStruct.graph.show();
//                heuristicExtension(&currentStruct.graph, type, originnotRequiredEdges);
//                std::cout<<"end after extend:"<<std::endl;
//                currentStruct.graph.show();
//                std::cout<<"end cost: "<<currentStruct.cost<<std::endl;
//                std::cout<<"end2 cost: "<<graph_utils::eulerianCost(*graph, originnotRequiredEdges)<<std::endl;
//                routing.init(currentStruct.graph, startId, endId, originnotRequiredEdges);
//                std::cout<<"-----------after solver";
//                circuit = routing.hierholzerSolver(currentStruct.graph, endId);
                return otpEdges;
            }

            int branchEdgeId = Graph::UnassignedId;
            float realCost =  0;
            for (int edgeId : currentStruct.remainedElements) {

                Graph::Edge* e = currentStruct.graph.edge(edgeId);
                int fromId = e->from()->id();
                int toId =  e->to()->id();

                float c = shortestPathsD.at(std::make_pair(fromId, toId));

                if (c >= realCost){
                    realCost = c;
                    branchEdgeId = edgeId;
                }

            }

            Graph g;
            BranchNBoundStruct structA = currentStruct;
            Graph::Edge* branchEdgeA = structA.graph.edge(branchEdgeId);
//            std::cout<<"remove edge from "<<branchEdgeA->from()->id()<<" to "<<branchEdgeA->to()->id()<<std::endl;

            structA.graph.removeEdge(branchEdgeA);
            structA.remainedElements.erase(branchEdgeId);
            g = structA.graph;
//            std::cout<<">>>>>>>>>>before extend";
//            g.show();
            heuristicExtension(&g, type, originnotRequiredEdges);
//            std::cout<<"~~~~~~~~~~after extend";
//            g.show();
            // the cost should be final path length instead of eulerianCost
            // because euler circle might be not connected
            routing.init(g, startId, endId, originnotRequiredEdges);
//            std::cout<<"-----------after solver";
            circuit = routing.hierholzerSolver(g, endId);
            structA.cost = graph_utils::circuitLength(g, circuit);
//            std::cout<<"cost of Ag with otp: "<<structA.cost<<std::endl;

            /// do not use euler cost because it cannot handle seperated coverage parts
            /// the original bug starts here
//            structA.cost = graph_utils::eulerianCost(g, originnotRequiredEdges);
//            std::cout<<"cost of Ag with otp: "<<structA.cost<<std::endl;
//            if(structA.cost >= minCoverCost){
            pq.push(structA);
//            }

            BranchNBoundStruct structB = currentStruct;
            structB.graph.edge(branchEdgeId)->setCost(realCost);
            structB.remainedElements.erase(branchEdgeId);

            g = structB.graph;
//            std::cout<<">>>>>>>>>>before extend";
//            g.show();
            heuristicExtension(&g, type, originnotRequiredEdges);
//            std::cout<<"~~~~~~~~~~after extend";
//            g.show();

            routing.init(g, startId, endId, originnotRequiredEdges);
//            std::cout<<"-----------after solver";
            circuit = routing.hierholzerSolver(g, endId);
            structB.cost = graph_utils::circuitLength(g, circuit);
//            std::cout<<"cost of Bg with otp: "<<structB.cost<<std::endl;

//            structB.cost = graph_utils::eulerianCost(g, originnotRequiredEdges);
//            std::cout<<"add edge from "<<structB.graph.edge(branchEdgeId)->from()->id()<<" to "<<structB.graph.edge(branchEdgeId)->to()->id()<<std::endl;
//            std::cout<<"cost of Bg with otp: "<<structB.cost<<std::endl;
//            if(structB.cost >= minCoverCost){
            pq.push(structB);
//            }

        }

    }


}
