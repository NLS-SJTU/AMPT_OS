#include "Decision.h"

Decision::Decision(ros::NodeHandle& _nh, string paramfile):
    nh(_nh), mode(0), to_next_node(false), new_graph_loaded(false)
  , current_id_visited(-1), safedir_record_ok(false), first_target_ok(false)
  , to_target(false), just_load_graph(false), slam_pose_ok(false)
  , topomaploop_working(false)
  , p3atctrl_h(_nh)
  , prior_graph("prior_graph")
  , visited_graph("slam")
{
    readParam(paramfile);

    initROS();
}

void Decision::readParam(string paramfile)
{
    ROS_INFO("[AMPT]read param");
    cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
    fsSettings["ampt"]["decision"]["mode"] >> mode;
    fsSettings["ampt"]["decision"]["graph_matching_loc"] >> graph_matching_loc;
    fsSettings["ampt"]["decision"]["baseline_actloc_ctrl"] >> baseline_actloc_ctrl;
    fsSettings["ampt"]["prior_graph"] >> graph_path;
    prior_graph.readAndContructTopoMetric(graph_path);
    prior_graph.showGraphData();
    fsSettings["ampt"]["visited_graph"] >> graph_path;
    if(mode == 0){
        visited_graph.readAndContructTopoMetric(graph_path);
    }
    else{
        visited_graph.file_path = graph_path;
    }
    fsSettings["ampt"]["graph_path"] >> graph_path;
    fsSettings["ampt"]["decision"]["t_prior_visited_x"] >> t_prior_visited(0);
    fsSettings["ampt"]["decision"]["t_prior_visited_y"] >> t_prior_visited(1);
    fsSettings["ampt"]["decision"]["t_prior_visited_z"] >> t_prior_visited(2);
    fsSettings["ampt"]["decision"]["q_prior_visited_w"] >> q_prior_visited.w();
    fsSettings["ampt"]["decision"]["q_prior_visited_x"] >> q_prior_visited.x();
    fsSettings["ampt"]["decision"]["q_prior_visited_y"] >> q_prior_visited.y();
    fsSettings["ampt"]["decision"]["q_prior_visited_z"] >> q_prior_visited.z();
    fsSettings["ampt"]["decision"]["first_action"] >> first_action;
    fsSettings["ampt"]["decision"]["puzzle_type"] >> puzzle_type;
    new_graph_loaded = true;

    tpsim_h.readParam(paramfile);

    graphmatching_h.readParam(paramfile);
    graphmatching_h.initialize(&prior_graph);

    rcpp_h.readParam(paramfile);
    param_path = paramfile;

    ROS_INFO("[AMPT]read param done!");
}

void Decision::initROS()
{
    pose_slam_sub = nh.subscribe("/slam/posecov", 1, &Decision::poseSlamCB, this);
    topomap_savedone_sub = nh.subscribe("/topo/savedone", 1, &Decision::topomapSavedoneCB, this);
    visitgraph_savedone_pub = nh.advertise<std_msgs::String>("/visitgraph/savedone", 1);
    global_tar_pub = nh.advertise<nav_msgs::Path>("/far_point_to_point/path", 1);
    local_tar_pub = nh.advertise<nav_msgs::Path>("/point_to_point/path", 1);
    save_topomap_pub = nh.advertise<std_msgs::String>("/slam/totopo", 1);
    safezone_sub = nh.subscribe("/obstacle/safezone", 1, &Decision::safezoneCB, this);
}

void Decision::run()
{
    ros::Rate r(10);
    while(ros::ok()){
        switch(mode){
            case 0:test();break;
            case 1:runOriginal();break;
            case 2:runActSLAM();break;
            case 3:locSim();break;
            case 4:actlocSim();break;
            case 5:viterbiLocSim();break;
            case 6:actlocBaselineSim();break;
            case 7:matchself();break;
            case 8:RCPPtest();break;
            case 9:actlocOverlaySim();break;
            default:break;
        }
        if(mode == -1){break;}
        ros::spinOnce();
        r.sleep();
    }
}


void Decision::runOriginal()
{
    ROS_INFO("[AMPT]run original mode.");

    initFirstGraph();
    graphmatching_h.initMatchWithFixTrans(&visited_graph, t_prior_visited);

    updateCurrentID();
    rcpp_h.calRCPPPath(&prior_graph, current_id_prior, -1, path_vec);

    activeMappingGazebo();
    mode = -1;
}

void Decision::runActSLAM()
{
    ROS_INFO("[AMPT]run active localization mode.");
    logfile.open(graph_path+"actloclog.txt");
    // puzzle zone (indistinguishable subgraph) is calculated offline and read from file.
//    graphmatching_h.matchSelf();
    // init visited graph
    initFirstGraph();

    // init match
    graphmatching_h.initMatch(&visited_graph);
    updateCurrentID();

    // active localization
    activeLocalizationGazebo();

    // rpp path
    updateCurrentID();
    labelVisitedEdges();
    rcpp_h.calRCPPPath(&prior_graph, current_id_prior, -1, path_vec);

    //active mapping
    activeMappingGazebo();
    logfile.close();
    ROS_INFO("[AMPT]exit active localization mode.");
    mode = -1;
}

void Decision::activeLocalizationGazebo()
{
    doing_actloc = true;
    ROS_INFO("[AMPT]ative localization");
    ros::Rate r(10);
    Eigen::Vector3d pose_last_node(0,0,0);
    float yaw_to_last_node;
    to_crossing = true;
    to_target = true;
    just_load_graph = true;
    double dist_least_move = 2;
    while (ros::ok()){
        if(to_crossing){
            if(to_target && just_load_graph){
                cout<<endl;

                Eigen::VectorXi id_nodes_in_puzzle_zone_vec;
                int puzzle_zone_id = getPuzzleZone(id_nodes_in_puzzle_zone_vec);
                if(id_nodes_in_puzzle_zone_vec.norm() == 0) break;
                searchPathOutPuzzleZone(id_nodes_in_puzzle_zone_vec);

                pose_last_node = pose_slam;
                to_crossing = false;
                to_target = false;
                just_load_graph = false;
            }
        }
        else{
            if(to_target){
                // in real world, to_crossing should be confirmed by recognizing the crossing
                to_crossing = true;
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    doing_actloc = false;
}

void Decision::activeMappingGazebo()
{
    ROS_INFO("[AMPT]ative mapping");
    ros::Rate r(10);
    ith_node_in_path = 1;
    Eigen::Vector3d pose_last_node(0,0,0);
    to_crossing = true;
    to_target = true;
    just_load_graph = true;
    int id_tar_prior = -1;
    GraphMatching<TopoMetric_c, Node_c> gmh;
    gmh.readParam(param_path);
    gmh.initialize(&prior_graph);
    bool set_new_target;
    while (ros::ok()){
        // to_final_node
        if(to_target && ith_node_in_path > (path_vec.size()-1)){
            if(topomaploop_working){
                ros::spinOnce();
                r.sleep();
                continue;
            }
            else{
            ROS_INFO("[AMPT]to final node.");
            break;
            }
        }

        if(to_crossing){
            if(to_target && just_load_graph){
                cout<<endl;
                id_tar_prior = path_vec[ith_node_in_path];
                cout << "[AMPT]going to node(prior): "<< path_vec[ith_node_in_path]<< " . at path node: "<<ith_node_in_path<<endl;
//                xyz_tar_visited = graphmatching_h.getCoordInVisitedFromIDPrior(id_tar_prior);
                // we use real pose as target as we assume we can recognize the crossing
                xyz_tar_visited = q_prior_visited.toRotationMatrix().transpose() * (prior_graph.getNode(id_tar_prior)->xyz - t_prior_visited);
                cout << "[AMPT]going to (visited): "<< xyz_tar_visited(0)<<","<< xyz_tar_visited(1)<<","<< xyz_tar_visited(2)<< endl;
                goToLocalTarget(xyz_tar_visited(0), xyz_tar_visited(1), xyz_tar_visited(2));
                pose_last_node = pose_slam;
                to_crossing = false;
                to_target = false;
                just_load_graph = false;
                set_new_target = false;
                ++ith_node_in_path;
            }
        }
        else{
            if(to_target){
                to_crossing = true;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

void Decision::viterbiLocSim()
{
    ROS_INFO("[AMPT]sim viterbi localization mode.");
    viterbiloc_h.init(prior_graph);
    viterbiloc_h.reset(graph_path);
    tpsim_h.reset();
    Eigen::Vector3d action(0,0,0);
    for(int i=0; i<tpsim_h._sim_path_to_go.size(); ++i)
    {
        cout<<"========step "<<i<<endl;
        vector<Eigen::Vector3d> obs = tpsim_h.getObservation(-1);
        stimer_h.tik();
        Eigen::VectorXd prob_obs = graphmatching_h.calProbOfObs(obs);
        viterbiloc_h.update(prob_obs, action);
        stimer_h.tok();
        action = tpsim_h.moveNext();
    }

    Eigen::VectorXi path = viterbiloc_h.getBestPath(-1);
    // cout<<"best loc result: "<<path<<endl;
    tpsim_h.compareEstPathWithGT(path);
    mode = -1;
}

void Decision::locSim(){
    ROS_INFO("[AMPT]sim localization mode.");
    tpsim_h.reset();
//    prior_graph.showGraphData();
    for(int i=0; i<tpsim_h._sim_path_to_go.size(); ++i)
    {
        cout<<"========step "<<i<<endl;
        // get graph
        current_id_visited = tpsim_h.getSimVisitedGraph(visited_graph);
//        visited_graph.showGraphData();

        tpsim_h.moveNext();
    }
    // match to get location estimation
    stimer_h.tik();
    switch(graph_matching_loc){
    case 0:graphmatching_h.matchTwoTopoMetric(&visited_graph);break;
    case 1:graphmatching_h.matchTwoTopoMetricFirstOrder(&visited_graph);break;
    case 2:graphmatching_h.matchTwoTopoMetricSecondOrder(&visited_graph);break;
    default:break;
    }
    stimer_h.tok();
    vector<int> path_of_visited_graph = tpsim_h.getActualPathVisited();
    Eigen::VectorXi estpath_of_prior_graph = graphmatching_h.recoverPriorPath(path_of_visited_graph);
    tpsim_h.compareEstPathWithGT(estpath_of_prior_graph);
    mode = -1;
}

void Decision::actlocBaselineSim(){
    ROS_INFO("[AMPT]sim baseline active localization mode.");
    // start pos is set in paramfile
    int i = 0;
    actlocbaseline_h.init(&prior_graph, first_action);
    logfile.open(graph_path+"actloclog.txt");
    while(ros::ok()){
        cout<<"========step "<<i++<<endl;
        // get visited graph
        current_id_visited = tpsim_h.getSimVisitedGraph(visited_graph);
        visited_graph.showGraphData();
        // match to get location estimation
        graphmatching_h.matchTwoTopoMetric(&visited_graph);
        current_id_prior = graphmatching_h.getIDPriorFromIDVisited(current_id_visited);

        //save information
        visited_graph.saveGraph(graph_path+to_string(i));
        graphmatching_h.saveBestMatchRes(to_string(i));
        logBestMatchPairs();

        Eigen::VectorXi id_nodes_in_puzzle_zone_vec;
        int puzzle_zone_id = getPuzzleZone(id_nodes_in_puzzle_zone_vec);
        if(id_nodes_in_puzzle_zone_vec.size() == 0) break;
        // action
        Eigen::Vector3d action = tpsim_h.getCurrentPos();
        vector<Eigen::Vector3d> obs = tpsim_h.getObservation(-1);
        switch (baseline_actloc_ctrl) {
        case 0:action=actlocbaseline_h.randomAction(obs);break;
        case 1:action=actlocbaseline_h.furthestAction(action);break;
        default:action=actlocbaseline_h.randomAction(obs);break;
        }
        tpsim_h.moveToRela(action);
        logfile<<endl;
    }
    vector<int> actualpath = tpsim_h.getActualPath();
    logPath(actualpath, &prior_graph);
    logfile.close();
    mode = -1;
}

void Decision::actlocSim(){
    ROS_INFO("[AMPT]sim active localization mode.");
    // start pos is set in paramfile
    int i = 0;
    logfile.open(graph_path+"actloclog.txt");
    double totaltime = 0;
    while(ros::ok()){
        cout<<"========step "<<i++<<endl;
        // get visited graph
        current_id_visited = tpsim_h.getSimVisitedGraph(visited_graph);
        visited_graph.showGraphData();
        // match to get location estimation
        graphmatching_h.matchTwoTopoMetric(&visited_graph);
        current_id_prior = graphmatching_h.getIDPriorFromIDVisited(current_id_visited);

        //save information
        visited_graph.saveGraph(graph_path+to_string(i));
        graphmatching_h.saveBestMatchRes(to_string(i));
        logBestMatchPairs();

        // do active localization decision
        stimer_h.tik();
        Eigen::VectorXi id_nodes_in_puzzle_zone_vec;
        int puzzle_zone_id = getPuzzleZone(id_nodes_in_puzzle_zone_vec);
        if(id_nodes_in_puzzle_zone_vec.norm() == 0) break;
        searchPathOutPuzzleZone(id_nodes_in_puzzle_zone_vec);
        totaltime += stimer_h.tok();
        // move to target
        tpsim_h.moveTo(xyz_tar_visited);
        logfile<<endl;
    }
    cout<<"average decision time is: "<<totaltime/i<<"seconds."<<endl;
    vector<int> actualpath = tpsim_h.getActualPath();
    logPath(actualpath, &prior_graph);
    logfile.close();
    mode = -1;
}

void Decision::actlocOverlaySim(){
    ROS_INFO("[AMPT]sim overlay active localization mode.");
    // start pos is set in paramfile
    int i = 0;
    double totaltime = 0;
    actlocoverlay_h.setPriorGraph(prior_graph);
    logfile.open(graph_path+"actloclog.txt");
    while(ros::ok()){
        cout<<"========step "<<i++<<endl;
        // get visited graph
        current_id_visited = tpsim_h.getSimVisitedGraph(visited_graph);
        visited_graph.showGraphData();
        // match to get location estimation
        graphmatching_h.matchTwoTopoMetric(&visited_graph);
        current_id_prior = graphmatching_h.getIDPriorFromIDVisited(current_id_visited);
        //save information
        visited_graph.saveGraph(graph_path+to_string(i));
        graphmatching_h.saveBestMatchRes(to_string(i));
        logBestMatchPairs();

        // get top several estimations
        vector<Eigen::Vector3d> _t_vec;
        vector<int> _node_id_vec;
        vector<Eigen::Quaterniond> _q_vec;
        graphmatching_h.getTopSimilarEstimation(current_id_visited, _node_id_vec, _q_vec, _t_vec);

        if(_node_id_vec.size() == 1) break;

        // overlay map and get best action
        stimer_h.tik();
        actlocoverlay_h.overlayMap(_node_id_vec, _q_vec, _t_vec);
        Eigen::Vector3d action = actlocoverlay_h.action();
        totaltime += stimer_h.tok();
        // move to target
        tpsim_h.moveTo(action);
        if(i >= 10) break;
        logfile<<endl;
    }
    cout<<"average decision time is: "<<totaltime/i<<"seconds."<<endl;
    vector<int> actualpath = tpsim_h.getActualPath();
    logPath(actualpath, &prior_graph);
    logfile.close();
    mode = -1;
}

void Decision::logPath(vector<int> pathids, TopoMetric_c *fromgraph)
{
    double pathlength = 0.0;
    for(int i=0; i<pathids.size(); ++i)
    {
        logfile << pathids[i] <<" ";
        if(i > 0){
            pathlength += fromgraph->getDistance(pathids[i], pathids[i-1]);
        }
    }
    logfile <<endl<<pathlength<<endl;
}

void Decision::logBestMatchPairs()
{
    Eigen::VectorXi retmatch = graphmatching_h.getBestMatchRes();
    string s1="", s2="";
    for(int tmpi=0; tmpi<prior_graph.size(); ++tmpi){
        if(retmatch(tmpi) < 0) continue;
        s1 += to_string(tmpi); // prior
        s1 += " ";
        s2 += to_string(retmatch(tmpi)); // visited
        s2 += " ";
    }
    logfile << s1 <<endl<< s2 <<endl
            << current_id_visited <<endl;
}

void Decision::matchself()
{
    ROS_INFO("[AMPT]match self.");
    stimer_h.tik();
    graphmatching_h.matchSelf();
    stimer_h.tok();

    mode = -1;
}

void Decision::test()
{
    ROS_INFO("[AMPT]test mode.");

    cout<<"[Decision]test match"<<endl;
    stimer_h.tik();

    stimer_h.tok();
    mode = -1;
}

void Decision::RCPPtest()
{
    vector<int> path_node_vec;
    prior_graph.showGraphData();
    rcpp_h.calRCPPPath(&prior_graph, tpsim_h._start_node_id, -1, path_node_vec);
    mode = -1;
}

void Decision::makeOneNodeGraphWithCurSZ(TopoMetric_c& graph)
{
    graph.clear();
    Node_c node(0,0,0,0);
    graph.nodes_vec.push_back(node);
    graph.id_near_zero_node = 0;
    graph.size_confirm_nodes = 1;
    double current_yaw = atan2(2 * (q_slam.w() * q_slam.z() + q_slam.x() * q_slam.y()) , 1 - 2 * (q_slam.y() * q_slam.y() + q_slam.z() * q_slam.z()));
    for(int i=1; i<cur_safe_dir_self_vec.size(); ++i)
    {
        double safeangle_slam = cur_safe_dir_self_vec[i] + current_yaw;
        double x = 3*cos(safeangle_slam), y = 3*sin(safeangle_slam);
        node.setxyzid(x,y,0,i);
        graph.nodes_vec.push_back(node);
        graph.nodes_vec[0].addLink(i,3);
        graph.nodes_vec[i].addLink(0,3);
    }
}

void Decision::initFirstGraph()
{
    ros::Rate r(20);
    ROS_INFO("[AMPT]init first graph(1 node)...");
    xyz_tar_visited = Eigen::Vector3d(0,0,10);
    visited_graph.initZeroNode(0,0,0,0);
    to_crossing = true;
    current_id_visited = 0;
    while (ros::ok()){
        if(!safedir_record_ok){
            // rotate robot until all safe direction recorded
            p3atctrl_h.move(0, 0.3);
        }
        else{
            // make init graph, only 0,0,0 is at visited graph at the beginning
            p3atctrl_h.move(0, 0);
            // int id_init_prior = prior_graph.searchNearestNode(Eigen::Vector3d(t_prior_visited));
            double dist = 3;
            Node_c node(0,0,0,0);
            visited_graph.nodes_vec.resize(safe_dir_vec.size()+1);
            visited_graph.nodes_vec[0].setxyzid(0,0,0,0);
            Eigen::Vector3d tmpxyz;
            tmpxyz << 0,0,0;
            vector<Eigen::Vector3d> bcneibor;
            Eigen::Vector3d neiborxyz;
            for(int i=0; i<safe_dir_vec.size(); ++i)
            {
                neiborxyz << dist*cos(safe_dir_vec[i]), dist*sin(safe_dir_vec[i]), 0;
                Eigen::Vector3d tmpv3d(dist*cos(safe_dir_vec[i]), dist*sin(safe_dir_vec[i]), 0);
                visited_graph.nodes_vec[i+1].setxyzid(tmpv3d(0), tmpv3d(1), tmpv3d(2), i+1);
                visited_graph.nodes_vec[0].addLink(i+1, dist);
                visited_graph.nodes_vec[i+1].addLink(0, dist);
                bcneibor.push_back(neiborxyz);
                cout<<"add node "<<(i+1)<<" ("<<visited_graph.nodes_vec[i+1].xyz(0)<<", "<<visited_graph.nodes_vec[i+1].xyz(1)<<", "<<visited_graph.nodes_vec[i+1].xyz(2)<<endl;
            }
            visited_graph.backUpNeibor(tmpxyz, bcneibor);
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("[AMPT]init first graph done.");
    visited_graph.showGraphData();
    visited_graph.saveGraph(graph_path+"show/1");
    std_msgs::String msg;
    msg.data = graph_path+"show/1_";
    visitgraph_savedone_pub.publish(msg);
}

int Decision::getPuzzleZone(Eigen::VectorXi& id_nodes_in_puzzle_zone_vec)
{
    ROS_INFO("[AMPT]start active loc judging...");
    // find out whether all nodes in visited_graph is in the same puzzle zone.
    // if there is no puzzle zone, exit active localization, save t.
    id_nodes_in_puzzle_zone_vec.resize(prior_graph.size_confirm_nodes);
    id_nodes_in_puzzle_zone_vec.setConstant(0);
    vector<int> nodes_id_visited;
    int id_puzzle_zone;
    vector<int> id_puzzle_zone_vec;
    for(int i=0; i<visited_graph.size_confirm_nodes; ++i)
    {
        nodes_id_visited.push_back(i);
    }
    // judge whether all nodes of visited graph are in one indistinguishable subgraph(puzzle zone)
    if(!graphmatching_h.inTheSamePuzzleZone(nodes_id_visited, 1, id_puzzle_zone_vec)){
        ROS_INFO("[AMPT]active localization done!");
        cout<<"[AMPT]fix t_prior_visited = ("<<t_prior_visited(0)<<", "<<t_prior_visited(1)<<", "<<t_prior_visited(2)<<")"<<endl;
        return -1;
    }
    switch (puzzle_type) {
    case 0:id_puzzle_zone = id_puzzle_zone_vec[id_puzzle_zone_vec.size()-1];break;
    case 1:id_puzzle_zone = id_puzzle_zone_vec[0];break;
    case 2:id_puzzle_zone = -1;break;
    default:id_puzzle_zone = id_puzzle_zone_vec[id_puzzle_zone_vec.size()-1];break;
    }
    cout<<"[AMPT]all nodes are in puzzle zone ";
    if(id_puzzle_zone == -1){
        for(int j=0; j<id_puzzle_zone_vec.size(); ++j)
        {
            vector<int> nodespz = graphmatching_h.getPuzzleZoneNodes(id_puzzle_zone_vec[j]);
            for(int i=0; i<nodespz.size(); ++i)
            {
                id_nodes_in_puzzle_zone_vec(nodespz[i]) = 1;
            }
        }
    }
    else{
        vector<int> nodespz = graphmatching_h.getPuzzleZoneNodes(id_puzzle_zone);
        for(int i=0; i<nodespz.size(); ++i)
        {
            id_nodes_in_puzzle_zone_vec(nodespz[i]) = 1;
        }
    }
    cout<<endl;
    // label visited paths in prior graph
    labelVisitedEdges();
    return id_puzzle_zone;
}

void Decision::labelVisitedEdges()
{
    cout<<"[AMPT]visited edge in prior: ";
    for(int i=0; i<visited_graph.size_confirm_nodes; ++i)
    {
        int id_1_prior = graphmatching_h.getIDPriorFromIDVisited(i);
        for(int j=0; j<visited_graph.nodes_vec[i].id_neibor.size(); ++j)
        {
            int id_neibor = visited_graph.nodes_vec[i].id_neibor[j];
            if(id_neibor < visited_graph.size_confirm_nodes && i < id_neibor){
                int id_2_prior = graphmatching_h.getIDPriorFromIDVisited(id_neibor);
                prior_graph.setVisitedEdge(id_1_prior, id_2_prior);
                cout<<"["<<id_1_prior<<", "<<id_2_prior<<"], ";
            }
        }
    }
    cout<<endl;
}

void Decision::searchPathOutPuzzleZone(Eigen::VectorXi id_nodes_in_puzzle_zone_vec)
{
    ROS_INFO("[AMPT]search path out puzzle zone.");
    vector<int> nodes_id_visited;
    //nodes_id_visited.clear(); // used to store path
    prior_graph.Dijkstra(current_id_prior, nodes_id_visited);
    int id_end_prior=-1;
    for(int i=0; i<nodes_id_visited.size(); ++i)
    {
//        if(!graphmatching_h.inThisPuzzleZone(nodes_id_visited[i], 0, id_puzzle_zone)){
        if(id_nodes_in_puzzle_zone_vec(nodes_id_visited[i]) < 1){
            id_end_prior = nodes_id_visited[i];
            cout<<"[AMPT]nearest node out of puzzle is "<<id_end_prior<<endl;
            break;
        }
    }
    vector<int> path_out_puzzle_zone;
    prior_graph.getPathFromDijkstra(id_end_prior, path_out_puzzle_zone);
    // publish target point
    int id_tar_prior = path_out_puzzle_zone[path_out_puzzle_zone.size()-2];
    /// should modify
    xyz_tar_visited = graphmatching_h.getCoordInVisitedFromIDPrior(id_tar_prior);
    /// should modify^
    cout << "[AMPT]going to (visited): "<< xyz_tar_visited(0)<<","<< xyz_tar_visited(1)<<","<< xyz_tar_visited(2)<< endl;
    goToLocalTarget(xyz_tar_visited(0), xyz_tar_visited(1), xyz_tar_visited(2));

    // log puzzle zone id end node of path out puzzle zone and target in visited frame
    for(int i=0; i<prior_graph.size_confirm_nodes; ++i)
    {
        if(id_nodes_in_puzzle_zone_vec(i)>0){
            logfile << i<<" ";
        }
    }
    logfile << endl << id_end_prior <<endl
            << xyz_tar_visited(0) <<" "<< xyz_tar_visited(1)
            <<" "<< xyz_tar_visited(2) <<endl;
}

void Decision::updateCurrentID()
{
    cout << "[AMPT]at ("<<pose_slam(0)<<","<<pose_slam(1)<<","<<pose_slam(2)<<")."<<endl;
    current_id_visited = visited_graph.searchNearestNode(pose_slam);
    current_id_prior = graphmatching_h.getIDPriorFromIDVisited(current_id_visited);
    cout << "[AMPT]nearest node in visited graph is: "<< current_id_visited <<endl;
    cout << "[AMPT]nearest node in prior graph is: "<< current_id_prior <<endl;
}

void Decision::backUpNeibor(Eigen::Vector3d _pose_slam, Eigen::Quaterniond _q_slam, vector<float> _cur_safe_dir_self_vec)
{
    vector<Eigen::Vector3d> bcneibor;
    Eigen::Vector3d neiborxyz;
    double dist = 3.0;
    for(int i=1; i<_cur_safe_dir_self_vec.size(); ++i)
    {
        // current frame
        neiborxyz << dist*cos(_cur_safe_dir_self_vec[i]), dist*sin(_cur_safe_dir_self_vec[i]), 0;
        // turn to slam frame
        neiborxyz = _q_slam * neiborxyz + _pose_slam;
        bcneibor.push_back(neiborxyz - _pose_slam);
    }
    visited_graph.backUpNeibor(_pose_slam, bcneibor);
}

void Decision::poseSlamCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    pose_slam = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_slam = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    int last_id_visited = current_id_visited;
    current_id_visited = visited_graph.searchNearbyNode(pose_slam, current_id_visited);

    slam_pose_ok = true;
    if(!safedir_record_ok){
        return;
    } 
    
    double distance_to_target = (pose_slam - xyz_tar_visited).norm();
    if(distance_to_target < 0.5){
        // back up nodes here
        backUpNeibor(pose_slam, q_slam, cur_safe_dir_self_vec);

        saveTopoMap(graph_path);
        to_target = true;
        // to save map only one time, set the target point to some other place
        xyz_tar_visited(2) += 10;
    }
}

void Decision::topomapSavedoneCB(const std_msgs::String::ConstPtr msg)
{
    visited_graph.readAndContructTopoMetric(msg->data);
    cout<<"[AMPT]load new graph from "<<msg->data<<endl;
    visited_graph.showGraphData();
    visited_graph.saveGraph(graph_path+"show/1");
    std_msgs::String msg1;
    msg1.data = graph_path+"show/1_";
    visitgraph_savedone_pub.publish(msg1);
    // match graph
    if(mode == 1 || mode == 8){
        // known start
        graphmatching_h.matchTwoTopoMetricWithFixTrans(&visited_graph, t_prior_visited);
    }
    else if(mode == 2){
        if(doing_actloc){
            // actloc
            graphmatching_h.matchTwoTopoMetric(&visited_graph);
        }
        else{
            // actloc succeed, lock t
            graphmatching_h.matchTwoTopoMetricWithFixTrans(&visited_graph, t_prior_visited);
        }
    }
    // update current id at once
    updateCurrentID();
    just_load_graph = true;
    new_graph_loaded = true;
    topomaploop_working = false;
}

bool Decision::judgeToCrossing()
{
    // judge to a crossing
    bool ret = false;
    if(cur_safe_dir_self_vec.size() == 2){
        // ignore the first safe dir
        if(fabs(cur_safe_dir_self_vec[1]) > 1.05){
            // case 2: the only one safe direction is not in front
            ret = true;
        }
    }
    else if(cur_safe_dir_self_vec.size() > 2){
        // case 1: number of safe direction more than 1
        ret = true;
    }
    return ret;
}

void Decision::safezoneCB(const std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(!slam_pose_ok) return;
    // record current safe direction
    cur_safe_dir_self_vec.clear();
    cur_safe_dist_self_vec.clear();
    int num_safezone = int(msg->data.size()/4);
    cur_safe_dir_self_vec.resize(num_safezone);
    cur_safe_dist_self_vec.resize(num_safezone);
    min_forward_dist = 2;
    max_forward_dist = 0;
    for(int i=0; i<num_safezone; ++i)
    {
        // the first one is for target judging, sometimes it should be ignored
        // direction
        float safeangle_self = (msg->data[4*i+1] + msg->data[4*i+3]) / 2;
        cur_safe_dir_self_vec[i] = safeangle_self;
        // distance
        float safedist_self = (msg->data[4*i+0] + msg->data[4*i+2]) / 2;
        cur_safe_dist_self_vec[i] = safedist_self;
        float fdist = safedist_self * cos(safeangle_self);
        if(i > 0){
            if(fdist < min_forward_dist){
                if(fdist > 0){
                    min_forward_dist = fdist;
                }
                else{
                    min_forward_dist = 0;
                }
            }
            if(fdist > max_forward_dist){
                max_forward_dist = fdist;
            }
        }
    }
    min_forward_dist = (min_forward_dist + max_forward_dist)/2;
    // not publish first target and pose_slam is ok
    if(to_crossing && !safedir_record_ok && current_id_visited >= 0){
        // Eigen::Vector3d euler_angle = q_slam.matrix().eulerAngles(2,1,0); //yaw,pitch,roll
        float current_yaw = atan2(2 * (q_slam.w() * q_slam.z() + q_slam.x() * q_slam.y()) , 1 - 2 * (q_slam.y() * q_slam.y() + q_slam.z() * q_slam.z()));
        int i=1;
        float safeangle_slam;
        // cout << "number of safezone: "<< num_safezone<< ". current yaw: "<< current_yaw<< endl;
        float pi_m_2 = M_PI * 2;
        bool first_matched = false;
        // find safe directions that has been added
        if(safe_dir_vec.size() > 0){
            // find matched safezone
            float last_record_safe_angle_slam = safe_dir_vec[safe_dir_vec.size()-1],
                first_record_safe_angle_slam = safe_dir_vec[0];
            int first_not_recorded_safedir = 1; // ignore first safe dir
            for(i=1; i<num_safezone; ++i)
            {
                safeangle_slam = cur_safe_dir_self_vec[i] + current_yaw;
                if(safeangle_slam < 0){
                    safeangle_slam += pi_m_2;
                }
                else if(safeangle_slam > pi_m_2){
                    safeangle_slam -= pi_m_2;
                }
                float err_angle = fabs(safeangle_slam - first_record_safe_angle_slam);
                if(err_angle < 0.25){
                    // matched first recorded safe direction
                    first_matched = true;
                }
                err_angle = fabs(safeangle_slam - last_record_safe_angle_slam);
                if(err_angle < 0.25){
                    first_not_recorded_safedir = i + 1;
                    // matched last recorded safe direction
                    // cout<<"current safezone "<<i<<"("<<safeangle_slam<<") matches the last recorded safezone("<<last_record_safe_angle_slam<<")"<<endl;
                    break;
                }
            }
            float err_yaw = first_record_yaw - current_yaw;
            if(err_yaw < -M_PI){
                err_yaw += pi_m_2;
            }
            else if(err_yaw > M_PI){
                err_yaw -= pi_m_2;
            }
            if(first_matched && err_yaw > 0.1){
                // match first recorded safe direction and turn nearly 360 degree
                safedir_record_ok = true;
                p3atctrl_h.move(0,0);
                ROS_INFO("[AMPT]record init safe zone done.");
                return;
            }
        
            // add new safe directions
            // cout<< "add new safe directions"<<endl;
            for(int j=first_not_recorded_safedir; j<num_safezone; ++j)
            {
                // ignore safe direction near +-90 degree (>80 degree)
                if(msg->data[4*j+1] > 1.45 || msg->data[4*j+3] < -1.45) continue;
                safeangle_slam = cur_safe_dir_self_vec[j] + current_yaw;
                if(safeangle_slam < 0){
                    safeangle_slam += pi_m_2;
                }
                else if(safeangle_slam > pi_m_2){
                    safeangle_slam -= pi_m_2;
                }
                // go to the first recorded safe direction
                if(fabs(safeangle_slam - safe_dir_vec[0]) < 0.25){
                    safedir_record_ok = true;
                    p3atctrl_h.move(0,0);
                    ROS_INFO("[AMPT]record init safe zone done.");
                    break;
                }
                safe_dir_vec.push_back(safeangle_slam);
                cout<< "add direction: "<< safeangle_slam<<"("<<msg->data[4*j+1]<<","<<msg->data[4*j+3]<<")"<<" , at yaw: "<<current_yaw<<endl;
            }
        }
        else{
            first_record_yaw = current_yaw;
            for(int j=1; j<num_safezone; ++j)
            {
                float safeangle_self = (msg->data[4*j+1] + msg->data[4*j+3]) / 2;
                // ignore safe direction near +-90 degree (>80 degree)
                if(msg->data[4*j+1] > 1.45 || msg->data[4*j+3] < -1.45) continue;
                safeangle_slam = safeangle_self + current_yaw;
                if(safeangle_slam < 0){
                    safeangle_slam += pi_m_2;
                }
                else if(safeangle_slam > pi_m_2){
                    safeangle_slam -= pi_m_2;
                }
                safe_dir_vec.push_back(safeangle_slam);
                cout<< "add direction: "<< safeangle_slam<<"("<<msg->data[4*j+1]<<","<<msg->data[4*j+3]<<")"<<" , at yaw: "<<current_yaw<<endl;
            }
        }
    }
}

void Decision::goToGlobalTarget(double _x, double _y, double _z)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "slam";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _x;
    pose.pose.position.y = _y;
    pose.pose.position.z = _z;
    msg.poses.push_back(pose);
    global_tar_pub.publish(msg);
}

void Decision::goToLocalTarget(double _x, double _y, double _z)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "slam";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _x;
    pose.pose.position.y = _y;
    pose.pose.position.z = _z;
    msg.poses.push_back(pose);
    local_tar_pub.publish(msg);
}

void Decision::saveTopoMap(string filepath)
{
    topomaploop_working = true;
    std_msgs::String msg;
    msg.data = filepath;
    save_topomap_pub.publish(msg);
    ROS_INFO("[AMPT]ask to save topomap.");
}
