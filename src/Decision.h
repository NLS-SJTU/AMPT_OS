#ifndef DECISION_H
#define DECISION_H

#include <sys/time.h>

#include "TopoMetric.h"
#include "GraphMatching.h"
#include "RCPP.h"
#include "utils/Pioneer3AT.h"
#include "TopoSim.h"
#include "cmpalg/ViterbiLoc.h"
#include "cmpalg/ActLocBaseline.h"
#include "cmpalg/ActLocOverlay.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

class SimpleTimer
{
public:
    SimpleTimer(){}
    ~SimpleTimer(){}

    // start timer
    void tik(){
        gettimeofday(&tv_s, NULL);
    }
    // end timer and return time(ms) from tik()
    double tok(){
        gettimeofday(&tv_e, NULL);
        double durationtime = 1000.0*(tv_e.tv_sec - tv_s.tv_sec) + 1.0*(tv_e.tv_usec - tv_s.tv_usec)/1000.0;
        cout<<"[SimpleTimer]"<<durationtime<<" ms."<<endl;
        return durationtime;
    }
private:
    struct timeval tv_s, tv_e;
};

class Decision
{
public:
    Decision(ros::NodeHandle& _nh, string paramfile);
    ~Decision(){}

    // run methods
    void run();
private:
    // read parameter
    void readParam(string paramfile);

    // init ros related stuff
    void initROS();

    // run original method with known start node
    void runOriginal();

    // run active localization method
    void runActSLAM();

    // test any method
    void test();

    // pure loc simulation on graph
    void locSim();

    // pure loc simulation with viterbi loc
    void viterbiLocSim();

    // pure actloc simulation with only graph
    void actlocSim();

    void actlocBaselineSim();

    void actlocOverlaySim();

    void matchself();

    // active localization in actloc mode
    void activeLocalizationGazebo();

    // active mapping
    void activeMappingGazebo();

    //
    void RCPPtest();

    //
    void logBestMatchPairs();

    //
    void logPath(vector<int> pathids, TopoMetric_c *fromgraph);

    // parameters
    int mode;
    string graph_path, param_path;
    int baseline_actloc_ctrl;
    int graph_matching_loc;
    double first_action;
    int puzzle_type;

    // useful class
    RCPP rcpp_h;
    // RCPP<TopoMetric_c, Node_c> rcpp_h;
    GraphMatching<TopoMetric_c, Node_c> graphmatching_h;
    TopoMetric_c prior_graph, visited_graph;
    SimpleTimer stimer_h;
    Pioneer3AT p3atctrl_h;
    TopoSim tpsim_h;
    ViterbiLoc viterbiloc_h;
    ActLocBaseline actlocbaseline_h;
    ActLocOverlay actlocoverlay_h;

    //ros
    ros::NodeHandle nh;
    ros::Subscriber pose_slam_sub;
    // subscribe whether topomap save done
    ros::Subscriber topomap_savedone_sub;
    // publish when visit graph is saved
    ros::Publisher visitgraph_savedone_pub;
    // ask to save topomap
    ros::Publisher save_topomap_pub;
    // publish only one target point which you want to get to through a path in the map
    ros::Publisher global_tar_pub;
    // publish only one target point which is not in the map
    ros::Publisher local_tar_pub;
    // subscribe safe zone
    ros::Subscriber safezone_sub;

    // whether a new node appears
    bool to_next_node;
    // whether a new graph is loaded
    bool new_graph_loaded;
    // whether the first target is published
    bool first_target_ok;
    // whether in the process of active localization
    bool doing_actloc;

    // path of RCPP
    vector<int> path_vec;

    //
    bool topomaploop_working;

    // id of current node
    int current_id_prior, current_id_visited;
    // at the ith node in the path_vec
    int ith_node_in_path;

    // pose in slam frame
    Eigen::Vector3d pose_slam;
    Eigen::Quaterniond q_slam;
    // target point in slam frame
    Eigen::Vector3d xyz_tar_visited;
    // t_prior_visited if mode 1
    Eigen::Vector3d t_prior_visited;
    Eigen::Quaterniond q_prior_visited;

    // just rotate to init first graph for original mode
    void initFirstGraph();
    // recorded save zone when rorating
    vector<float> safe_dir_vec;
    // whether all safe direction is recorded
    bool safedir_record_ok;
    // yaw at start pose, used to judge whether it rotates enough
    float first_record_yaw;
    // minimum distance to go for forward
    float min_forward_dist, max_forward_dist;
    // whether already got pose from slam
    bool slam_pose_ok;
    // make a graph with current safezone direction
    void makeOneNodeGraphWithCurSZ(TopoMetric_c& graph);

    //
    ofstream logfile;

    // go to three nodes to init graph for actloc mode
    void initThreeNodesGraph();
    // go to one crossing
    bool to_crossing;
    // reach target point
    bool to_target;
    // current safe directions
    vector<float> cur_safe_dir_self_vec;
    // current safe distance coresponding to directions
    vector<float> cur_safe_dist_self_vec;
    // judge whether getting to a crossing
    bool judgeToCrossing();
    // search path to get out of puzzle zone
    void searchPathOutPuzzleZone(VectorXi id_nodes_in_puzzle_zone_vec);
    // back up neibor nodes of current position to visited_graph using current pose and safe dirs
    void backUpNeibor(Eigen::Vector3d _pose_slam, Eigen::Quaterniond _q_slam, vector<float> _cur_safe_dir_self_vec);
    // find out which puzzle zone is in
    int getPuzzleZone(VectorXi &id_nodes_in_puzzle_zone_vec);

    // callbacks
    void poseSlamCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void topomapSavedoneCB(const std_msgs::String::ConstPtr msg);
    void safezoneCB(const std_msgs::Float32MultiArray::ConstPtr msg);

    // publish target point in slam frame
    void goToGlobalTarget(double _x, double _y, double _z);
    void goToLocalTarget(double _x, double _y, double _z);

    // ask to save topomap
    void saveTopoMap(string filepath);
    bool just_load_graph;

    // update current id using current pose(should be used after graph matching)
    void updateCurrentID();
    //
    void labelVisitedEdges();
};

#endif
