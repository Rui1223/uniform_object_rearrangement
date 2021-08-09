#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <uniform_object_rearrangement/AstarPathFinding.h>
#include <uniform_object_rearrangement/AstarPathFindingLabeled.h>
#include <uniform_object_rearrangement/ResetRoadmap.h>

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "Timer.hpp"

// struct AstarResult
// {
//     bool searchSuccess;
//     std::vector<int> path;
// };

class Planner_t {

public:

    Graph_t m_right_torso_g;
    AstarSolver_t m_astar_solver;

    // constructor
    Planner_t() {}
    Planner_t(std::string right_torso_samples_file, std::string right_torso_connections_file) 
    {
        m_right_torso_g.constructGraph(right_torso_samples_file, right_torso_connections_file);
    }

    bool astarSolverCallback(
        uniform_object_rearrangement::AstarPathFinding::Request &req,
        uniform_object_rearrangement::AstarPathFinding::Response &resp) 
    {
        if (req.armType == "Right_torso"){
            if (m_astar_solver.getQueryIdx() != req.query_idx) {
                // this is a new query, let's set the new query
                m_astar_solver.setPlanningQuery(m_right_torso_g, req.query_idx, 
                    req.start_idx, req.goal_idx, req.start_config, req.goal_config,
                    req.start_neighbors_idx, req.goal_neighbors_idx,
                    req.start_neighbors_cost, req.goal_neighbors_cost,
                    req.violated_edges);
            }
            m_right_torso_g.modifyEdge(req.violated_edges, req.query_idx);
            m_astar_solver.prepareToSearch(m_right_torso_g);
            m_astar_solver.Astar_search(m_right_torso_g);
        }
        // let's return the response after a search
        resp.searchSuccess = m_astar_solver.getSearchSuccessInfo();
        resp.path = m_astar_solver.getPath();
        return true;
    }

    bool astarSolverLabeledCallback(
        uniform_object_rearrangement::AstarPathFindingLabeled::Request &req,
        uniform_object_rearrangement::AstarPathFindingLabeled::Response &resp) 
    {
        if (req.armType == "Right_torso"){
            if (m_astar_solver.getQueryIdx() != req.query_idx) {
                // this is a new query, let's set the new query
                m_astar_solver.setPlanningQuery_labeled(m_right_torso_g, req.query_idx, 
                    req.start_idx, req.goal_idx, req.start_config, req.goal_config,
                    req.start_neighbors_idx, req.goal_neighbors_idx,
                    req.start_neighbors_cost, req.goal_neighbors_cost,
                    req.occupied_labels, req.isInHandManipulation, 
                    req.violated_edges);
            }
            m_right_torso_g.modifyEdge(req.violated_edges, req.query_idx);
            m_astar_solver.prepareToSearch(m_right_torso_g);
            m_astar_solver.Astar_search_labeled(m_right_torso_g);
        }
        // let's return the response after a search
        resp.searchSuccess = m_astar_solver.getSearchSuccessInfo();
        resp.path = m_astar_solver.getPath();
        return true;
    }

    bool resetRoadmapCallback(
        uniform_object_rearrangement::ResetRoadmap::Request &req,
        uniform_object_rearrangement::ResetRoadmap::Response &resp)
    {
        if (req.armType == "Right_torso") {
            m_right_torso_g.resetEdgeStatus();
        }
        resp.success = true;
        return true;
    }

};

int main(int argc, char** argv)
{
    // initialize the ros node
    ros::init(argc, argv, "main_planner");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("initialize main_plannner_node");

    std::string package_path = ros::package::getPath("uniform_object_rearrangement");
    Timer t;

    // load the roadmap for left and right arm
    std::string right_torso_samples_file = package_path + "/roadmaps/samples_Right_torso.txt";
    std::string right_torso_connections_file = package_path + "/roadmaps/connections_Right_torso.txt";
    Planner_t planner(right_torso_samples_file, right_torso_connections_file);

    // planner.printWrapper();
    std::cout << "time to load graph with " << planner.m_right_torso_g.getnNodes() << " nodes is " << t.elapsed() << "\n";

    // claim service the node provide (server)
    ros::ServiceServer astar_server = nh.advertiseService("astar_path_finding", &Planner_t::astarSolverCallback, &planner);
    ros::ServiceServer astar_labeled_server = nh.advertiseService("astar_path_finding_labeled", &Planner_t::astarSolverLabeledCallback, &planner);
    ros::ServiceServer reset_roadmap_server = nh.advertiseService("reset_roadmap", &Planner_t::resetRoadmapCallback, &planner);


    // Loop at 2Hz until the node is shut down
    // ros::Rate rate(2);
    while (ros::ok()) {
        ros::spin();
        // rate.sleep();
    }

    return 0;

}