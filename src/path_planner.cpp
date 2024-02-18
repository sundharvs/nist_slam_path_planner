#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

#include "conversions.h"

#include <eigen3/Eigen/Dense>
#include "octomap/octomap.h"

// #include "fcl/config.h"
// #include "fcl/geometry/octree/octree.h"
// #include "fcl/narrowphase/collision.h"
// #include "fcl/broadphase/broadphase_bruteforce.h"
// #include "fcl/broadphase/broadphase_spatialhash.h"
// #include "fcl/broadphase/broadphase_SaP.h"
// #include "fcl/broadphase/broadphase_SSaP.h"
// #include "fcl/broadphase/broadphase_interval_tree.h"
// #include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
// #include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
// #include "fcl/broadphase/default_broadphase_callbacks.h"
// #include "fcl/geometry/geometric_shape_to_BVH_model.h"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

using std::placeholders::_1;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner : public rclcpp::Node
{
public:
    Planner():Node("planner")
    {
        octomap_subscription = this->create_subscription<octomap_msgs::msg::Octomap>(
            "octomap_full",10,std::bind(&Planner::octomap_callback, this, _1)
        );
        goal_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal",1, std::bind(&Planner::goal_callback, this, _1)
        );
        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "dlio/odom_node/odom", 10, std::bind(&Planner::odom_callback, this, _1)
        );
        start_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&Planner::start_callback, this, _1)
        );
        vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        traj_pub = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("waypoints", 10);
        path_pub = this->create_publisher<nav_msgs::msg::Path>("plan_path", 10);

        Quadcopter = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.5, 0.5, 0.5));
        fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
        tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
        
        space = ob::StateSpacePtr(new ob::SE3StateSpace());

        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space);
    
        // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);

        // set the bounds for the R^3
        ob::RealVectorBounds bounds(3);

        bounds.setLow(0,-10);
        bounds.setHigh(0,10);
        bounds.setLow(1,-10);
        bounds.setHigh(1,10);
        bounds.setLow(2,-3);
        bounds.setHigh(2,3);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // construct an instance of space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        start->setXYZ(0,0,0); //3.2464 -7.2917 0.1610
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        // start->values[0] = 3.2464;
        // start->values[1] = -7.2917;
        // start->values[2] = 0.1610;
        // start.random();

        goal->setXYZ(0,0,0);
        prev_goal[0] = 0;
        prev_goal[1] = 0;
        prev_goal[2] = 0;

        // goal->values[0] = -6.0098;
        // goal->values[1] = -0.3118;
        // goal->values[2] = 0.1523;

        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // goal.random();
        
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // set Optimizattion objective
        pdef->setOptimizationObjective(getPathLengthObjWithCostToGo(si));

        RCLCPP_INFO(this->get_logger(), "Planner Initialized");
    }

    void init_start(void){
        if(!set_start){
            std::cout << "Initialized" << std::endl;
        set_start = true;
        }
    }

    void setStart(double x, double y, double z)
    {
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(x,y,z);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearStartStates();
        pdef->addStartState(start);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Start point set to: " << x << " " << y << " " << z);
    }

    void setGoal(double x, double y, double z)
    {
        if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z)
        {
            ob::ScopedState<ob::SE3StateSpace> goal(space);
            goal->setXYZ(x,y,z);
            prev_goal[0] = x;
            prev_goal[1] = y;
            prev_goal[2] = z;
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

            ob::State *state =  space->allocState();
	        state->as<ob::SE3StateSpace::StateType>()->setXYZ(goal->getX(), goal->getY(), goal->getZ());
            state->as<ob::SE3StateSpace::StateType>()->rotation().w = goal->rotation().w;
            state->as<ob::SE3StateSpace::StateType>()->rotation().x = goal->rotation().x;
            state->as<ob::SE3StateSpace::StateType>()->rotation().y = goal->rotation().y;
            state->as<ob::SE3StateSpace::StateType>()->rotation().z = goal->rotation().z;

            if(isStateValid(state)){    
                pdef->clearGoal();
                pdef->setGoalState(goal);
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal point set to: " << x << " " << y << " " << z);
                if(set_start){
                    plan();
                }
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal point invalid");
            }
        }
    }

    void updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map/*octomap::OcTree tree_oct*/)
    {
        tree_obj = map;
    }


    void replan(void)
    {
        if(path_smooth != NULL && set_start)
        {
            std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
            if(path_smooth->getStateCount () <= 2)
				plan();
            else
            {
                for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
				{
					if(!replan_flag)
						replan_flag = !isStateValid(path_smooth->getState(idx));
					else
						break;

				}
				if(replan_flag)
					plan();
				else
					std::cout << "Replanning not required" << std::endl;
			}
		}

    }

	void plan(void)
	{

	    // create a planner for the defined space
		ob::PlannerPtr plan(new og::InformedRRTstar(si));

	    // set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

	    // perform setup steps for the planner
		plan->setup();

        plan->printProperties(std::cout);
        plan->printSettings(std::cout);

	    // print the settings for this space
		// si->printSettings(std::cout);

	    // print the problem settings
		// pdef->print(std::cout);

	    // attempt to solve the problem within one second of planning time
		ob::PlannerStatus solved = plan->solve(2);

		if (solved)
		{
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);
	        // print the path to screen
	        // path->print(std::cout);
			trajectory_msgs::msg::MultiDOFJointTrajectory msg;
			trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point_msg;

			msg.header.stamp = this->now();
			msg.header.frame_id = "odom";
			msg.joint_names.clear();
			msg.points.clear();
			msg.joint_names.push_back("Quadcopter");
			
			for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
			{
				const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
				point_msg.time_from_start = rclcpp::Duration::from_seconds(this->now().seconds());
				point_msg.transforms.resize(1);

				point_msg.transforms[0].translation.x= pos->values[0];
				point_msg.transforms[0].translation.y = pos->values[1];
				point_msg.transforms[0].translation.z = pos->values[2];

				point_msg.transforms[0].rotation.x = rot->x;
				point_msg.transforms[0].rotation.y = rot->y;
				point_msg.transforms[0].rotation.z = rot->z;
				point_msg.transforms[0].rotation.w = rot->w;

				msg.points.push_back(point_msg);

			}
			traj_pub->publish(msg);

			
	        //Path smoothing using bspline

			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth,3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);

			
			//Publish path as markers

			visualization_msgs::msg::Marker marker;
			marker.action = visualization_msgs::msg::Marker::DELETEALL;
			vis_pub->publish(marker);

			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
	                // cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
				
				marker.header.frame_id = "odom";
				marker.header.stamp = this->now();
				marker.ns = "path";
				marker.id = idx;
				marker.type = visualization_msgs::msg::Marker::CUBE;
				marker.action = visualization_msgs::msg::Marker::ADD;
				marker.pose.position.x = pos->values[0];
				marker.pose.position.y = pos->values[1];
				marker.pose.position.z = pos->values[2];
				marker.pose.orientation.x = rot->x;
				marker.pose.orientation.y = rot->y;
				marker.pose.orientation.z = rot->z;
				marker.pose.orientation.w = rot->w;
				marker.scale.x = 0.15;
				marker.scale.y = 0.15;
				marker.scale.z = 0.15;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				vis_pub->publish(marker);
				// ros::Duration(0.1).sleep();
				std::cout << "Published marker: " << idx << std::endl;  
			}

            // Publish path as nav_msgs/Path

            nav_msgs::msg::Path plan_path;

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
			{
				const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

	            // extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	            // extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
                geometry_msgs::msg::PoseStamped path_pose;
    
                path_pose.header.frame_id = "odom";
                path_pose.header.stamp = this->now();
                path_pose.pose.position.x = pos->values[0];
				path_pose.pose.position.y = pos->values[1];
				path_pose.pose.position.z = pos->values[2];
				path_pose.pose.orientation.x = rot->x;
				path_pose.pose.orientation.y = rot->y;
				path_pose.pose.orientation.z = rot->z;
				path_pose.pose.orientation.w = rot->w;

                plan_path.poses.push_back(path_pose);
			}

            plan_path.header.frame_id = "odom";
            plan_path.header.stamp = this->now();
            path_pub->publish(plan_path);
			
			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
			std::cout << "No solution found" << std::endl;
	}

private:
	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

    double prev_goal[3];

	og::PathGeometric* path_smooth = NULL;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry<double>> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj;

    bool set_start = false;

    bool isStateValid(const ob::State *state)
	{
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObject<double> treeObj((tree_obj));
		fcl::CollisionObject<double> aircraftObject(Quadcopter);


        // check validity of state defined by pos
        fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
        aircraftObject.setTransform(rotation, translation);
        fcl::CollisionRequest<double> requestType(1,false,1,false);
        fcl::CollisionResult<double> collisionResult;
        fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
	}


    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        // obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }

    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg)
    {
        octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*octomap_msg));
        fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(tree_oct));

        this->updateMap(std::shared_ptr<fcl::CollisionGeometry<double>>(tree));
	    this->replan();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        this->setStart(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        this->init_start();
    }

    void start_callback(const geometry_msgs::msg::PointStamped::SharedPtr start_msg)
    {
        this->setStart(start_msg->point.x, start_msg->point.y, start_msg->point.z);
        this->init_start();
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        this->setGoal(goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z);
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_subscription;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}
