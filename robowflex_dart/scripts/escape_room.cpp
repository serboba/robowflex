//
// Created by serboba on 20.01.22.
//

//
// Created by serboba on 03.12.21.
//

//
// Created by serboba on 13.10.21.
//

//
// Created by serboba on 05.09.21.
//

//
// Created by serboba 19.11.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <robowflex_dart/ActionRobot.h>
#include <robowflex_dart/urdf_read.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <robowflex_library/class_forward.h>
#include <robowflex_dart/gui.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/conversion_functions.h>
#include <robowflex_dart/quaternion_factory.h>
#include <robowflex_dart/Object.h>
#include <robowflex_dart/planningFunctions.h>

using namespace robowflex;

boost::filesystem::path p(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = p.string() + "/src/robowflex/robowflex_dart/include/io/";


static const std::string GROUP_X = "arm_with_x_move";



int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    std::string env_name;
    if(argc > 1 )
        env_name = std::string(argv[1]);
    else
        env_name = "room1"; // test in cpp


    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE */
    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             abs_path +"envs/fetch/urdf/fetch4.urdf",  //
                                             abs_path +"envs/fetch/srdf/fetch4.srdf");


    auto maze_dart = darts::loadMoveItRobot(env_name,
                                            abs_path +"envs/" + env_name+ "/" + "urdf/" + env_name + ".urdf",
                                            abs_path +"envs/" + env_name+ "/" + "srdf/" + env_name + ".srdf");

    auto fetch_name = fetch_dart->getName();
    auto door_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(maze_dart);



//    create_txt_from_urdf(env_name);
    std::vector<Object> obj_;
    read_obj_txt_file(env_name,obj_);

    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE UNTIL HERE !!!! */
    darts::Window window(world);

    // fetch_dart->setJoint("torso_lift_joint",0.25); // maze2 to avoid start collision
    // fetch_dart->setJoint("move_x_axis_joint",-0.10);
    Eigen::VectorXd start(11);
    start << 0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0, 0.0,0.0,0.0;
    fetch_dart->setGroupState("arm_with_x_move",start);

    window.run([&] {

        URDF_IO input_(env_name);
        int lastIndex = input_.group_indices.back().back();
        std::vector<int> robot_gr = {lastIndex+1,lastIndex+2};
        input_.group_indices.push_back(robot_gr);

        std::vector<ActionP> actions_path;
        std::vector<ActionR> actions_robot;
        getActionsFromPath(env_name, input_.group_indices, actions_path);
        translateRoomActions(actions_path, obj_, actions_robot);

        darts::PlanBuilder builder(world);
        builder.addGroup("fetch", GROUP_X);
        if(env_name=="room1")
        {
            world->getRobot("fetch")->setJoint("move_y_axis_joint",0.7);
            world->getRobot("fetch")->setJoint("move_x_axis_joint",-0.6);
        }
        //builder.setStartConfiguration({0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0, 0.0,0.0,0.0}); // folded arm maybe necessary
        builder.setStartConfigurationFromWorld();
        Eigen::VectorXd start_config = builder.getStartConfiguration();

//        world->getRobot(env_name)->setJoint("link_0_joint_0",0.89);
//        world->getRobot(env_name)->setJoint("link_0_joint_1",-0.84);
//        world->getRobot(env_name)->setJoint("link_1_joint_0",1.90);
//        world->getRobot(env_name)->setJoint("link_1_joint_1",-0.25);
//        world->getRobot(env_name)->setJoint("link_3_joint_0",1.57);

        // int surface_no = 4; //maze vertical surface number
        int surface_no = 5; // regular mazes

        std::this_thread::sleep_for(std::chrono::milliseconds(20000));

        size_t i =0;


        Eigen::VectorXd backup_state_fetch(int(world->getRobot(fetch_dart->getName())->getGroupJoints(GROUP_X).size()));

        world->getRobot(fetch_name)->getGroupState(GROUP_X, backup_state_fetch);
        while (i < actions_robot.size())
        {

            if(actions_robot[i].obj_index == 3)
                surface_no = 4;
            else surface_no = 5;

            Eigen::VectorXd old_config = builder.getStartConfiguration();
            if (plan_to_grasp(world, window, obj_[actions_robot[i].obj_index], surface_no, true, fetch_dart,backup_state_fetch))
            {
                if(plan_to_move(world, window, obj_[actions_robot[i].obj_index], actions_robot[i], fetch_dart, maze_dart,surface_no))
                {
                    builder.setStartConfigurationFromWorld();
                    start_config = builder.getStartConfiguration();
                    world->getRobot(fetch_name)->getGroupState(GROUP_X, backup_state_fetch);
                    i++;

                }else{
                    world->getRobot(fetch_name)->setGroupState(GROUP_X,backup_state_fetch);
                }
            }else{
                world->getRobot(fetch_name)->setGroupState(GROUP_X,backup_state_fetch);
            }
//                start_config = old_config;
        }
        plan_to_fold_arm(world,window);

        while(!plan_to_move_robot(world,window, stdvec_to_eigen_vec(input_.goal_pose))){
            world->getRobot(fetch_name)->setGroupState(GROUP_X,backup_state_fetch);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    });

    return 0;
}