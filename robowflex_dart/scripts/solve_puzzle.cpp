//
// Created by serboba on 23.03.22.
//

//
// Created by serboba on 15.12.21.
//

//
// Created by serboba on 06.12.21.
//


//
// Created by serboba on 12.10.21.
//

//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <robowflex_dart/RRTnew.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/solution_parser.h>
#include <robowflex_dart/IsoManipulationOptimization.h>
#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/urdf_read.h>

boost::filesystem::path p(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = p.string() + "/src/robowflex/robowflex_dart/include/io/";

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);
    double time;
    std::string env_name;

    if(argc > 1 )
    {
        env_name = std::string(argv[1]);
        time = atof(argv[2]);
    }
    else{
        env_name = "maze3"; // test in cpp
        time = 10;
    }

    auto maze_dart = darts::loadMoveItRobot(env_name,
                                            abs_path +"envs/" + env_name+ "/" + "urdf/" + env_name + ".urdf",
                                            abs_path +"envs/" + env_name+ "/" + "srdf/" + env_name + ".srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(maze_dart);


    URDF_IO input_(env_name);

    darts::PlanBuilder builder(world,input_.group_indices); // using my statespace


    for(std::string group : input_.group_names) {
        builder.addGroup(maze_name,group);
    }
        // ADD ALL GROUPS THAT ARE NEEDED

    builder.setStartConfigurationFromWorld();

    builder.initialize();


    darts::TSR::Specification goal_spec;
    goal_spec.setFrame(maze_name, "link_0_joint_0", "base_link");
    goal_spec.setPose(input_.goal_pose);
    auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
    auto goal = builder.getGoalTSR(goal_tsr);

    builder.setGoal(goal);


    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,input_.group_indices));
    auto planner = std::make_shared<ompl::geometric::RRTnew>(builder.info,input_.group_indices,true); // bool parameter is state isolation
        //auto planner = std::make_shared<ompl::geometric::RRTstar>(builder.info);
        //auto planner = std::make_shared<ompl::geometric::BITstar>(builder.info);
        //auto planner = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);

    builder.ss->setPlanner(planner);
    builder.setup();

    builder.space->sanityChecks();
    builder.rspace->sanityChecks();

    goal->startSampling();
    ompl::base::PlannerStatus solved = builder.ss->solve(time);
    goal->stopSampling();


    if (solved)
    {
        ompl::geometric::PathGeometric path(builder.getSolutionPath(false,false));
            //path.interpolate() if not rrtnew maybe
        std::string file_name = abs_path +"path_result/"+env_name + ".txt";
        std::ofstream fs(file_name);
        path.printAsMatrix(fs);

    }
        else
        {
            RBX_WARN("No solution found");
            std::string file_name = abs_path +"path_result/"+env_name + ".txt";
            std::ofstream fs(file_name);
        }



    return 0;
}

