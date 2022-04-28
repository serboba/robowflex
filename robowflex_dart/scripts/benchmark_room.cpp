//
// Created by serboba on 09.02.22.
//

//
// Created by serboba on 28.01.22.
//


#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>

#include <robowflex_library/builder.h>
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
#include <robowflex_dart/IsoManipulationOptimization.h>


#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/PathGeometric.h>

#include <robowflex_dart/LARRT.h>
#include <robowflex_dart/urdf_read.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <robowflex_dart/IsoStateSpace.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>


boost::filesystem::path p(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = p.string() + "/src/robowflex/robowflex_dart/include/io/";


using namespace ompl;



void postRunEvent(const base::PlannerPtr & planner, tools::Benchmark::RunProperties & run)
{

    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->stopSampling();

}


void preRunEvent(const base::PlannerPtr & planner){
    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->startSampling();

}

void benchmark(){


    std::string env_name = "room0";
    std::string fetch_name;

    if(env_name == "room0" || env_name == "room1")
        fetch_name = "fetch3";

    else if(env_name =="room2")
        fetch_name = "fetch2";

    auto fetch_dart = robowflex::darts::loadMoveItRobot("fetch",                                         //
                                                        abs_path +"envs/fetch/urdf/" + fetch_name + ".urdf",
                                                        abs_path +"envs/fetch/srdf/" + "fetch3" + ".srdf");


    auto room_dart = robowflex::darts::loadMoveItRobot("a1",
                                                       abs_path +"envs/" + env_name+ "/" + "urdf/" + env_name + ".urdf",
                                                       abs_path +"envs/" + env_name+ "/" + "srdf/" + env_name + ".srdf");


    auto world = std::make_shared<robowflex::darts::World>(); // achtung, world->robots_ is std map, automatically sorts robots by their names
    world->addRobot(room_dart);
    world->addRobot(fetch_dart);


    world->getRobot("fetch")->setJoint("torso_lift_joint",0.05);
    world->getRobot("fetch")->setJoint("shoulder_pan_joint",1.32);
    world->getRobot("fetch")->setJoint("shoulder_lift_joint",1.4);
    world->getRobot("fetch")->setJoint("upperarm_roll_joint",-0.2);
    world->getRobot("fetch")->setJoint("elbow_flex_joint",1.72);
    world->getRobot("fetch")->setJoint("forearm_roll_joint",0);
    world->getRobot("fetch")->setJoint("wrist_flex_joint",1.66);
    world->getRobot("fetch")->setJoint("wrist_roll_joint",0);


    URDF_IO input_(env_name);

    if(env_name == "room2" || env_name == "room3")  // start position is outside the room
    {
        world->getRobot("fetch")->setJoint("move_x_axis_joint",2.2);
        world->getRobot("fetch")->setJoint("move_y_axis_joint",-1.0);
    }else if(env_name=="room1")
    {
        world->getRobot("fetch")->setJoint("move_y_axis_joint",0.7);
        world->getRobot("fetch")->setJoint("move_x_axis_joint",-0.6);
    }


    int lastIndex = input_.group_indices.back().back();
    std::vector<int> robot_gr = {lastIndex+1,lastIndex+2};
    input_.group_indices.push_back(robot_gr);

    int goal_index = input_.group_indices.size()-1;

    robowflex::darts::PlanBuilder builder(world,input_.group_indices); // using my statespace

    for(std::string group : input_.group_names) {
        builder.addGroup("a1",group);
    }

    builder.addGroup("fetch","move_robot");

    builder.setStartConfigurationFromWorld();
    builder.initialize();

    robowflex::darts::TSR::Specification goal_spec;
    goal_spec.setFrame("fetch", "base_link", "move_x_axis");
    goal_spec.setPose(input_.goal_pose);            //  SET WANTED ROBOT POSITION (inside urdf)

    auto goal_tsr = std::make_shared<robowflex::darts::TSR>(world, goal_spec);
    auto goal = builder.getGoalTSR(goal_tsr);

    builder.setGoal(goal);

    builder.space->sanityChecks();
    builder.rspace->sanityChecks();

    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,input_.group_indices));

    builder.setup();

    double runtime_limit = 300.0;
    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    int run_count =100;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit,run_count);

    std::string b_name = "maze_Benchmarks";
    ompl::tools::Benchmark b(*builder.ss, b_name);


    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
    b.setPostRunEvent(
            [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });


    auto rrt_new1 =std::make_shared<geometric::LARRT>(builder.ss->getSpaceInformation(), input_.group_indices, true, goal_index);
    rrt_new1->setName("LA-RRT");
    b.addPlanner(rrt_new1);


//    b.addPlanner(std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation()));

//    b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::ABITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::AITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));

//    b.addPlanner(std::make_shared<geometric::BFMT>(builder.ss->getSpaceInformation()));


    b.addExperimentParameter("sampler_id", "INTEGER", "0"); // ?
    b.benchmark(request);

    std::string result_filename = abs_path + "benchmark_results/benchmark_room";
    b.saveResultsToFile(result_filename.c_str());

    std::string db_output = abs_path + "db_files/benchmark_room.db";

    std::string output = "python3 " + abs_path + "ompl_benchmark_statistics.py " + result_filename + " -d " + db_output;
    std::system(output.c_str());

}



int main(int argc, char **argv)
{
    benchmark();
    return 0;
}