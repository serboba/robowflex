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

#include <robowflex_dart/RRTnew.h>
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


//    planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>()->interpolate();
}


void preRunEvent(const base::PlannerPtr & planner){
    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->startSampling();

}

void benchmark(){
    auto fetch_dart = robowflex::darts::loadMoveItRobot("fetch",                                         //
                                             abs_path +"meshes/fetch3.urdf",  //
                                             abs_path +"meshes/fetch3.srdf");

    auto door_dart = robowflex::darts::loadMoveItRobot("escape_scene",
                                            abs_path+"envs/room.urdf",
                                            abs_path+"envs/room.srdf");


    auto world = std::make_shared<robowflex::darts::World>();
    world->addRobot(door_dart);
    world->addRobot(fetch_dart);

    std::vector<std::vector<int>> group_indices;


    std::vector<int> gr2 ={0,1}; // cub
    std::vector<int> gr3 ={2,3}; //
    std::vector<int> gr4 ={4}; //
    std::vector<int> gr5 ={5}; //

    std::vector<int> gr1 ={6,7}; // moverobot

    group_indices.push_back(gr2);
    group_indices.push_back(gr3);
    group_indices.push_back(gr4);
    group_indices.push_back(gr5);

    group_indices.push_back(gr1);

    world->getRobot("fetch")->setJoint("torso_lift_joint",0.05);
    world->getRobot("fetch")->setJoint("shoulder_pan_joint",1.32);
    world->getRobot("fetch")->setJoint("shoulder_lift_joint",1.4);
    world->getRobot("fetch")->setJoint("upperarm_roll_joint",-0.2);
    world->getRobot("fetch")->setJoint("elbow_flex_joint",1.72);
    world->getRobot("fetch")->setJoint("forearm_roll_joint",0);
    world->getRobot("fetch")->setJoint("wrist_flex_joint",1.66);
    world->getRobot("fetch")->setJoint("wrist_roll_joint",0);


    robowflex::darts::PlanBuilder builder(world,group_indices);
    builder.addGroup("escape_scene","cube1_gr1");
    builder.addGroup("escape_scene","cube_gr2");
    builder.addGroup("escape_scene","front_door_j");
    builder.addGroup("escape_scene","door_lock_j");

    builder.addGroup("fetch","move_robot");

    builder.setStartConfigurationFromWorld();
    builder.initialize();

    robowflex::darts::TSR::Specification goal_spec;
    goal_spec.setFrame("fetch", "base_link", "move_x_axis");
    goal_spec.setPose(2.0,0.0,0.0,
                      1.0,0.0,0.0,0.0);            //  SET WANTED CUBE POSITION
    auto goal_tsr = std::make_shared<robowflex::darts::TSR>(world, goal_spec);
    auto goal = builder.getGoalTSR(goal_tsr);


    builder.setGoal(goal);


    builder.space->setLongestValidSegmentFraction(0.01);

    builder.space->sanityChecks();
    builder.rspace->sanityChecks();

    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,group_indices));

    builder.setup();

    double runtime_limit = 10.0;
    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    int run_count =5;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit,run_count);

    std::string b_name = "maze_Benchmarks";
    ompl::tools::Benchmark b(*builder.ss, b_name);


    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
    b.setPostRunEvent(
            [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });



    b.addPlanner(std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));



    /*
      //b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
      auto lbtrrt =std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation());
      lbtrrt->setRange(0.1);
      b.addPlanner(lbtrrt);
  */

//    b.addPlanner(std::make_shared<geometric::ABITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::AITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));

//    b.addPlanner(std::make_shared<geometric::BFMT>(builder.ss->getSpaceInformation()));


   /* auto rrt_new1 =std::make_shared<geometric::RRTnew>(builder.ss->getSpaceInformation(),group_indices,false,true);
    rrt_new1->setName("LA-RRT");
    b.addPlanner(rrt_new1);
*/

    b.addExperimentParameter("sampler_id", "INTEGER", "0"); // ?
    b.benchmark(request);

    std::string result_filename = abs_path + "benchmark_results/benchmark_room";
    b.saveResultsToFile(result_filename.c_str());

    std::string db_output = abs_path + "db_files/benchmark_room.db";

    std::string output = "python " + abs_path + "ompl_benchmark_statistics.py " + result_filename + " -d " + db_output;
    std::system(output.c_str());

}



int main(int argc, char **argv)
{
    benchmark();
    return 0;
}