//
// Created by serboba on 28.01.22.
//

#include <thread>

#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

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


using namespace ompl;

boost::filesystem::path p(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = p.string() + "/src/robowflex/robowflex_dart/include/io/";



void postRunEvent(const base::PlannerPtr & planner, tools::Benchmark::RunProperties & run)
{

    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->stopSampling();

}


void preRunEvent(const base::PlannerPtr & planner){
    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->startSampling();

}

void benchmark(std::string robot_name,std::string urdf_name, std::string srdf_name, double time, int run_count){
    auto puzzle = robowflex::darts::loadMoveItRobot(robot_name,urdf_name,srdf_name);

    auto puzzle_name = puzzle->getName();
    auto world = std::make_shared<robowflex::darts::World>();
    world->addRobot(puzzle);

    URDF_IO input_(robot_name);

    robowflex::darts::PlanBuilder builder(world,input_.group_indices);

    for(std::string group : input_.group_names)
        builder.addGroup(puzzle_name,group);

    builder.setStartConfigurationFromWorld();
    builder.initialize();

    robowflex::darts::TSR::Specification goal_spec;
    goal_spec.setFrame(puzzle_name, "link_0_joint_0", "base_link");
    goal_spec.setPose(input_.goal_pose);            //  SET WANTED CUBE POSITION
    auto goal_tsr = std::make_shared<robowflex::darts::TSR>(world, goal_spec);
    auto goal = builder.getGoalTSR(goal_tsr);


    builder.setGoal(goal);

    //builder.space->setLongestValidSegmentFraction(0.01);
    builder.space->sanityChecks();
    builder.rspace->sanityChecks();

    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,input_.group_indices));

    builder.setup();

    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly

    ompl::tools::Benchmark::Request request(time, memory_limit,run_count);

    std::string b_name = "maze_Benchmarks";
    ompl::tools::Benchmark b(*builder.ss, b_name);



    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
    b.setPostRunEvent(
            [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });


    auto rrt_new1 =std::make_shared<geometric::LARRT>(builder.ss->getSpaceInformation(), input_.group_indices, true);
    rrt_new1->setName("LA-RRT");
    b.addPlanner(rrt_new1);


//    b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::ABITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::AITstar>(builder.ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));


    /*   b.addPlanner(std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation()));
       b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));
       b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));


       b.addPlanner(std::make_shared<geometric::ABITstar>(builder.ss->getSpaceInformation()));
       b.addPlanner(std::make_shared<geometric::AITstar>(builder.ss->getSpaceInformation()));
       b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));
       b.addPlanner(std::make_shared<geometric::KPIECE1>(builder.ss->getSpaceInformation()));
   */

 //   b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));

    b.addExperimentParameter("sampler_id", "INTEGER", "0"); // ?
    b.benchmark(request);

    std::string result_filename = abs_path + "benchmark_results/"+robot_name;
    b.saveResultsToFile(result_filename.c_str());

    std::string db_output = abs_path + "db_files/"+robot_name+".db";

    std::string output = "python3 " + abs_path + "ompl_benchmark_statistics.py " + result_filename + " -d " + db_output;
    std::system(output.c_str());

}



int main(int argc, char **argv)
{
    //(maybe TODO get multiple puzzles to benchmark as argument, or config as the list of planners that will be used)
    std::string env_name;
    if(argc > 1 )
        env_name = std::string(argv[1]);
    else
        env_name = "maze3"; // test in cpp

    double time;
    int run_count;

    if(argc >3)
    {
        time = atof(argv[2]);
        run_count = atoi(argv[3]);
    }
    else
    {
        time = 60.0;
        run_count = 5;
    }



    benchmark(env_name,
              abs_path +"envs/" + env_name+ "/" + "urdf/" + env_name + ".urdf",
              abs_path +"envs/" + env_name+ "/" + "srdf/" + env_name + ".srdf",
              time,run_count);


//
//    benchmark("maze1",
//              abs_path +"envs/" + "maze1"+ "/" + "urdf/" + "maze1" + ".urdf",
//              abs_path +"envs/" + "maze1"+ "/" + "srdf/" + "maze1" + ".srdf",
//              60.0,100);
//
//
//
//    benchmark("maze2",
//              abs_path +"envs/" + "maze2"+ "/" + "urdf/" + "maze2" + ".urdf",
//              abs_path +"envs/" + "maze2"+ "/" + "srdf/" + "maze2" + ".srdf",
//              60.0,100);
//
//
//    benchmark("maze3",
//              abs_path +"envs/" + "maze3"+ "/" + "urdf/" + "maze3" + ".urdf",
//              abs_path +"envs/" + "maze3"+ "/" + "srdf/" + "maze3" + ".srdf",
//              60.0,100);
//
//
//    benchmark("maze_vertical",
//              abs_path +"envs/" + "maze_vertical"+ "/" + "urdf/" + "maze_vertical" + ".urdf",
//              abs_path +"envs/" + "maze_vertical"+ "/" + "srdf/" + "maze_vertical" + ".srdf",
//              60.0,100);
//
//
////    benchmark("maze_vertical",
////              abs_path +"envs/" + "maze_vertical"+ "/" + "urdf/" + "maze_vertical" + ".urdf",
////              abs_path +"envs/" + "maze_vertical"+ "/" + "srdf/" + "maze_vertical" + ".srdf",
////              60.0,100);


    return 0;
}