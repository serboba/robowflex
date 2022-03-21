//
// Created by serboba on 09.02.22.
//

//
// Created by serboba on 28.01.22.
//

// TEST FILE
//#include <chrono>
//#include <thread>
//
//#include <ompl/geometric/SimpleSetup.h>
//
//#include <robowflex_library/builder.h>
//#include <robowflex_library/log.h>
//#include <robowflex_library/planning.h>
//#include <robowflex_library/robot.h>
//#include <robowflex_library/scene.h>
//#include <robowflex_library/tf.h>
//#include <robowflex_library/util.h>
//
//#include <robowflex_dart/gui.h>
//#include <robowflex_dart/planning.h>
//#include <robowflex_dart/robot.h>
//#include <robowflex_dart/space.h>
//#include <robowflex_dart/tsr.h>
//#include <robowflex_dart/world.h>
//#include <robowflex_dart/IsoManipulationOptimization.h>
//
//
//#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
//#include <ompl/geometric/planners/kpiece/KPIECE1.h>
//#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
//#include <ompl/geometric/planners/prm/PRM.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/rrt/LBTRRT.h>
//
//#include <ompl/geometric/planners/sbl/SBL.h>
//#include <ompl/geometric/planners/fmt/FMT.h>
//#include <ompl/geometric/planners/fmt/BFMT.h>
//#include <ompl/geometric/planners/informedtrees/BITstar.h>
//#include <ompl/geometric/planners/informedtrees/AITstar.h>
//#include <ompl/geometric/planners/informedtrees/ABITstar.h>
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/PathGeometric.h>
//
//#include <robowflex_dart/RRTnew.h>
//#include <robowflex_dart/urdf_read.h>
//#include <ompl/tools/benchmark/Benchmark.h>
//
//#include <ompl/base/samplers/UniformValidStateSampler.h>
//
//
//using namespace ompl;
//
//
//
//void postRunEvent(const base::PlannerPtr & planner, tools::Benchmark::RunProperties & run)
//{
//
//    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->stopSampling();
//
//
////    planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>()->interpolate();
//}
//
//
//void preRunEvent(const base::PlannerPtr & planner){
//    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->startSampling();
//
//}
//
//int main(int argc, char **argv)
//{
//
//    auto fetch_dart = robowflex::darts::loadMoveItRobot("fetch",                                         //
//                                                        "/home/serboba/Desktop/blenderFLEX/fetch3.urdf",  //
//                                                        "/home/serboba/Desktop/blenderFLEX/fetch3.srdf");
//
//    auto door_dart = robowflex::darts::loadMoveItRobot("escape_scene",
//                                                       "/home/serboba/rb_ws/devel/lib/robowflex_dart/room.urdf",
//                                                       "/home/serboba/rb_ws/devel/lib/robowflex_dart/room.srdf");
//
//
//    auto world = std::make_shared<robowflex::darts::World>();
//    world->addRobot(door_dart);
//    world->addRobot(fetch_dart);
//
//
//    robowflex::darts::Window window(world);
//
//    std::vector<std::vector<int>> group_indices;
//
//
//    std::vector<int> gr2 ={0,1}; // cub
//    std::vector<int> gr3 ={2,3}; //
//    std::vector<int> gr4 ={4}; //
//    std::vector<int> gr5 ={5}; //
//
//    std::vector<int> gr1 ={6,7}; // moverobot
//
//    group_indices.push_back(gr2);
//    group_indices.push_back(gr3);
//    group_indices.push_back(gr4);
//    group_indices.push_back(gr5);
//
//    group_indices.push_back(gr1);
//
//
//
//    const auto &plan_solution_all = [&]() {
//
//    robowflex::darts::PlanBuilder builder(world);
//    world->getRobot("fetch")->setJoint("torso_lift_joint",0.05);
//    world->getRobot("fetch")->setJoint("shoulder_pan_joint",1.32);
//    world->getRobot("fetch")->setJoint("shoulder_lift_joint",1.4);
//    world->getRobot("fetch")->setJoint("upperarm_roll_joint",-0.2);
//    world->getRobot("fetch")->setJoint("elbow_flex_joint",1.72);
//    world->getRobot("fetch")->setJoint("forearm_roll_joint",0);
//    world->getRobot("fetch")->setJoint("wrist_flex_joint",1.66);
//    world->getRobot("fetch")->setJoint("wrist_roll_joint",0);
//
//
//    builder.addGroup("escape_scene","cube1_gr1");
//    builder.addGroup("escape_scene","cube_gr2");
//    builder.addGroup("escape_scene","front_door_j");
//    builder.addGroup("escape_scene","door_lock_j");
//
//    builder.addGroup("fetch","move_robot");
//
//    builder.setStartConfigurationFromWorld();
//    builder.initialize();
//
//
//       // std::this_thread::sleep_for(std::chrono::milliseconds(200000));
//
//    robowflex::darts::TSR::Specification goal_spec;
//    goal_spec.setFrame("fetch", "base_link", "move_x_axis");
//    goal_spec.setPose(2.0,0.0,0.0,
//                      1.0,0.0,0.0,0.0);            //  SET WANTED CUBE POSITION
//    auto goal_tsr = std::make_shared<robowflex::darts::TSR>(world, goal_spec);
//    auto goal = builder.getGoalTSR(goal_tsr);
//
//
//    builder.setGoal(goal);
//
//    builder.setGroupIndices(group_indices);
//
//    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,group_indices));
//    // auto planner = std::make_shared<ompl::geometric::RRTnew>(builder.info,input_.group_indices,false,false);
//    auto planner = std::make_shared<ompl::geometric::RRTnew>(builder.info,group_indices,false,true);
//    //  auto planner = std::make_shared<ompl::geometric::RRTstar>(builder.info);
//    //  auto planner = std::make_shared<ompl::geometric::BITstar>(builder.info);
//    // auto planner = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);
//    //planner->setRange(0.1);
//    // builder.space->setLongestValidSegmentFraction(0.1);
//    // builder.space->setValidSegmentCountFactor(2);
//
//    builder.ss->setPlanner(planner);
//    builder.setup();
//
//
//
//    builder.space->sanityChecks();
//    builder.rspace->sanityChecks();
//
//    goal->startSampling();
//    ompl::base::PlannerStatus solved = builder.ss->solve(300);
//    goal->stopSampling();
//
//
//
//    if (solved)
//    {
//        ompl::geometric::PathGeometric path(builder.getSolutionPath(false,false));
//
//        //path.interpolate() if not rrtnew
//
//        std::cout << "path" << std::endl;
//        std::cout << path.getStateCount() << std::endl;
//        std::ofstream fs("maze3doors.txt");
//        path.printAsMatrix(fs);
//
//        window.animatePath(builder, path,4,1);
//
//
//    }else
//        RBX_WARN("No solution found");
//
//};
//
//    window.run([&]{
//
//        plan_solution_all();
//    });
//    return 0;
//}