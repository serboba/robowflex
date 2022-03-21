////
//// Created by serboba on 03.12.21.
////
//
////
//// Created by serboba on 13.10.21.
////
//
////
//// Created by serboba on 05.09.21.
////
//
////
//// Created by serboba 19.11.21.
////
//
//#include <chrono>
//#include <thread>
//
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/kpiece/KPIECE1.h>
////#include <ompl/geometric/planners/rrt/RRT.h>
////#include <ompl/geometric/planners/sst/SST.h>
//
//#include <robowflex_library/builder.h>
//#include <robowflex_library/log.h>
//#include <robowflex_library/robot.h>
//#include <robowflex_library/scene.h>
//#include <robowflex_library/util.h>
//#include <ompl/base/goals/GoalLazySamples.h>
//#include <robowflex_library/class_forward.h>
//#include <robowflex_dart/gui.h>
//#include <robowflex_dart/planning.h>
//#include <robowflex_dart/robot.h>
//#include <robowflex_dart/tsr.h>
//#include <robowflex_dart/world.h>
//
//#include <robowflex_dart/point_collector.h>
//#include <robowflex_dart/conversion_functions.h>
//#include <robowflex_dart/quaternion_factory.h>
//#include <robowflex_dart/Object.h>
//#include <robowflex_dart/planningFunctions.h>
//
//using namespace robowflex;
//
//static const std::string GROUP = "arm_with_torso";
//static const std::string GROUP_X = "arm_with_x_move";
//
//// todo needs recheck completely
//
//int main(int argc, char **argv)
//{
//    // Startup ROS
//    ROS ros(argc, argv);
//
//    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE */
//    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
//                                             "/home/serboba/Desktop/blenderFLEX/fetch3.urdf",  //
//                                             "/home/serboba/Desktop/blenderFLEX/fetch3.srdf");
//
//    auto door_dart = darts::loadMoveItRobot("myfirst",
//                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/room.urdf",
//                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/room.srdf");
//
//    auto fetch_name = fetch_dart->getName();
//    auto door_name = door_dart->getName();
//    auto world = std::make_shared<darts::World>();
//    world->addRobot(fetch_dart);
//    world->addRobot(door_dart);
//
//    std::string urdf_name = "room.txt";
//    std::string srdf_name = "room_srdf.txt";
//
//    create_objects_from_urdf(urdf_name, srdf_name);
//    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE UNTIL HERE !!!! */
//    darts::Window window(world);
//
//
//
//    window.run([&] {
//        //create_txt_from_urdf();
//        std::vector<Object> obj_s = get_objects();
//        std::this_thread::sleep_for(std::chrono::milliseconds(6000));
//
//        darts::PlanBuilder builder(world);
//        builder.addGroup("fetch", GROUP_X);
//        Eigen::VectorXd start_config = builder.getStartConfiguration();
//        if(start_config.data() == NULL){
//            builder.setStartConfigurationFromWorld();
//            std::cout<< "hop" << std::endl;
//            std::cout << builder.getStartConfiguration()<< std::endl;
//            start_config = builder.getStartConfiguration();
//        }
//
//        if(start_config[0] == 0.0) // if its the starting(beginning) motion the arm may be collide therefore adjust the torso as you want
//            start_config[0] += 0.10;
//
//        int chosen_surface_no = 5;
//        bool normals_help = false;
//
//        std::vector<Eigen::Vector3d> degrees;
//        Eigen::Vector3d  d_;
//        d_ << 1.56,0.0,0.0;
//        degrees.push_back(d_);
//        d_ << 0.0,0.0,1.56;
//        degrees.push_back(d_);
//
//
//        while(true){
//            plan_to_grasp(world, window, obj_s[1], chosen_surface_no, false, fetch_dart,start_config); // door1
//            if(plan_to_move_xyz_axis(world,window,obj_s[1],0,-1.10,fetch_dart,door_dart))
//                break;
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//         builder.setStartConfigurationFromWorld();
//        start_config = builder.getStartConfiguration();
//        chosen_surface_no = 5;
//
//        while(true){
//            plan_to_grasp(world, window, obj_s[0], chosen_surface_no, false, fetch_dart,start_config); // door1
//            if(plan_to_move_xyz_axis(world,window,obj_s[0],1,-1.0,fetch_dart,door_dart))
//                break;
//        }
//
//        builder.setStartConfigurationFromWorld();
//        start_config = builder.getStartConfiguration();
//
//        chosen_surface_no = 4;
//
//        while(true){
//            plan_to_grasp(world, window, obj_s[2], chosen_surface_no, true, fetch_dart,start_config); // door1
//            if(!plan_to_rotate_rpy(world,window,obj_s[2],degrees[0], chosen_surface_no,fetch_dart,door_dart))
//                break;
//
//        }
//
//        builder.setStartConfigurationFromWorld();
//        start_config = builder.getStartConfiguration();
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//        while(true){
//            plan_to_grasp(world, window, obj_s[3], chosen_surface_no, true, fetch_dart,start_config); // door1
//            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//            if(plan_to_rotate_rpy(world,window,obj_s[3],degrees[1], chosen_surface_no,fetch_dart,door_dart))
//                break;
//        }
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//
//        builder.setStartConfigurationFromWorld();
//        start_config = builder.getStartConfiguration();
//        plan_to_fold_arm(world,window);
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//
//        Vector3d np;
//        np << 2.0, 0.0,0.0;
//        plan_to_move_robot(world,window,np);
//
//    });
//
//    return 0;
//}
