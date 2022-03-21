//
// Created by serboba on 28.01.22.
//

#include <robowflex_dart/urdf_read.h>
#include <robowflex_dart/conversion_functions.h>
#include <boost/filesystem.hpp>

boost::filesystem::path p_(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path_ = p_.string() + "/src/robowflex/robowflex_dart/include/io/";


void create_txt_file(std::string filename){

    std::string script = "python " + abs_path_ + "urdf2config.py " + filename;
    std::system(script.c_str());
}


URDF_IO::URDF_IO(std::string filename) {

    create_txt_file(filename);

    std::vector<std::string> gr_names;
    std::vector<std::vector<int>> gr_indices;
    std::vector<double> goal_p;


    std::string str = abs_path_+"txt_files/" + filename + ".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME
    std::string gr_name, num;

    if (inFile.is_open()) {
        std::string line;
        while (std::getline(inFile, line)) {
            std::stringstream ss(line);
            if (line.compare("-") != 0) {
                std::getline(ss, gr_name, ',');
                group_names.push_back(gr_name);

                std::vector<int> gr_temp;
                int dim;
                std::getline(ss, num, ',');
                dim = std::stoi(num);
                int start;
                std::getline(ss, num, ',');
                start = std::stoi(num);

                for(int i = start; i < start+dim; i++){
                    gr_temp.push_back(i);
                }

                gr_indices.push_back(gr_temp);

            } else {
                //last line goal
                std::getline(inFile, line);
                std::stringstream ss2(line);

                for(int i = 0; i< 3; i++){
                    std::getline(ss2, num, ' ');
                    goal_pose.push_back(stod(num));
                }
                std::vector<double> rotation_;
                for(int i = 0; i<3 ; i++){
                    std::getline(ss2,num,' ');
                    rotation_.push_back(stod(num));
                }

                std::vector<double> orn = quaternion_matrix_to_stdvec(rpy_to_quaternion(rotation_[0],rotation_[1],rotation_[2]));
                goal_pose.insert(goal_pose.end(),orn.begin(),orn.end());
            }
        }
    }

    group_indices = gr_indices;

}
