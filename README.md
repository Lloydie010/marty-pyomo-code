# marty-pyomo-code

## enter

    #include <iostream>
    #include <string>
    #include "body.h"
    // Include other necessary headers
    
    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "unitree_gazebo_servo");
    
        string robot_name;
        std::vector<std::vector<double>> trajectories;
        std::vector<double> durations;
    
        ros::param::get("/robot_name", robot_name);
    
        // Load your trajectory data here
        loadTrajectoryFromFile("/path/to/your/trajectory_file.csv", trajectories, durations);
    
        cout << "robot_name: " << robot_name << endl;
        cout << "Trajectory steps: " << trajectories.size() << endl;
    
        multiThread listen_publish_obj(robot_name);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        usleep(300000);
    
        ros::NodeHandle n;
        // ... (rest of your publisher setup code)
    
        // Call the stand function
        stand();
    
        // Wait for user input
        std::cout << "Robot is in standing position. Press Enter to start the trajectory motion..." << std::endl;
        std::cin.get();
    
        // Execute the trajectory
        moveAllPosition(trajectories, durations);
    
        while (ros::ok()){
            lowState_pub.publish(lowState);
            sendServoCmd();
        }
        return 0;
    }

## python print csv

    import csv
    
    def write_trajectory_to_csv(m, filename):
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            
            for n in m.N:
                row = [
                    0,  # j1 (always 0)
                    m.q[n, 'thul1'].value,  # j2
                    m.q[n, 'thll1'].value,  # j3
                    0,  # j4 (always 0)
                    m.q[n, 'thul1'].value,  # j5 (same as j2)
                    m.q[n, 'thll1'].value,  # j6 (same as j3)
                    0,  # j7 (always 0)
                    m.q[n, 'thul2'].value,  # j8
                    m.q[n, 'thll2'].value,  # j9
                    0,  # j10 (always 0)
                    m.q[n, 'thul2'].value,  # j11 (same as j8)
                    m.q[n, 'thll2'].value,  # j12 (same as j9)
                    m.dt[n].value  # duration
                ]
                writer.writerow(row)
    
    # Usage:
    # Assuming 'm' is your Pyomo model
    write_trajectory_to_csv(m, 'trajectory.csv')


    import csv
    
    def write_trajectory_to_csv(m, filename, decimal_places=6):
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            
            for n in m.N:
                row = [
                    0,  # j1 (always 0)
                    round(m.q[n, 'thul1'].value, decimal_places),  # j2
                    round(m.q[n, 'thll1'].value, decimal_places),  # j3
                    0,  # j4 (always 0)
                    round(m.q[n, 'thul1'].value, decimal_places),  # j5 (same as j2)
                    round(m.q[n, 'thll1'].value, decimal_places),  # j6 (same as j3)
                    0,  # j7 (always 0)
                    round(m.q[n, 'thul2'].value, decimal_places),  # j8
                    round(m.q[n, 'thll2'].value, decimal_places),  # j9
                    0,  # j10 (always 0)
                    round(m.q[n, 'thul2'].value, decimal_places),  # j11 (same as j8)
                    round(m.q[n, 'thll2'].value, decimal_places),  # j12 (same as j9)
                    round(m.dt[n].value, decimal_places)  # duration
                ]
                writer.writerow(row)
    
    # Usage:
    # Assuming 'm' is your Pyomo model
    write_trajectory_to_csv(m, 'trajectory.csv', decimal_places=4)

## Code for copying

    void move_groups(int group_number, const std::vector<double>& positions, const std::vector<double>& durations)
    {
        if (positions.size() != durations.size()) {
            ROS_ERROR("Number of positions and durations must match");
            return;
        }
    
        paramInit();
    
        double lastPos[12];
        for (int j = 0; j < 12; j++)
            lastPos[j] = lowState.motorState[j].q;
    
        for (size_t step = 0; step < positions.size(); step++) {
            double position = positions[step];
            double duration = durations[step];
    
            double start_time = ros::Time::now().toSec();
            double end_time = start_time + duration;
    
            while (ros::Time::now().toSec() < end_time) {
                if (!ros::ok()) return;
    
                double percent = (ros::Time::now().toSec() - start_time) / duration;
                percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
                if (group_number == 0) {
                    lowCmd.motorCmd[0].q = lastPos[0] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[3].q = lastPos[3] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[6].q = lastPos[6] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[9].q = lastPos[9] * (1 - percent) + position * percent;
                } else if (group_number == 1) {
                    lowCmd.motorCmd[1].q = lastPos[1] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[4].q = lastPos[4] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[7].q = lastPos[7] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[10].q = lastPos[10] * (1 - percent) + position * percent;
                } else if (group_number == 2) {
                    lowCmd.motorCmd[2].q = lastPos[2] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[5].q = lastPos[5] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[8].q = lastPos[8] * (1 - percent) + position * percent;
                    lowCmd.motorCmd[11].q = lastPos[11] * (1 - percent) + position * percent;
                }
    
                sendServoCmd();
                ros::spinOnce();
            }
    
            // Update lastPos for the next step
            for (int j = 0; j < 12; j++)
                lastPos[j] = lowCmd.motorCmd[j].q;
        }
    }

## more

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "unitree_gazebo_servo");
    
        string robot_name;
        int group_number;
        std::vector<double> positions;
        std::vector<double> durations;
    
        ros::param::get("/robot_name", robot_name);
        ros::param::get("/group_number", group_number);
    
        // Load your trajectory data here
        // This is just an example, replace with your actual data loading method
        positions = {0.1, 0.2, 0.3, 0.4, 0.5};
        durations = {0.016, 0.018, 0.02, 0.019, 0.017};
    
        cout << "robot_name: " << robot_name << endl;
        cout << "Trajectory steps: " << positions.size() << endl;
    
        multiThread listen_publish_obj(robot_name);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        usleep(300000);
    
        ros::NodeHandle n;
        // ... (rest of your publisher setup code)
    
        move_groups(group_number, positions, durations);
    
        while (ros::ok()){
            lowState_pub.publish(lowState);
            sendServoCmd();
        }
        return 0;
    }

# more launch

    <launch>
      <param name="/robot_name" value="go1" />
      <param name="/group_number" value="$(arg group)" />
      <!-- Add a parameter for trajectory file if needed -->
      <!-- <param name="/trajectory_file" value="$(arg trajectory)" /> -->
    
      <node name="group_joint" pkg="gerald_second" type="gerald_group"  />
    </launch>

## file input

    #include <fstream>
    #include <sstream>
    
    bool loadTrajectoryFromFile(const std::string& filename, std::vector<double>& positions, std::vector<double>& durations)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file: %s", filename.c_str());
            return false;
        }
    
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string position_str, duration_str;
            if (std::getline(ss, position_str, ',') && std::getline(ss, duration_str, ',')) {
                positions.push_back(std::stod(position_str));
                durations.push_back(std::stod(duration_str));
            }
        }
    
        return true;
    }

## main

    std::string trajectory_file;
    ros::param::get("/trajectory_file", trajectory_file);
    if (!loadTrajectoryFromFile(trajectory_file, positions, durations)) {
        ROS_ERROR("Failed to load trajectory file");
        return 1;
    }

## move all

    void moveAllPosition(const std::vector<std::vector<double>>& trajectories, const std::vector<double>& durations)
    {
        if (trajectories.empty() || trajectories[0].size() != 12 || trajectories.size() != durations.size()) {
            ROS_ERROR("Invalid trajectory data");
            return;
        }
    
        paramInit();
    
        double lastPos[12];
        for (int j = 0; j < 12; j++) {
            lastPos[j] = lowState.motorState[j].q;
        }
    
        for (size_t step = 0; step < trajectories.size(); step++) {
            const auto& targetPos = trajectories[step];
            double duration = durations[step];
    
            double start_time = ros::Time::now().toSec();
            double end_time = start_time + duration;
    
            while (ros::Time::now().toSec() < end_time) {
                if (!ros::ok()) return;
    
                double percent = (ros::Time::now().toSec() - start_time) / duration;
                percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
                for (int j = 0; j < 12; j++) {
                    lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                }
    
                sendServoCmd();
                ros::spinOnce();
            }
    
            // Update lastPos for the next step
            for (int j = 0; j < 12; j++) {
                lastPos[j] = lowCmd.motorCmd[j].q;
            }
        }
    }

## servocpp



    int main(int argc, char **argv)
        {
        ros::init(argc, argv, "unitree_gazebo_servo");
    
        string robot_name;
        std::vector<std::vector<double>> trajectories;
        std::vector<double> durations;
    
        ros::param::get("/robot_name", robot_name);
    
        // Load your trajectory data here
        // This is just an example, replace with your actual data loading method
        loadTrajectoryFromFile("/path/to/your/trajectory_file.csv", trajectories, durations);
    
        cout << "robot_name: " << robot_name << endl;
        cout << "Trajectory steps: " << trajectories.size() << endl;
    
        multiThread listen_publish_obj(robot_name);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        usleep(300000);
    
        ros::NodeHandle n;
        // ... (rest of your publisher setup code)
    
        moveAllPosition(trajectories, durations);
    
        while (ros::ok()){
            lowState_pub.publish(lowState);
            sendServoCmd();
        }
        return 0;
        }

## another read file ex

    #include <fstream>
    #include <sstream>
    
    bool loadTrajectoryFromFile(const std::string& filename, std::vector<std::vector<double>>& trajectories, std::vector<double>& durations)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file: %s", filename.c_str());
            return false;
        }
    
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> position_values;
            std::string value;
    
            // Read 12 position values
            for (int i = 0; i < 12; i++) {
                if (!std::getline(ss, value, ',')) {
                    ROS_ERROR("Invalid line in trajectory file");
                    return false;
                }
                position_values.push_back(std::stod(value));
            }
    
            // Read duration
            if (!std::getline(ss, value, ',')) {
                ROS_ERROR("Invalid line in trajectory file");
                return false;
            }
            double duration = std::stod(value);
    
            trajectories.push_back(position_values);
            durations.push_back(duration);
        }
    
        return true;
    }

## another launch

    <launch>
      <param name="/robot_name" value="go1" />
      <rosparam param="/group_numbers">[0, 1, 2]</rosparam>
      <param name="/joint_angle_position" value="$(arg rad_angle)" />
      <param name="/duration" value="$(arg duration)" />
    
      <node name="group_joint" pkg="gerald_second" type="gerald_group"  />
    </launch>
