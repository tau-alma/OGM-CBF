/**
 * @file io.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains C++ operations for parsing and writing csv files.
 * @version 0.1
 * @date 2024-26-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include "datatypes.h"
#include <sstream>
#include <cstdlib>


/**
 * @brief Function for parsing a one instance of pointcloud from csv files
 * 
 * @param line the line from the csv file to be parsed
 * @return vector<LidarData> collection of the LidarData type in vector
 */
std::vector<LidarData> parse_cloud(std::string line) {
    std::vector<LidarData> pointCloud;

    LidarData point;
    char* token = strtok(&line[0], ",");
    
    // Read the timestamp
    if (token != NULL) {
        point.timestamp = std::stof(token);
    } else {
        std::cerr << "Error parsing timestamp on line: " << line << std::endl;
    }

    token = strtok(NULL, ",");

    try
    {

    
        while (token != NULL) {
            point.x = std::stof(token);

            token = strtok(NULL, ",");
            point.y = std::stof(token);

            token = strtok(NULL, ",");
            point.z = std::stof(token);

            token = strtok(NULL, ",");
            point.intensity = std::stof(token);

            // Print and add the point to the vector
            //std::cout << "Timestamp: " << point.timestamp << " X: " << point.x << " Y: " << point.y << " Z: " << point.z << " Intensity: " << point.intensity << std::endl;
            pointCloud.push_back(point);

            token = strtok(NULL, ",");
        }

    }
    catch(const std::invalid_argument)
    {
        std::cerr << "bad csv input" << '\n';
    }

    return pointCloud;

}

/**
 * @brief Function for parsing a one instance of state from csv files
 * 
 * @param line the line from the csv file to be parsed
 * @param state Reference to the list of floats to be filled
 */
void parse_state(std::string gt_line, float * state) {

    try
    {
        
        std::istringstream ss(gt_line);
        std::string token_gt;

        // Parse each token and assign it to the corresponding variable
        if (std::getline(ss, token_gt, ',')) state[0] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[1] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[2] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[3] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[4] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[5] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[6] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[7] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[8] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[9] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[10] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[11] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[12] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[13] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[14] = std::stof(token_gt);
        if (std::getline(ss, token_gt, ',')) state[15] = std::stof(token_gt);
    }
    catch(const std::invalid_argument)
    {
        std::cerr << "bad input" << '\n';
    }
}

/**
 * @brief Write one instance of the OccypancyGrid to csv file 
 * 
 * @param ogm OccypancyGrid to be written to the file
 * @param filename Direct file and path where to write the file
 */
void writeGridToCSV(const OccypancyGrid ogm, const std::string& filename) {
    std::ofstream file(filename); // Open file for writing
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return;
    }

    for (int x = 0; x < ogm.width; x++) {
        for (int y = 0; y < ogm.height; y++) {
            file << ogm.grid[x][y];
            if (x != ogm.width - 1) {
                file << ",";
            }
        }
        file << "\n";

    }

    file.close(); // Close file
}


