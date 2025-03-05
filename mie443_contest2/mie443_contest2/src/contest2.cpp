// functions to be made

// 1. Offset Coordinate Function
// input: coordinate argument, output: new coordinate
// from the given coordinates for the images, calaculate the new target point for the robot to drive to that is offset from the image in the direction normal to the coordinate. Offset amount to be made as a global parameter so we can change later

// 2. Calculate Distances Between Coordinates Function
// input: 2 coordinate arguments, output: scalar distance between the two
// from a set of two input coordinate arguments, output the distance between the two points

// 3. Calculate Best Route To Take Function
// input: robot starting coordinate, all 5 offset image coordinates, output: array of all coordinates in order of best route, beginning with robot starting position and ending with robot starting position

// 4. Drive To Coordinate Function - they have done for us
// input: coordinate argument, output: robot drives to the given coordinate
// from the given coordinate argument, drive the robot to that coordinate

// 5. Take photo grabs at slightly varrying angles
// Take a photo, check if quality above threshold
// If not take a photo after slightly turning left and another photo after slightly turning right
// Check if three measurements are above a lower threshold
// If not above any move on to next one
// If above store image tag with object location

// 6. Hail mary function
// If some image tags not stored return to any remaining object with no image tag and try function 5 again

// FORMAT FOR COORDINATES: [x, y, p]

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

#define OFFSET 0.5 //meters


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}

// // Calculate the best route to take
// float calculateBestRoute(float robotCoordinate[5], float offsetImageCoordinates[5][3]) {
    
//     float bestRoute[6][3];
//     float tempRoute[6][3];
    
//     shortest_distance = 0
//     for (int a = 0; a<5; a++){
//         for (int b = 0; b<5; b++){
//             for (int c = 0; c<5; c++){
//                 for (int d = 0; d<5; d++){
//                     for (int e = 0; e<5; e++){
//                         if (a != b && a != c && a != d && a != e && b != c && b != d && b != e && c != d && c != e && d != e){
//                             tempRoute[0] = robotCoordinate;
//                             tempRoute[1] = offsetImageCoordinates[a];
//                             tempRoute[2] = offsetImageCoordinates[b];
//                             tempRoute[3] = offsetImageCoordinates[c];
//                             tempRoute[4] = offsetImageCoordinates[d];
//                             tempRoute[5] = offsetImageCoordinates[e];
//                             tempRoute[6] = robotCoordinate;
//                             calculateRouteLength(tempRoute);
//                             if (shortest_distance == 0){
//                                 shortest_distance = calculateRouteLength(tempRoute);
//                                 bestRoute = tempRoute;
//                             }
//                             else if (calculateRouteLength(tempRoute) < shortest_distance){
//                                 shortest_distance = calculateRouteLength(tempRoute);
//                                 bestRoute = tempRoute;
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     return bestRoute;
// }

// Calculate the best route to take
float calculateBestRoute(float robotCoordinate[5], float offsetImageCoordinates[5][3]) {
    
    float bestRoute[6][3];
    float tempRoute[6][3];
    
    shortest_distance = 0
    for (int a = 0; a<5; a++){
        for (int b = 1; b<5; b++){
            for (int c = 2; c<5; c++){
                for (int d = 3; d<5; d++){
                    for (int e = 4; e<5; e++){
                        tempRoute[0] = robotCoordinate;
                        tempRoute[1] = offsetImageCoordinates[a];
                        tempRoute[2] = offsetImageCoordinates[b];
                        tempRoute[3] = offsetImageCoordinates[c];
                        tempRoute[4] = offsetImageCoordinates[d];
                        tempRoute[5] = offsetImageCoordinates[e];
                        tempRoute[6] = robotCoordinate;                            
                        calculateRouteLength(tempRoute);
                        if (shortest_distance == 0){
                            shortest_distance = calculateRouteLength(tempRoute);
                            bestRoute = tempRoute;
                        }
                        else if (calculateRouteLength(tempRoute) < shortest_distance){
                            shortest_distance = calculateRouteLength(tempRoute);
                            bestRoute = tempRoute;
                        }
                    
                    }
                }
            }
        }
    }

    return bestRoute;
}



float calculateRouteLength(float route[6][3]) {
    float length = 0;
    for(int i = 0; i < 5; i++) {
        length += calculateDistance(route[i], route[i+1]);
    }
    return length;
}

float offsetCoordinate(float coordinate[3]) {
    float newCoordinate[3];
    nnewCorrdinate[0] = coordinate[0] +OFFSET*cos(coordinate[2]);
    newCoordinate[1] = coordinate[1] +OFFSET*sin(coordinate[2]);
    newCoordinate[2] = coordinate[2];
    return newCoordinate;
}

float calculateDistance(float coordinate[3]) {
    return sqrt(pow(coordinate[0], 2) + pow(coordinate[1], 2));
}
