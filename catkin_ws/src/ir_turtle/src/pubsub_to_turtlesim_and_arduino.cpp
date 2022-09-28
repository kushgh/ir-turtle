/*
Author : kushal.ghosh@jabotics.com

THIS PROGRAM PUBLISHES TO cmd_vel, CAUSING A VIRTUAL TURTLE TO MOVE RANDOMLY IN ANY DIRECTION ,
IT STOPS THE TURTLE (VIRTUALLY ON SCREEN) WHENEVER THE IR SENSOR (PHYSICALLY CONNECTED TO ARDUINO -> PC -> ROS)
ENCOUNTERS AN OBSTACLE PHYSICALLY

PUBLISHING RANDOM TWISTS TO turtle1/cmd_vel
SUBSCRIBING TO IR SENSOR CONNECTED TO ARDUINO USING rosserial, TOPIC chatter

ARDUINO CODE IS PRESENT IN : ir_publisher.ino

TESTED ON :
ROS NOETIC
UBUNTU 20.04
ARDUINO 1.8

RUN THIS PROJECT:

1) Connect IR to arduino UNO and UNO to System. 

$ll /dev/ttyACM0
$sudo chmod a+rw /dev/ttyACM0

IR Pin number used in this case is 4.
(_/) Copy the code in ir_publisher.ino & Feed the code in ir_publisher.ino to Arduino. 
(_/) Install rosserial and make necessary configurations - 
     Just edit the msg.h file located Arduino/libraries/Rosserial_Arduino_Library/src/ros in line 40 
     write #include <string.h> and in line 68 and 182 remove std:: before memcpy
     Then restart Arudino


2) Create a catkin_ws
$cd catkin_ws/src
$catkin_create_pkg ir_turtle geometry_msgs turtlesim rospy roscpp
$cd ir_turtle/src
$code pubsub_to_turtlesim_and_arduino.cpp

3) Paste the code in this cpp file inside of pubsub_to_turtlesim_and_arduino.cpp

4) Add these two lines at the last of CMakeLists.txt

||add_executable(pub_turtle_sub_arduino src/pubsub_to_turtlesim_and_arduino.cpp)
||target_link_libraries(pub_turtle_sub_arduino ${catkin_LIBRARIES})

5) Build Package
$cd ~/catkin_ws
$catkin_make


6) Finally,

$roscore
$rosrun turtlesim turtlesim_node
$rosrun rosserial_python serial_node.py /dev/ttyACM0
$rosrun ir_turtle pub_turtle_sub_arduino

Now you can Play with the IR.

*/

#include <ros/ros.h>                // FOR OBVIOUS REASONS
#include <geometry_msgs/Twist.h>    // FOR Twist MESSAGE
#include <std_msgs/Int32.h>         // FOR Int32 MESSAGE
#include <stdlib.h>                 // FOR GENERATING RANDOM NUMBERS

// A class is efficient when you want to perform Subscribing and Publishing both
class SubscribeAndPublish
{
  
  // linear and angular increment
  double x=0.01, z=0.01;

public:
  // Constructor gets called and pub + sub is initialized
  SubscribeAndPublish()
  {
    //Publishing to Turtlesim's topic cmd_vel
    pub_ = n_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    //Chatter Topic's Subscriber
    sub_ = n_.subscribe("chatter", 1000, &SubscribeAndPublish::callback, this);
    
  }

  void callback(const std_msgs::Int32::ConstPtr& ir_input)
  {
    // When something is in proximity of IR - > Stops Turtle
    if (ir_input->data == 1){
      //Give 0 vel output
        vel_output_.linear.x = 0;
        vel_output_.angular.z = 0;
        ROS_INFO("Obstacle Detected, Stopping.");
    }
    // When nothing is in proximity of IR - > Turtle Crawls
    else{  
      //Give random vel output
        // vel_output_.linear.x = double(rand())/double(RAND_MAX);
        // vel_output_.angular.z = 2*double(rand())/double(RAND_MAX)-1;
        vel_output_.linear.x += x;
        vel_output_.angular.z += z;
        ROS_INFO("Spiraling...");
    }

    //Publishing vel output and logging the output
    pub_.publish(vel_output_);
    ROS_INFO_STREAM("Sending commands\n  linear x: "<<vel_output_.linear.x
        <<"  angular z:  "<<vel_output_.angular.z);
  }

//All the required variables waiting to be initiated
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  geometry_msgs::Twist vel_output_;

};//End of class SubscribeAndPublish


// Driver Code
int main(int argc, char** argv){

    // Initializing ROS
    ros::init(argc, argv, "sub_ir_pub_turtle");
    //Seeding Random Number Generator
    srand(time(0));     
    
    //Calling constructor by creating object
    SubscribeAndPublish SAPObject;

    //Loop at 10Hz until node shut down
    ros::Rate rate(10);

    //Wait for callbacks to happen
    ros::spin();
}