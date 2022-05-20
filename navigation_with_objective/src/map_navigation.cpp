#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char choose();

/** declare the coordinates of interest **/
double xh = -1.95;
double yh = -0.44;

double xp1 = 0.09;
double yp1 = -2.24;

double x2 = -0.1;
double y2 = -2.0;

double x3 = -3.45;
double y3 = -2.05;


bool goalReached = false;
 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   ros::spinOnce();
   char choice = 'q';
   do{
      choice =choose();
      if (choice == '0'){
         goalReached = moveToGoal(xh, yh);
      }else if (choice == '1'){
         goalReached = moveToGoal(xp1, yp1);
      }else if (choice == '2'){
         goalReached = moveToGoal(x2, y2);
      }else if (choice == '3'){
         goalReached = moveToGoal(x3, y3);
      }
      if (choice!='q'){
         if (goalReached){
            ROS_INFO("Point reached");
            ros::spinOnce();
           
           // ros::spinOnce();

         }else{
            ROS_INFO("Failed");
            
         }
      }
   }while(choice !='q');
   return 0;
}

/*movo to goal; function*/
bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}

char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|'0': Home "<<std::endl;
	std::cout<<"|'1': Point 1 "<<std::endl;
	std::cout<<"|'2': Point 2 "<<std::endl;
	std::cout<<"|'3': Point 3 "<<std::endl;
	std::cout<<"|'q': Quit "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|Destination?";
	std::cin>>choice;

	return choice;


}
