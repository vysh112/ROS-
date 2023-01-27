#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chatroom/Message.h>

void receiveMessage(const chatroom::Message::ConstPtr& msg)
{
    if((msg->message).empty()!=1){ 

        ROS_INFO(" %s : %s ",msg->username.c_str(),msg->message.c_str());
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "User1");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Subscriber to listen for incoming messages
    ros::Subscriber sub = nh.subscribe("chat_topic", 1000 ,receiveMessage);

    // Publisher to send messages
    ros::Publisher pub = nh.advertise<chatroom::Message>("chat_topic", 1000);
    std::string username;
    std::cout << "Enter Username: ";
    std::getline(std::cin, username);

    while (ros::ok()) {
        std::string input;
        std::cout << "";
        std::getline(std::cin, input);

        // Message to send
        chatroom::Message msg;
        msg.message = input;
        msg.username = username;
        

        // Publish the message
        pub.publish(msg);
        
        
    //    ros::spinOnce();
    }

    return 0;
}