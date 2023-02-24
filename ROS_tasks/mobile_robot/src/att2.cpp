#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include<tf/transform_datatypes.h>
#include<geometry_msgs/Twist.h>
#include<cmath>

sensor_msgs::NavSatFix gpsrn;
sensor_msgs::Imu imurn;

double destlat, destlong, requiredheading;
double angularerror;

ros::Subscriber gpssub, imusub;
ros::Publisher cmd_vel;

geometry_msgs::Twist velmsg;

double calcdistance(double lat1, double lon1, double lat2, double lon2) {
   const double earth_radius = 6371000.0; 
   double d_lat = (lat2 - lat1) * M_PI / 180.0;
   double d_lon = (lon2 - lon1) * M_PI / 180.0;
   double a = pow(sin(d_lat/2), 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * pow(sin(d_lon/2), 2);
   double c = 2 * atan2(sqrt(a), sqrt(1-a));
   return earth_radius * c;
}

void gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
gpsrn = *msg;
}

void imucallback(const sensor_msgs::Imu::ConstPtr& msg) {
imurn = *msg;
}

double calcheading(double headingrn){
    requiredheading = atan2(destlong - gpsrn.longitude, destlat - gpsrn.latitude);

    
    std::cout<<"\n heading is \n"<<headingrn;
    return requiredheading;
}

double calcvelocity(double headingrn){
   /* double error = requiredheading - headingrn;
   if (error > M_PI) {
       error -= 2 * M_PI;
   } else if (error < -M_PI) {
       error += 2 * M_PI;
   }
   return 0.4 * error;*/
    double heading_error = requiredheading - headingrn;
    if (heading_error > M_PI) {
        heading_error -= 2 * M_PI;
    } else if (heading_error < -M_PI) {
        heading_error += 2 * M_PI;
    }

    double drift_error = imurn.angular_velocity.z;
    double gain = 0.4;

    return gain * (heading_error + drift_error);
   

}
int main(int argc, char **argv) {

   ros::init(argc, argv, "att_node");
   ros::NodeHandle n;
gpssub=n.subscribe<sensor_msgs::NavSatFix>("/gps/fix",1,gpscallback);
imusub=n.subscribe<sensor_msgs::Imu>("/imu",1,imucallback);
if (argc != 3) {
       ROS_ERROR("missing or additional args");

       return 1;
}
destlat=atof(argv[1]);
destlong=atof(argv[2]);

ROS_INFO("Destination coordinates: %f, %f",destlat,destlong);
cmd_vel=n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
ros::Rate loop_rate(30);
while(ros::ok()){
    double headingrn = tf::getYaw(imurn.orientation);
    double angularvel=calcvelocity(headingrn);
    velmsg.angular.z= angularvel;
    cmd_vel.publish(velmsg);
double remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);

if (std::abs(headingrn) < 0.5) {
    velmsg.angular.z=0;
    cmd_vel.publish(velmsg);
    remainingdistance = calcdistance(gpsrn.latitude, gpsrn.longitude, destlat, destlong);
    std::cout<<"\n DISTANCE LEFT "<<remainingdistance;
    std::cout<<"\n CURRENT HEADING "<<headingrn;
    if (remainingdistance < 1.0) {
           ROS_INFO("Target GPS coordinates reached");
           velmsg.linear.x=0;
           cmd_vel.publish(velmsg);
           break;
       } else {
           ROS_INFO("going to target heading");
           velmsg.linear.x=0.2;
           cmd_vel.publish(velmsg);
       }
   }

ros::spinOnce();
loop_rate.sleep();
}
return 0;
    
}