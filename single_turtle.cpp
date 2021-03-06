#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <algorithm>
#include <iomanip>

using namespace std;

ros::Publisher pub;
const float PI = 3.14159265;
float rate = 5;
turtlesim::Pose current_pose;
geometry_msgs::Twist msg;
const double tolerance = 1e-2;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub =
        h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

    double x0, y0;

    for (int n = 1; n < argc; n += 2)
    {
        x0 = atof(argv[n]);
        y0 = atof(argv[n+1]);
        bool first_run = true;
        while (ros::ok()) {
            loopRate.sleep();
            ros::spinOnce();

            if (current_pose.theta > PI) current_pose.theta = current_pose.theta - 2*PI;
            if (current_pose.theta < -PI) current_pose.theta = current_pose.theta + 2*PI;

            double distance = sqrt( pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            if (distance < tolerance) {
                pub.publish(getMessage(0,0));
                break;
            }

            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;

            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);

            cout << "x = " << left << setw(10) << current_pose.x << " | y = " << setw(10) << current_pose.y << " | theta = " << setw(10) << current_pose.theta << " | distance = " << setw(10) << distance << endl;

            if (current_pose.x > x0 + 0.5)
            {
                if (current_pose.theta > -PI/2 && current_pose.theta < PI/2)
                    msg = getMessage(
                    -min(6*distance, 8.0),
                    -6*dalpha
                    );
                else msg = getMessage(
                    min(6*distance, 8.0),
                    6*dalpha
                    );
            }
            else if(current_pose.x < x0 - 0.5)
            {
                if (current_pose.theta > PI/2 || current_pose.theta < -PI/2)
                    msg = getMessage(
                    -min(6*distance, 8.0),
                    -6*dalpha
                    );
                else msg = getMessage(
                    min(6*distance, 8.0),
                    6*dalpha
                    );

            }
            else if (current_pose.x <= x0 + 0.5 && current_pose.x >= x0 - 0.5)
            {
                if (current_pose.y >= y0)
                {
                    if (current_pose.theta >= 0)
                        msg = getMessage(
                        -min(6*distance, 8.0),
                        -6*dalpha
                        );
                    else msg = getMessage(
                        min(6*distance, 8.0),
                        6*dalpha
                        );
                }
                else
                {
                    if (current_pose.theta < 0)
                        msg = getMessage(
                        -min(6*distance, 8.0),
                        -6*dalpha
                        );
                    else msg = getMessage(
                        min(6*distance, 8.0),
                        6*dalpha
                        );
                }
            }
            pub.publish(msg);
        }
    }
    return 0;
}
