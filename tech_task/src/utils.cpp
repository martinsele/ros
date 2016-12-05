#include "utils.h"

namespace utils {

    /**
     * Computes Euklidean distance between 2 points in 2D space
     * @param point1 - first point
     * @param point2 - second point
     * @return - Euklidean distance between first and second point
     */
    double compute_euklidean_distance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2){
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
    }

    /**
     * Computes Euklidean distance between 2 points in 2D space
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return Euklidean distance between first and second point
     */
    double compute_euklidean_distance(double x1, double y1, double x2, double y2){
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }

    /**
     * Converts geometry_msgs Quaternion to tf Quaternion
     * @param geometry_quaternion - geometry_msgs representation of quaternion
     * @param tf_quaternion - tf_msgs representation of quaternion
     */
    void convert_to_tf_quaternion(const geometry_msgs::Quaternion &geometry_quaternion, tf::Quaternion &tf_quaternion){
        tf_quaternion.setX(geometry_quaternion.x);
        tf_quaternion.setY(geometry_quaternion.y);
        tf_quaternion.setZ(geometry_quaternion.z);
        tf_quaternion.setW(geometry_quaternion.w);
    }

    /**
     * Fill position of the Pose message and basic rotation
     * @param pose
     * @param x
     * @param y
     * @param z
     */
    void setPosePoint(geometry_msgs::Pose &pose, double x, double y, double z){
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        pose.orientation.x = 1.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
    }

    /**
     * Fill position of the Pose message and basic rotation
     * @param pose
     * @param point
     */
    void setPosePoint(geometry_msgs::Pose &pose, geometry_msgs::Point p){
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;

        pose.orientation.x = 1.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
    }

    std::string planToString(std::vector<geometry_msgs::Point> plan){
        std::string str = "";
        for(auto wpt : plan){
            str += "x:"+ std::to_string(wpt.x)+" y:"+std::to_string(wpt.y)+" z:"+std::to_string(wpt.z)+"\n";
        }
        return str;
    }
}
