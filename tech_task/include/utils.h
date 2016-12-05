#ifndef UAV_TASK_UTILS_H
#define UAV_TASK_UTILS_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf/tf.h>
#include <unordered_map>

namespace utils {

    /**
     * Computes Euklidean distance between 2 points in 2D space
     * @param point1 - first point
     * @param point2 - second point
     * @return - Euklidean distance between first and second point
     */
    double compute_euklidean_distance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);

    /**
     * Computes Euklidean distance between 2 points in 2D space
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return Euklidean distance between first and second point
     */
    double compute_euklidean_distance(double x1, double y1, double x2, double y2);

    /**
     * Converts geometry_msgs Quaternion to tf Quaternion
     * @param geometry_quaternion - geometry_msgs representation of quaternion
     * @param tf_quaternion - tf_msgs representation of quaternion
     */
    void convert_to_tf_quaternion(const geometry_msgs::Quaternion &geometry_quaternion, tf::Quaternion &tf_quaternion);

    /**
     * Fill position of the Pose message and basic rotation
     * @param pose
     * @param x
     * @param y
     * @param z
     */
    void setPosePoint(geometry_msgs::Pose &pose, double x, double y, double z);

    /**
     * Fill position of the Pose message and basic rotation
     * @param pose
     * @param point
     */
    void setPosePoint(geometry_msgs::Pose &pose, geometry_msgs::Point p);

    std::string planToString(std::vector<geometry_msgs::Point> plan);

}
#endif //UAV_TASK_UTILS_H
