#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <unordered_map>

/**
 * Broadcaster of ground truth pose of robots to bypass the need of robot localization
 */
class GroundTruthBroadcaster {
public:
    GroundTruthBroadcaster(ros::NodeHandle &nh): nodeH(nh)
    {
        robot_pose_subscriber = nh.subscribe("/gazebo/model_states", 10, &GroundTruthBroadcaster::estimate_robot_pose, this);
        nh.param<std::string>("world_frame", worldFrame, "/map");
        nh.param<std::string>("odometry_topic", topic, "ground_truth/pose");
        ros::spin();
    }


private:
    /**
     * Process gazebo position topics
     * @param msg
     */
    void estimate_robot_pose(const gazebo_msgs::ModelStates::ConstPtr &msg) {
        int index = 0;
        ros::Time current_time = ros::Time::now();
        for(auto name : msg->name){
            geometry_msgs::Pose pose = msg->pose[index];
            geometry_msgs::Twist twist = msg->twist[index];
            index++;

            //publish the transform over tf
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
            tf::Quaternion quat;
            tf::quaternionMsgToTF(pose.orientation, quat);
            transform.setRotation(quat);

            br.sendTransform(tf::StampedTransform(transform, current_time, worldFrame, name+"/"+topic));

            // publish pose topic
            if(publishers.find(name) == publishers.end()){
                publishers[name] = nodeH.advertise<geometry_msgs::Pose>(name+"/"+topic, 20);
            }
            publishers[name].publish(pose);
        }
    }

    ros::NodeHandle nodeH;

    /** Name of the world frame */
    std::string worldFrame;
    /** Name of the final base topic */
    std::string topic;

    /** Subscriber of gazebo model states */
    ros::Subscriber robot_pose_subscriber;

    /** Publishers of ground truth states */
    std::unordered_map<std::string, ros::Publisher> publishers;

    tf::TransformBroadcaster br;
};


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pose_broadcaster");
    ros::NodeHandle nh;
    GroundTruthBroadcaster broadcaster(nh);

    return 0;
}

