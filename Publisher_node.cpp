##############

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudPublisher : public rclcpp::Node {
public:
    PointCloudPublisher() : Node("pointcloud_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PointCloudPublisher::publish_pointcloud, this));
    }

private:
    void publish_pointcloud() {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map";

        cloud_msg.height = 1;
        cloud_msg.width = points_.size();
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& p : points_) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            ++iter_x; ++iter_y; ++iter_z;
        }

        publisher_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<PositionComponent> points_ = { {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0} };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}


###################

##### subscriber Node ######

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudSubscriber : public rclcpp::Node {
public:
    PointCloudSubscriber() : Node("pointcloud_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&PointCloudSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        while (iter_x != iter_x.end()) {
            RCLCPP_INFO(this->get_logger(), "Point: [%f, %f, %f]", *iter_x, *iter_y, *iter_z);
            ++iter_x; ++iter_y; ++iter_z;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}

########## ECS system for Pointcloud Processing ##############
class PointCloudSystem {
    public:
        void update(PointCloudComponent& pointCloud) {
            for (auto& point : pointCloud.points) {
                point.x += 0.1;  // Example processing
                point.y += 0.1;
                point.z += 0.1;
            }
        }
    };


#############################################


#### Integration #############


#########Publisher: Converts ECS PointCloudComponent → sensor_msgs::msg::PointCloud2#######
########Subscriber: Converts sensor_msgs::msg::PointCloud2 → ECS PointCloudComponent#######


