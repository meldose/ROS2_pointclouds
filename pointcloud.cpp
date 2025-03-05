#include <entt/entt.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Define a component for storing a point cloud
struct PointCloudComponent {
    sensor_msgs::msg::PointCloud2 cloud;
};

// Define a component for transformations
struct TransformComponent {
    float x, y, z;   // Position
    float roll, pitch, yaw;  // Orientation
};


############################

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <entt/entt.hpp>

class PointCloudProcessorSystem {
public:
    void process(entt::registry &registry) {
        auto view = registry.view<PointCloudComponent>();
        for (auto entity : view) {
            auto &pc = view.get<PointCloudComponent>(entity);
            RCLCPP_INFO(rclcpp::get_logger("PointCloudProcessor"), 
                        "Processing a PointCloud with %lu points", pc.cloud.data.size());
        }
    }
};

#############################
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <entt/entt.hpp>

class PointCloudSubscriber : public rclcpp::Node {
public:
    PointCloudSubscriber() : Node("pointcloud_ecs_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_topic", 10, 
            std::bind(&PointCloudSubscriber::callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    entt::registry registry_;  // ECS registry

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto entity = registry_.create();
        registry_.emplace<PointCloudComponent>(entity, *msg);
        RCLCPP_INFO(this->get_logger(), "New PointCloud entity created with %lu points", msg->data.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#############################


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSubscriber>();

    entt::registry registry;
    PointCloudProcessorSystem processor;

    rclcpp::Rate rate(10);  // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        processor.process(registry);  // Process entities with PointCloudComponent
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


####################################


