#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vdb_mapping_interfaces/srv/batch_raytrace.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <thread>

class DroneRaytracerNode : public rclcpp::Node
{
public:
    DroneRaytracerNode() : Node("drone_raytracer_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        raytrace_client_ = this->create_client<vdb_mapping_interfaces::srv::BatchRaytrace>(
            "/vdb_mapping/batch_raytrace");
        
        ray_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/ray_visualization", 10);
        hit_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/hit_points", 10);
        hit_points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/hit_points_cloud", 10);
        rays_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/rays_cloud", 10);
        
        this->declare_parameter<std::string>("drone_frame", "px4_body");
        this->declare_parameter<std::string>("world_frame", "local_origin");
        this->declare_parameter<double>("ray_length", 10.0);
        this->declare_parameter<double>("raytracing_rate", 5.0);
        this->declare_parameter<int>("num_rays_horizontal", 16);
        this->declare_parameter<int>("num_rays_vertical", 8);
        this->declare_parameter<double>("horizontal_fov", 2 * M_PI);
        this->declare_parameter<double>("vertical_fov", M_PI);
        
        this->get_parameter("drone_frame", drone_frame_);
        this->get_parameter("world_frame", world_frame_);
        this->get_parameter("ray_length", ray_length_);
        this->get_parameter("num_rays_horizontal", num_rays_horizontal_);
        this->get_parameter("num_rays_vertical", num_rays_vertical_);
        this->get_parameter("horizontal_fov", horizontal_fov_);
        this->get_parameter("vertical_fov", vertical_fov_);
        
        double raytracing_rate;
        this->get_parameter("raytracing_rate", raytracing_rate);
        
        raytrace_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / raytracing_rate)),
            std::bind(&DroneRaytracerNode::raytraceCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Drone Raytracer Node initialized");
        RCLCPP_INFO(this->get_logger(), "Drone frame: %s", drone_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "World frame: %s", world_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Ray length: %.2f m", ray_length_);
        RCLCPP_INFO(this->get_logger(), "Rays: %dx%d", num_rays_horizontal_, num_rays_vertical_);
    }

private:
    void raytraceCallback()
    {
        if (!raytrace_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Raytracing service not available");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                world_frame_, drone_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not transform %s to %s: %s", 
                drone_frame_.c_str(), world_frame_.c_str(), ex.what());
            return;
        }

        auto request = std::make_shared<vdb_mapping_interfaces::srv::BatchRaytrace::Request>();
        request->header.frame_id = world_frame_;
        request->header.stamp = this->get_clock()->now();
        
        generateRays(transform, request->rays);
        
        auto result = raytrace_client_->async_send_request(request);
        
        auto shared_future = result.future.share();
        
        std::thread([this, shared_future, transform]() {
            try {
                auto response = shared_future.get();
                this->processRaytraceResponse(response, transform);
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(this->get_logger(), "Raytracing service call failed: %s", ex.what());
            }
        }).detach();
    }
    
    void generateRays(const geometry_msgs::msg::TransformStamped& transform, 
                      std::vector<vdb_mapping_interfaces::msg::Ray>& rays)
    {
        rays.clear();
        
        Eigen::Vector3d drone_position(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        
        Eigen::Quaterniond drone_orientation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );
        
        for (int h = 0; h < num_rays_horizontal_; ++h) {
            for (int v = 0; v < num_rays_vertical_; ++v) {
                double azimuth = (static_cast<double>(h) / num_rays_horizontal_) * horizontal_fov_ - horizontal_fov_/2.0;
                double elevation = (static_cast<double>(v) / num_rays_vertical_) * vertical_fov_ - vertical_fov_/2.0;
                
                Eigen::Vector3d ray_direction_local(
                    cos(elevation) * cos(azimuth),
                    cos(elevation) * sin(azimuth),
                    sin(elevation)
                );
                
                Eigen::Vector3d ray_direction_world = drone_orientation * ray_direction_local;
                
                vdb_mapping_interfaces::msg::Ray ray;
                ray.origin.x = drone_position.x();
                ray.origin.y = drone_position.y();
                ray.origin.z = drone_position.z();
                
                ray.direction.x = ray_direction_world.x();
                ray.direction.y = ray_direction_world.y();
                ray.direction.z = ray_direction_world.z();
                
                ray.max_ray_length = ray_length_;
                
                rays.push_back(ray);
            }
        }
    }
    
    void processRaytraceResponse(
        std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> response,
        const geometry_msgs::msg::TransformStamped& drone_transform)
    {
        publishRayVisualization(response, drone_transform);
        publishHitPointVisualization(response);
        publishHitPointsCloud(response, drone_transform);
        publishRaysCloud(response, drone_transform);
        
        int successful_hits = 0;
        for (const auto& success : response->successes) {
            if (success) successful_hits++;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Raytracing completed: %d/%zu rays hit obstacles", 
                    successful_hits, response->successes.size());
    }
    
    void publishRayVisualization(
        std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> response,
        const geometry_msgs::msg::TransformStamped& drone_transform)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        Eigen::Vector3d drone_position(
            drone_transform.transform.translation.x,
            drone_transform.transform.translation.y,
            drone_transform.transform.translation.z
        );
        
        for (size_t i = 0; i < response->end_points.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "rays";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.points.resize(2);
            marker.points[0].x = drone_position.x();
            marker.points[0].y = drone_position.y();
            marker.points[0].z = drone_position.z();
            
            if (response->successes[i]) {
                marker.points[1].x = response->end_points[i].x;
                marker.points[1].y = response->end_points[i].y;
                marker.points[1].z = response->end_points[i].z;
                
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
            } else {
                Eigen::Vector3d ray_direction(
                    response->end_points[i].x - drone_position.x(),
                    response->end_points[i].y - drone_position.y(),
                    response->end_points[i].z - drone_position.z()
                );
                ray_direction.normalize();
                
                marker.points[1].x = drone_position.x() + ray_direction.x() * ray_length_;
                marker.points[1].y = drone_position.y() + ray_direction.y() * ray_length_;
                marker.points[1].z = drone_position.z() + ray_direction.z() * ray_length_;
                
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.3;
            }
            
            marker.scale.x = 0.02;
            marker.scale.y = 0.05;
            marker.scale.z = 0.1;
            
            marker_array.markers.push_back(marker);
        }
        
        ray_markers_pub_->publish(marker_array);
    }
    
    void publishHitPointVisualization(
        std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> response)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        int hit_point_id = 0;
        for (size_t i = 0; i < response->successes.size(); ++i) {
            if (response->successes[i]) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = world_frame_;
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "hit_points";
                marker.id = hit_point_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose.position.x = response->end_points[i].x;
                marker.pose.position.y = response->end_points[i].y;
                marker.pose.position.z = response->end_points[i].z;
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                
                marker_array.markers.push_back(marker);
            }
        }
        
        hit_points_pub_->publish(marker_array);
    }
    
    void publishHitPointsCloud(
        std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> response,
        const geometry_msgs::msg::TransformStamped& drone_transform)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.frame_id = world_frame_;
        cloud_msg.header.stamp = this->get_clock()->now();
        
        Eigen::Vector3d drone_position(
            drone_transform.transform.translation.x,
            drone_transform.transform.translation.y,
            drone_transform.transform.translation.z
        );
        
        std::vector<geometry_msgs::msg::Point> hit_points;
        std::vector<float> distances;
        
        for (size_t i = 0; i < response->successes.size(); ++i) {
            if (response->successes[i]) {
                hit_points.push_back(response->end_points[i]);
                
                double distance = sqrt(
                    pow(response->end_points[i].x - drone_position.x(), 2) +
                    pow(response->end_points[i].y - drone_position.y(), 2) +
                    pow(response->end_points[i].z - drone_position.z(), 2)
                );
                distances.push_back(static_cast<float>(distance));
            }
        }
        
        if (hit_points.empty()) {
            return;
        }
        
        cloud_msg.height = 1;
        cloud_msg.width = hit_points.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
            "danger", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud_msg, "distance");
        sensor_msgs::PointCloud2Iterator<float> iter_danger(cloud_msg, "danger");
        
        for (size_t i = 0; i < hit_points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_distance, ++iter_danger) {
            *iter_x = static_cast<float>(hit_points[i].x);
            *iter_y = static_cast<float>(hit_points[i].y);
            *iter_z = static_cast<float>(hit_points[i].z);
            *iter_distance = distances[i];
            *iter_danger = ray_length_ - distances[i];  // Higher values for closer obstacles
        }
        
        hit_points_cloud_pub_->publish(cloud_msg);
    }
    
    void publishRaysCloud(
        std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> response,
        const geometry_msgs::msg::TransformStamped& drone_transform)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.frame_id = world_frame_;
        cloud_msg.header.stamp = this->get_clock()->now();
        
        Eigen::Vector3d drone_position(
            drone_transform.transform.translation.x,
            drone_transform.transform.translation.y,
            drone_transform.transform.translation.z
        );
        
        std::vector<geometry_msgs::msg::Point> ray_endpoints;
        std::vector<float> distances;
        std::vector<uint8_t> hit_flags;
        
        for (size_t i = 0; i < response->end_points.size(); ++i) {
            ray_endpoints.push_back(response->end_points[i]);
            
            double distance;
            if (response->successes[i]) {
                distance = sqrt(
                    pow(response->end_points[i].x - drone_position.x(), 2) +
                    pow(response->end_points[i].y - drone_position.y(), 2) +
                    pow(response->end_points[i].z - drone_position.z(), 2)
                );
            } else {
                distance = ray_length_;
            }
            distances.push_back(static_cast<float>(distance));
            hit_flags.push_back(response->successes[i] ? 1 : 0);
        }
        
        cloud_msg.height = 1;
        cloud_msg.width = ray_endpoints.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
            "danger", 1, sensor_msgs::msg::PointField::FLOAT32,
            "hit", 1, sensor_msgs::msg::PointField::UINT8);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud_msg, "distance");
        sensor_msgs::PointCloud2Iterator<float> iter_danger(cloud_msg, "danger");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_hit(cloud_msg, "hit");
        
        for (size_t i = 0; i < ray_endpoints.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_distance, ++iter_danger, ++iter_hit) {
            *iter_x = static_cast<float>(ray_endpoints[i].x);
            *iter_y = static_cast<float>(ray_endpoints[i].y);
            *iter_z = static_cast<float>(ray_endpoints[i].z);
            *iter_distance = distances[i];
            *iter_danger = ray_length_ - distances[i];  // Higher values for closer obstacles/rays
            *iter_hit = hit_flags[i];
        }
        
        rays_cloud_pub_->publish(cloud_msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Client<vdb_mapping_interfaces::srv::BatchRaytrace>::SharedPtr raytrace_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ray_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hit_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hit_points_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rays_cloud_pub_;
    
    rclcpp::TimerBase::SharedPtr raytrace_timer_;
    
    std::string drone_frame_;
    std::string world_frame_;
    double ray_length_;
    int num_rays_horizontal_;
    int num_rays_vertical_;
    double horizontal_fov_;
    double vertical_fov_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneRaytracerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}