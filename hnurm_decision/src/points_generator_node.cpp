#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <hnurm_interfaces/msg/zone_end_point2_d.hpp>
#include <hnurm_interfaces/msg/special_area.hpp>
#include <hnurm_interfaces/msg/area.hpp>
#include <hnurm_interfaces/msg/type.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <mutex>

using namespace std::chrono_literals;

class PolygonGenerator : public rclcpp::Node {
public:
    PolygonGenerator(const rclcpp::NodeOptions &options) : Node("polygon_generator"), should_exit_(false) {
        if (!initialized)
        {
            init();
        }
        // subscribe clicked_point
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(points_mutex_);
                hnurm_interfaces::msg::ZoneEndPoint2D point;
                point.x = msg->point.x;
                point.y = msg->point.y;
                points_.push_back(point);
                RCLCPP_INFO(this->get_logger(), "Point %zu: (%.2f, %.2f)", 
                          points_.size(), point.x, point.y);
            });
        filter_process_area_in_odom_ = this->create_subscription<hnurm_interfaces::msg::SpecialArea>(
            "/transformed_special_area",
            rclcpp::SensorDataQoS(),
            std::bind(&PolygonGenerator::spa_sub_callback,this,std::placeholders::_1)
            );
        
        //start keyboard_thread_
        // keyboard_thread_ = std::thread(&PolygonGenerator::keyboard_listener, this);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_special_areas", 10);
        marker_pub_tfed_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_special_areas_transformed", 10);
        // special_areas_pub_ = this->create_publisher<hnurm_interfaces::msg::SpecialArea>("special_areas", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&PolygonGenerator::timer_callback, this));
    }


    ~PolygonGenerator() {
        process_polygon();  //use ENTER to export polygon still bugs,use this for temporary
        // should_exit_ = true;
        // if(keyboard_thread_.joinable()) keyboard_thread_.join();
    }

private:
    std::mutex points_mutex_;
    std::vector<hnurm_interfaces::msg::SpecialArea> spa_s;
    std::vector<std::string> special_areas_names;
    std::map<std::string,hnurm_interfaces::msg::SpecialArea> areas_;

    hnurm_interfaces::msg::SpecialArea getAreas(const std::string &area_name_)
    {
       this->declare_parameter<std::vector<std::string>>(area_name_ + ".polygon_ends", std::vector<std::string>());
       std::string test_type = this->declare_parameter<std::string>(area_name_+".type","both");
       std::string test_area = this->declare_parameter<std::string>(area_name_+".area","red");
        // std::string test_type; 
        // std::string test_area; 
        // this->get_parameter(area_name_+".type",test_type);
        // this->get_parameter(area_name_+".area",test_area);
        
       auto test =  this->get_parameter(area_name_+".polygon_ends").as_string_array();
       //decode points
       hnurm_interfaces::msg::SpecialArea spa_; 
       for(const auto& point : test)
       {
        hnurm_interfaces::msg::ZoneEndPoint2D zep;
        size_t comma_pos = point.find(',');
        zep.x = std::stod(point.substr(0, comma_pos));
        zep.y = std::stod(point.substr(comma_pos + 1));
        spa_.points.push_back(zep);
       }
        if (test_type == "both") {
            spa_.type.data = 2;
        } else if (test_type == "single") {
            spa_.type.data = 1;
        } else {
            spa_.type.data = 0;
        }

        if (test_area == "across") {
            spa_.area.data = 3;
        } else if (test_area == "mid") {
            spa_.area.data = 2;
        } else if (test_area == "red") {
            spa_.area.data = 1;
        }else {
            spa_.area.data = 0;
        }
        
       return spa_;
    }  

    bool initialized = false;
    void init()
    {
        this->declare_parameter<std::vector<std::string>>("special_areas_names", std::vector<std::string>());
        special_areas_names = this->get_parameter("special_areas_names").as_string_array();
        for(const auto& name:special_areas_names)
        {
            areas_[name] = getAreas(name);
        }
        initialized = true;
    }


    // void keyboard_listener() {
    //     std::cout<<"test"<<std::endl;
    //     while(rclcpp::ok() && !should_exit_) {
    //         int c = std::cin.get();
    //         std::cout<<"test out put:"<<c<<std::end;
    //         if(c == '\n') { // press ENTER
    //             process_polygon();
    //         }
    //     }
    // }

    void process_polygon() {
        std::lock_guard<std::mutex> lock(points_mutex_);
        if(points_.size() < 3) {
            RCLCPP_WARN(get_logger(), "need at least 3 points to form polygon");
            return;
        }
        
        // generate polygons
        auto closed_polygon = points_;
        closed_polygon.push_back(points_.front());
        
        // format output
        std::cout << "\ngenerated closed polygon: \n[";

        for(const auto& p : closed_polygon) {
            std::cout << "\"" << std::fixed << std::setprecision(2) 
                    << p.x << "," << p.y << "\",";
        }
        std::cout << "]"<<std::endl;
        
        // clear cache
        points_.clear();
        RCLCPP_INFO(get_logger(), "Polygon exported. Ready for new points...");
    }

    void publish_area_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // delete old marker
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header.frame_id = "map";
        delete_marker.ns = "areas";
        marker_array.markers.push_back(delete_marker);
    
        int id = 0;
        for (const auto& [name, area] : areas_) {
            // polygon marker
            auto polygon_marker = create_polygon_marker(area, id++);
            marker_array.markers.push_back(polygon_marker);
    
            // text marker
            auto text_marker = create_text_marker(area, id++);
            marker_array.markers.push_back(text_marker);
        }
    
        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_polygon_marker(
        const hnurm_interfaces::msg::SpecialArea& area, int id) 
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "areas";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // set marker type 
        switch(area.type.data) {
            case 1: // single
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                break;
            case 2: // both
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                break;
            default: // normal
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
        }
        marker.color.a = 0.7;
        marker.scale.x = 0.1; // line width
    
        // add points
        for(const auto& p : area.points) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.1;
            marker.points.push_back(point);
        }
    
        // closed polygon
        if(!area.points.empty()) {
            geometry_msgs::msg::Point first;
            first.x = area.points.front().x;
            first.y = area.points.front().y;
            marker.points.push_back(first);
        }
    
        return marker;
    }

    visualization_msgs::msg::Marker create_text_marker(
        const hnurm_interfaces::msg::SpecialArea& area, int id) 
    {
        visualization_msgs::msg::Marker marker;
        marker.header = create_polygon_marker(area, 0).header; // 复用header
        marker.ns = "labels";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // calculate center position
        geometry_msgs::msg::Point center;
        for(const auto& p : area.points) {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= area.points.size();
        center.y /= area.points.size();
        center.z = 0.5; // hover height
        
        marker.pose.position = center;
        marker.scale.z = 0.3; // label size
        
        
        // generate text label
        std::stringstream ss;
        std::string area_;
        std::string type_;
        switch (area.area.data)
        {
            case 0:
                area_ = "BLUE";
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;
                break;
            case 1:
                area_ = "RED";
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                break;
            case 2:
                area_ = "MID";
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;
                break;
            
            default:
                area_ = "ACROSS";
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                break;
        }
        switch (area.type.data)
        {
            case 0:
                type_ = "NORMAL";
                break;
            case 1:
                type_ = "SINGLE";
                break;
            default:
                type_ = "BOTH";
                break;
        }
        ss << "Area:"<<area_<<"\n"
           << "Type:"<<type_;
        marker.text = ss.str();
    
        return marker;
    }

    void timer_callback()
    {   
        // for(std::string const &fp:special_areas_names)
        // {
        //     special_areas_pub_->publish(areas_[fp]);
        // }
        publish_area_markers();
    }
    void spa_sub_callback(const hnurm_interfaces::msg::SpecialArea::SharedPtr msg)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = create_polygon_marker(*msg, 0).header; // 复用header
        marker.ns = "polygon_in_basefootprint";

        visualization_msgs::msg::MarkerArray marker_array;
        
        // delete old marker
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header.frame_id = "lidar_link";
        delete_marker.ns = "polygon_in_basefootprint";
        marker_array.markers.push_back(delete_marker);
    
        int id = 0;
        auto polygon_marker = create_polygon_marker(*msg,100);
        polygon_marker.header.frame_id="lidar_link";
        polygon_marker.color.r = 0.5;
        polygon_marker.color.g = 0.5;
        polygon_marker.color.b = 0.5;
        marker_array.markers.push_back(polygon_marker);
        marker_pub_tfed_->publish(marker_array);

    }


    std::vector<hnurm_interfaces::msg::ZoneEndPoint2D> points_;
    std::thread keyboard_thread_;
    std::atomic<bool> should_exit_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Subscription<hnurm_interfaces::msg::SpecialArea>::SharedPtr filter_process_area_in_odom_;   //subscribe pointcloud filter process transformed_spa

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_tfed_;

    // rclcpp::Publisher<hnurm_interfaces::msg::SpecialArea>::SharedPtr special_areas_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PolygonGenerator>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}