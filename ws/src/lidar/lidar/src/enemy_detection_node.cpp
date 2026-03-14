#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>

class LidarEnemyDetector : public rclcpp::Node
{
public:
    LidarEnemyDetector() : Node("lidar_enemy_detector")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarEnemyDetector::scanCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float,float>> points;

        float angle = msg->angle_min;

        for (auto r : msg->ranges)
        {
            if (std::isfinite(r))
            {
                float x = r * cos(angle);
                float y = r * sin(angle);
                points.push_back({x,y});
            }

            angle += msg->angle_increment;
        }

        detectClusters(points);
    }

    void detectClusters(const std::vector<std::pair<float,float>> &points)
    {
        std::vector<std::vector<std::pair<float,float>>> clusters; //Createes the clusters

        const float cluster_dist = 0.15;

        std::vector<std::pair<float,float>> current_cluster;

        for(size_t i=0;i<points.size();i++)
        {
            if(current_cluster.empty())
            {
                current_cluster.push_back(points[i]);
                continue;
            }

            float dx = points[i].first - points[i-1].first;
            float dy = points[i].second - points[i-1].second;
            float dist = sqrt(dx*dx + dy*dy);

            if(dist < cluster_dist)
                current_cluster.push_back(points[i]);
            else
            {
                clusters.push_back(current_cluster);
                current_cluster.clear();
            }
        }

        for(auto &cluster : clusters)
        {
            if(cluster.size() > 5 && cluster.size() < 40)
            {
                float cx=0, cy=0;

                for(auto &p:cluster)
                {
                    cx+=p.first;
                    cy+=p.second;
                }

                cx/=cluster.size();
                cy/=cluster.size();

                RCLCPP_INFO(this->get_logger(),
                    "Enemy detected at %.2f %.2f",cx,cy);
            }
        }
    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarEnemyDetector>());
    rclcpp::shutdown();
}