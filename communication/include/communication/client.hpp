#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/serialization.h>

#include <vk_costmap_2d/AgvInfo.h>
#include <vk_costmap_2d/AgvInfoArray.h>

#include <thread>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

#define SERVER_PORT 5555
#define SERVER_IP "10.1.8.235"
constexpr unsigned int MAX_SOCKET_CLIENTS = 100;

class AGVClient {
    public:
        AGVClient(tf2_ros::Buffer& tf);

        void startReceivingData() {
            std::thread receive_thread(std::bind(&AGVClient::dataCallback, this));
            receive_thread.detach();
        }

    private:
        ros::NodeHandle nh_, private_nh_;
        ros::Publisher agv_pub_;

        vk_costmap_2d::AgvInfo agv_;
        tf2_ros::Buffer& tf_;
        std::string base_frame_, global_frame_;

        int server_port_;
        std::string server_ip_;
        boost::asio::io_context io_context_;
        boost::asio::ip::tcp::socket socket_;

        bool getRobotPose();
        std::vector<uint8_t> convertROSmsgToByte();
        void convertByteToROSmsg(const std::vector<uint8_t>& buffer, vk_costmap_2d::AgvInfoArray& msg);
        bool sendMessages(const std::vector<uint8_t>& message);
        bool connectToServer();
        void dataCallback();
};