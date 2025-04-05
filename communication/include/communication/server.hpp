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
#include <unordered_map>
#include <queue>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

using boost::asio::ip::tcp;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

/* #define RECEIVED_DATA */
#define SERVER_PORT 5555

constexpr size_t MAX_QUEUE_SIZE = 10;       
constexpr unsigned int SEND_INTERVAL = 100; // unit: ms

class AGVServer {
    public:
        AGVServer(boost::asio::io_context& io_context, int port);

        void startAccept();
        void handleAccept(const boost::system::error_code& error, std::shared_ptr<tcp::socket> socket);
        void receiveData(std::shared_ptr<tcp::socket> socket);
        void handleReceive(const boost::system::error_code& error,
                           size_t bytes_transferred,
                           std::shared_ptr<tcp::socket> socket,
                           std::shared_ptr<std::vector<uint8_t>> buffer);

        void sendMessages();

    private:
        boost::asio::io_context& io_context_;
        tcp::acceptor acceptor_;
        std::vector<std::shared_ptr<tcp::socket>> clients_;
        boost::asio::steady_timer timer_;
        std::unordered_map<std::shared_ptr<tcp::socket>, std::queue<std::vector<uint8_t>>> client_data_;
        std::mutex client_data_mutex_;
};