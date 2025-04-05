#include <communication/client.hpp>

AGVClient::AGVClient(tf2_ros::Buffer& tf)
: tf_(tf), private_nh_("~"), socket_(io_context_)
{   
    agv_pub_ = nh_.advertise<vk_costmap_2d::AgvInfoArray>("agv_info", 10);
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("global_frame", global_frame_, std::string("map"));

    private_nh_.param("ID", agv_.id, std::string("AGV-1709"));
    private_nh_.param("model", agv_.model, std::string(""));
    private_nh_.param("state", agv_.state, std::string(""));
    private_nh_.param("battery_level", agv_.battery_level, float(100.0));
    private_nh_.param("length", agv_.length, float(0.5));
    private_nh_.param("width", agv_.width, float(0.5));
    private_nh_.param("height", agv_.height, float(0.5));

    private_nh_.param("SERVER_PORT", server_port_, int(SERVER_PORT));
    private_nh_.param("SERVER_IP", server_ip_, std::string(SERVER_IP));
    
    if(!connectToServer())
        return;

    startReceivingData();

    ros::Rate rate(10); 
    while (ros::ok()) {   
        if(!getRobotPose()) continue;
        std::vector<uint8_t> message = convertROSmsgToByte();
        if(!sendMessages(message)) {
            socket_.close();
            sleep(1.0);
            if(!connectToServer()) {
                ROS_WARN("Reconnecting failed!");
                return;
            }
        }
        rate.sleep();
    }
}

void AGVClient::dataCallback() {
    ros::Rate rate(10);
    while (ros::ok()) {
        uint32_t num_socket = 0, data_length = 0;
        boost::system::error_code error;
        size_t bytes_read = boost::asio::read(socket_, boost::asio::buffer(&num_socket, sizeof(num_socket)), error);
        if (error) {
            ROS_ERROR("Failed to read data length: %s", error.message().c_str());
            return;
        }
        boost::asio::read(socket_, boost::asio::buffer(&data_length, sizeof(data_length)));

        // Convert from big-endian if necessary
        // data_length = ntohl(data_length);

        if (num_socket == 0) continue;

        if (num_socket > MAX_SOCKET_CLIENTS) {
            ROS_WARN("Exceeded max socket clients: %u", num_socket);
            continue;
        }

        std::vector<uint8_t> buffer(data_length);
        bytes_read = boost::asio::read(socket_, boost::asio::buffer(buffer), error);
        if(!error) {
            vk_costmap_2d::AgvInfoArray msg;
            msg.agvs.resize(num_socket);
            try {
                convertByteToROSmsg(buffer, msg);
                agv_pub_.publish(msg);
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to deserialize AgvInfoArray: %s", e.what());
            }
        } else
            ROS_ERROR("Failed to read data: %s", error.message().c_str());
        
        rate.sleep();
    }
}

bool AGVClient::sendMessages(const std::vector<uint8_t>& message) {
    #ifdef SEND_DATA
    std::ostringstream oss;
    for (size_t i = 0; i < message.size(); ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)(message[i]) << " ";
    }
    ROS_INFO("Sending data: %s", oss.str().c_str());
    #endif
    boost::system::error_code ec;
    boost::asio::write(socket_, boost::asio::buffer(message), ec);
    if(ec) {
        ROS_WARN("Failed to send data: %s", ec.message().c_str());
        return false;
    }
    return true;
}

bool AGVClient::connectToServer() {
    boost::asio::ip::tcp::resolver resolver(io_context_);
    boost::asio::ip::tcp::resolver::query query(server_ip_, std::to_string(server_port_));
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    boost::system::error_code ec;
    boost::asio::connect(socket_, endpoint_iterator, ec);
    if(ec) {
        ROS_WARN("Failed to connect to server: %s", ec.message().c_str());
        return false;
    }
    ROS_INFO("Connection to server successful!");
    return true;
}

std::vector<uint8_t> AGVClient::convertROSmsgToByte() {
    uint32_t serial_size = ros::serialization::serializationLength(agv_);
    std::vector<uint8_t> buffer(serial_size);
    ros::serialization::OStream stream(buffer.data(), serial_size);
    ros::serialization::serialize(stream, agv_);
    return buffer;
}

void AGVClient::convertByteToROSmsg(const std::vector<uint8_t>& buffer, vk_costmap_2d::AgvInfoArray& msg) {
    if (buffer.empty()) {
        ROS_ERROR("Received empty buffer, cannot deserialize AgvInfoArray.");
        return;
    }
    try {
        ros::serialization::IStream stream(const_cast<uint8_t*>(buffer.data()), buffer.size());
        for (uint32_t i = 0; i < msg.agvs.size(); ++i) {
            ros::serialization::deserialize(stream, msg.agvs[i]);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Deserialization failed: %s", e.what());
    }
}

bool AGVClient::getRobotPose() {
    geometry_msgs::PoseStamped global_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);

    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = base_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();
    if(!tf_.canTransform(global_frame_, base_frame_, current_time, ros::Duration(0.1))) {
        ROS_WARN("TF transform not available from %s to %s", base_frame_.c_str(), global_frame_.c_str());
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, base_frame_, current_time);
        tf2::doTransform(robot_pose, global_pose, transform);
    }catch (tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    agv_.current_pose = global_pose.pose;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "agv_client");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    AGVClient agv(buffer);
    return 0;
}




