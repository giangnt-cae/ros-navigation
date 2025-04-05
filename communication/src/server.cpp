#include <communication/server.hpp>

AGVServer::AGVServer(boost::asio::io_context& io_context, int port)
    : io_context_(io_context),
      acceptor_(io_context, tcp::endpoint(tcp::v4(), port)),
      timer_(io_context, boost::asio::chrono::milliseconds(SEND_INTERVAL))
{
    startAccept();
    timer_.async_wait(boost::bind(&AGVServer::sendMessages, this));
}

void AGVServer::startAccept() {
    auto socket = std::make_shared<tcp::socket>(io_context_);
    acceptor_.async_accept(*socket,
        [this, socket](const boost::system::error_code& error) {
            handleAccept(error, socket);
        });
}

void AGVServer::handleAccept(const boost::system::error_code& error, std::shared_ptr<tcp::socket> socket) {
    if (!error) {
        ROS_INFO("Client connected!");
        clients_.push_back(socket);
        receiveData(socket);
    }
    startAccept();
}

void AGVServer::receiveData(std::shared_ptr<tcp::socket> socket) {
    auto buffer = std::make_shared<std::vector<uint8_t>>(1024);
    socket->async_read_some(boost::asio::buffer(*buffer),
        boost::bind(&AGVServer::handleReceive, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred,
            socket,
            buffer));
}

void AGVServer::handleReceive(const boost::system::error_code& error,
                              size_t bytes_transferred,
                              std::shared_ptr<tcp::socket> socket,
                              std::shared_ptr<std::vector<uint8_t>> buffer)
{
    if(!error) {
        {
            std::string client_ip = socket->remote_endpoint().address().to_string();
            std::lock_guard<std::mutex> lock(client_data_mutex_);
            auto& queue = client_data_[socket];
            if (queue.size() == MAX_QUEUE_SIZE) {
                queue.pop();
            }
            queue.push(std::vector<uint8_t>(buffer->begin(), buffer->begin() + bytes_transferred));

            #ifdef RECEIVED_DATA
            std::ostringstream oss;
            for (size_t i = 0; i < bytes_transferred; ++i) {
                oss << std::hex << std::setw(2) << std::setfill('0') << (int)(buffer->at(i)) << " ";
            }
            ROS_INFO("Received " YELLOW "%zu" RESET " bytes from client " GREEN "%s" RESET ": %s",
                     bytes_transferred, client_ip.c_str(), oss.str().c_str());
            #endif
        }
        receiveData(socket);
    }else {
        ROS_WARN("Error receiving data: %s", error.message().c_str());
        // Remove socket out of clients_
        if (error == boost::asio::error::eof || error == boost::asio::error::connection_reset) {
            ROS_WARN("Client disconnected, removing from list.");
            {
                std::lock_guard<std::mutex> lock(client_data_mutex_);
                client_data_.erase(socket);
            }
            clients_.erase(std::remove(clients_.begin(), clients_.end(), socket), clients_.end());
        }
    }
}

void AGVServer::sendMessages() {
    std::vector<uint8_t> combined_data;
    uint32_t num_sockets = 0;
    {
        std::lock_guard<std::mutex> lock(client_data_mutex_);
        for (auto it = client_data_.begin(); it != client_data_.end(); ++it) {
            std::shared_ptr<tcp::socket> socket = it->first;
            std::queue<std::vector<uint8_t>>& queue = it->second;

            if (!queue.empty()) {
                std::vector<uint8_t> data = queue.front();
                queue.pop();
                combined_data.insert(combined_data.end(), data.begin(), data.end());
                num_sockets += 1;
            }
        }
    }
    
    uint32_t data_size = static_cast<uint32_t>(combined_data.size());
    std::vector<uint8_t> buffer;

    // Add the number of sockets to the beginning of the buffer
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&num_sockets), 
                  reinterpret_cast<uint8_t*>(&num_sockets) + sizeof(num_sockets));

    // Add the number of bytes of combined_data to the beginning of the buffer
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&data_size), 
                  reinterpret_cast<uint8_t*>(&data_size) + sizeof(data_size));

    buffer.insert(buffer.end(), combined_data.begin(), combined_data.end());

    auto data_ptr = std::make_shared<std::vector<uint8_t>>(std::move(buffer));
    for (std::shared_ptr<tcp::socket>& socket : clients_) {
        if (socket->is_open()) {
            boost::asio::async_write(*socket, boost::asio::buffer(*data_ptr),
                [socket, data_ptr](const boost::system::error_code& error, std::size_t bytes_transferred) {
                    if (error) {
                        ROS_WARN("Error sending data to %s: %s",
                                 socket->remote_endpoint().address().to_string().c_str(),
                                 error.message().c_str());
                    } else {
                    /*     ROS_INFO("Successfully sent %zu bytes to %s",
                                 bytes_transferred,
                                 socket->remote_endpoint().address().to_string().c_str()); */
                    }
                });
        }
    }
    timer_.expires_after(boost::asio::chrono::milliseconds(SEND_INTERVAL));
    timer_.async_wait(boost::bind(&AGVServer::sendMessages, this));
}


int main(int argc, char** argv) {
    try {
        boost::asio::io_context io_context;
        AGVServer server(io_context, SERVER_PORT);

        std::thread io_thread([&io_context]() {
            io_context.run();
        });

        signal(SIGINT, [](int /*signum*/) {
            ROS_WARN("Shutting down...");
            ros::shutdown();
        });

        ros::init(argc, argv, "agv_server");
        ros::spin();

        io_context.stop();
        io_thread.join();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}