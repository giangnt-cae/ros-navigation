# CppLinuxSerial - Giao tiếp Serial trên Linux bằng C++

## Giới thiệu

[CppLinuxSerial](https://github.com/gbmhunter/CppLinuxSerial) là một thư viện C++ đơn giản giúp giao tiếp với các thiết bị Serial trên hệ điều hành Linux. Thư viện này là một lựa chọn thay thế nhẹ hơn so với `boost::asio` hoặc `termios` khi làm việc với cổng nối tiếp (UART/Serial).

## Cài đặt

### 1. Clone Repository

```sh
 git clone https://github.com/gbmhunter/CppLinuxSerial.git
 cd CppLinuxSerial
```

### 2. Biên dịch thư viện

```sh
mkdir build
cd build
cmake ..
make
sudo make install
```

Thư viện sẽ được cài vào `/usr/local/lib` và tệp header sẽ có trong `/usr/local/include`.

## Sử dụng

### 1. Thêm vào dự án của bạn

Sau khi cài đặt, bạn có thể sử dụng thư viện bằng cách include header trong file C++:

```cpp
#include "CppLinuxSerial/SerialPort.hpp"
using namespace mn::CppLinuxSerial;
```

### 2. Ví dụ sử dụng

Dưới đây là một chương trình C++ đơn giản để mở cổng serial, gửi dữ liệu và nhận phản hồi.

```cpp
#include <iostream>
#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

int main() {
    // Khởi tạo đối tượng SerialPort với thiết bị /dev/ttyUSB0
    SerialPort serial("/dev/ttyUSB0", BaudRate::B_9600);
    
    // Mở cổng Serial
    serial.Open();
    
    // Gửi dữ liệu
    serial.Write("Hello Serial!");
    
    // Nhận dữ liệu phản hồi
    std::string response;
    serial.Read(response);
    
    // Hiển thị dữ liệu nhận được
    std::cout << "Received: " << response << std::endl;
    
    // Đóng cổng Serial
    serial.Close();
    return 0;
}
```

### 3. Biên dịch chương trình

Lưu chương trình vào tệp `main.cpp` và sử dụng lệnh sau để biên dịch:

```sh
g++ main.cpp -o serial_test -lCppLinuxSerial
```

Sau đó, chạy chương trình:

```sh
./serial_test
```

## Cấu hình nâng cao

CppLinuxSerial hỗ trợ các cấu hình nâng cao như:

- Thay đổi Baud rate, Parity, Stop bits
- Chế độ blocking và non-blocking
- Điều khiển DTR/RTS

Ví dụ thay đổi Baud rate:

```cpp
serial.SetBaudRate(BaudRate::B_115200);
```

## Tài liệu tham khảo

- Repository chính thức: [CppLinuxSerial](https://github.com/gbmhunter/CppLinuxSerial)
- POSIX Serial Programming

---

📌 Nếu bạn gặp lỗi hoặc có câu hỏi, hãy mở issue trên GitHub hoặc thảo luận tại phần bình luận!