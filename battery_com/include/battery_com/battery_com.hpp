#include <ros/ros.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <vector>
#include <string>

using namespace mn::CppLinuxSerial;
using namespace std;

#define DEFAULT_BAURATE BaudRate::B_9600
#define DEFAULT_WAIT_MS 10
#define DEFAULT_DEVICE "/dev/ttyUSB0"

#define ID_START 0xDD
#define ID_STATE_READ 0xA5
#define ID_STATE_WRITE 0x5A
#define ID_STOP 0x77

#define PID_READ_INF_STT 0x03           /* read basic information and status */
#define PID_READ_BAT_CELL_VOL 0x04      /* read battery cell voltage */
#define PID_READ_HW_VER 0x05            /* read the hardware version number of the protection board */

#define NumDataBytes__ 34               /* number of bytes read from PID_READ_INF_STT */

uint16_t Byte2Int(uint8_t byHigh, uint8_t byLow) {
    return ((uint16_t)byHigh << 8 | byLow);
}

int CheckSum(std::vector<uint8_t>& data) {
    uint8_t csum = 0;
    uint16_t i;
    for(i = 0; i < data.size(); i++) {
        csum += data[i];
    }
    if(csum == 0) return 0;
    else return 1;
}

class BATTERY {
    private:
        std::unique_ptr<SerialPort> port;   /* serial port */
        uint16_t voltage;                   /* unit 10mV */
        int16_t current;                    /* unit 10mA, if negative, It's discharge */
        uint16_t remain_capacity;           /* unit is 10mAH */
        uint16_t nominal_capacity;
        std::vector<uint8_t> sendData;      /* package sent */
        std::vector<uint8_t> readData;      /* package response */

        int first_byte = 4;
        bool __success;

    public:
        void ReadInfoBattery() {
            sendData.clear();
            sendData.push_back(ID_START);
            sendData.push_back(ID_STATE_READ);
            sendData.push_back(PID_READ_INF_STT);
            sendData.push_back(0x00);
            sendData.push_back(0xFF);
            sendData.push_back(0xFD);
            sendData.push_back(ID_STOP);
            port->WriteBinary(sendData);

            #ifdef DEBUG
            cout << "Host sends: ";
            for(int i = 0; i < sendData.size(); i++) {
                cout << hex << int(sendData[i]) << " ";
            }
            cout << endl;
            ROS_INFO("Reading data ...");
            #endif

            std::vector<uint8_t> buffer;
            readData.clear();
            do{
                port->ReadBinary(buffer);
                for(int i = 0; i < buffer.size(); i++) {
                    readData.push_back(buffer[i]);
                }
            }while(CheckSum(buffer));
            if(readData.size() == NumDataBytes__) {
                #ifdef DEBUG
                ROS_INFO("Reading data successful!");
                cout << "BMS response: ";
                for(int i = 0; i < readData.size(); i++) {
                    cout << hex << int(readData[i]) << " ";
                }
                #endif

                voltage = Byte2Int(readData[first_byte], readData[first_byte+1]);

                current = Byte2Int(readData[first_byte+2], readData[first_byte+3]);

                remain_capacity = Byte2Int(readData[first_byte+4], readData[first_byte+5]);

                nominal_capacity = Byte2Int(readData[first_byte+6], readData[first_byte+7]);

                std::cout << dec << "    Voltage                 : " << voltage / 100.0 << "V" << "\n" \
                                 << "    Current                 : " << current / 100.0 << "A" << "\n" \
                                 << "    Remain capacity         : " << remain_capacity / 100.0 << "AH" << "\n" \
                                 << "    Nominal capacity        : " << nominal_capacity / 100.0 << "AH" << "\n" \
                                 << "    Battery capacity/Voltage: " << remain_capacity*10 / voltage << "%" << endl;
                
                if(current > 0) {
                    ROS_INFO("Battery is charging!");
                }else if(remain_capacity*10 / voltage < 25) {
                    ROS_WARN("Low battery!");
                }
            }else {
                ROS_WARN("Received data failed, please try again!");
            }
        }

        BATTERY(ros::NodeHandle* nh) {
            voltage = 0;
            current = 0;
            remain_capacity = 0;
            nominal_capacity = 0;

            port = std::make_unique<SerialPort>(DEFAULT_DEVICE, DEFAULT_BAURATE, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
            port->SetTimeout(100);
            port->Open();

            ros::Rate rate(1.0);
            while(ros::ok()) {
                ReadInfoBattery();
                rate.sleep();
            }
        }
        
        ~BATTERY() {
            port->Close();
        }
};