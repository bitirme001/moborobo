#include <ros/ros.h>
#include <serial/serial.h>

#include <sstream>
#include <vector>

#include "Constants.h"
#include "ErrorCodes.h"
#include "RoboteqDevice.h"

using namespace std;

class Driver
{
public:
  Driver(const string &preferred_port = "") : preferred_port_(preferred_port)
  {
    if (Connect() == RQ_SUCCESS)
      Arm();
  }
  ~Driver() { Disconnect(); }

  int Connect()
  {
    vector<string> candidate_ports;
    auto add_candidate = [&candidate_ports](const string &port) {
      for (const string &existing_port : candidate_ports)
      {
        if (existing_port == port)
          return;
      }
      candidate_ports.push_back(port);
    };

    if (!preferred_port_.empty())
      add_candidate(preferred_port_);

    add_candidate("/dev/ttyACM0");
    add_candidate("/dev/ttyUSB0");

    for (const string &port : candidate_ports)
    {
      int status = device.Connect(port);
      if (status == RQ_SUCCESS)
      {
        connected_port_ = port;
        ROS_INFO_STREAM("Connection with motor driver has been established on " << connected_port_);
        return status;
      }

      ROS_WARN_STREAM("Failed to connect with motor driver on " << port);
    }

    ROS_ERROR("Motor driver could not be opened on /dev/ttyACM0 or /dev/ttyUSB0");
    return RQ_ERR_OPEN_PORT;
  }
  void Disconnect() { device.Disconnect(); }
  bool IsConnected() { return device.IsConnected(); }
  int Arm()
  {
    int status = 0;
    if ((status = device.SetCommand(_MG, 1)) == RQ_SUCCESS)
      return 0;
    else
      return 1;
  }
  int TurnWheel(int canIndex, int motorIndex, float value)
  {
    int status = 0;
    if ((status = device.SetCanCommand(canIndex, _G, motorIndex, value)) !=
        RQ_SUCCESS)
      return 1;
    else
      return 0;
  }
  int TurnWheelRPM(int motorIndex, int rpm)
  {
    return device.SetCommand(_S, motorIndex, rpm);
  }
  int GetMotorRPM(int motorIndex, int &rpm)
  {
    return device.GetValue(_BS, motorIndex, rpm);
  }
  float GetMotorCurrent(int motorIndex, int &motorCurrent)
  {
    return device.GetValue(_A, motorIndex, motorCurrent);
  }
  bool GetButtonStatus(int button_pin_id) // Diğital pin çıkışları boolean,
                                          // dolayısıyla burası değişecek
  {
    int status = 0, value = 0;
    if ((status = device.GetValue(_DIN, button_pin_id, value)) != RQ_SUCCESS)
      return false;

    else
    {
      if (value == 0)
        return false;
      else
        return true;
    }
  }
  float GetControllerChannel(
      int controller_channel_id) // Diğital pin çıkışları boolean, dolayısıyla
                                 // burası değişecek
  {
    int status = 0, value = 0;
    if ((status = device.GetCanValue(1, _AI, controller_channel_id, value)) !=
        RQ_SUCCESS)
      return -111111;
    else
      return value;
  }

  int GetBatteryVoltage(int voltageIndex, int &voltage)
  {
    return device.GetValue(_V, voltageIndex, voltage);
  }


private:
  string preferred_port_;
  string connected_port_;
  RoboteqDevice device;
  void Wait() { sleepms(10); }
};
