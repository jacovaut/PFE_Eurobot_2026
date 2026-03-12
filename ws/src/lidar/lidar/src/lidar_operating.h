#include "CYdLidar.h"
#include <iostream>

int main()
{
    CYdLidar laser;

    std::string port = "/dev/ttyUSB0";
    int baudrate = 230400;

    laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

    int lidar_type = TYPE_TRIANGLE;
    laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

    bool fixed_resolution = true;
    laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));

    bool reversion = true;
    laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

    bool inverted = true;
    laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

    float max_range = 16.0;
    float min_range = 0.01;

    laser.setlidaropt(LidarPropMaxRange, &max_range, sizeof(float));
    laser.setlidaropt(LidarPropMinRange, &min_range, sizeof(float));

    if (!laser.initialize())
    {
        std::cout << "Failed to initialize LiDAR\n";
        return -1;
    }

    if (!laser.turnOn())
    {
        std::cout << "Failed to start LiDAR\n";
        return -1;
    }

    LaserScan scan;

    while (true)
    {
        if (laser.doProcessSimple(scan))
        {
            std::cout << "Scan received: " << scan.points.size() << " points\n";

            for (auto &p : scan.points)
            {
                float x = p.range * cos(p.angle);
                float y = p.range * sin(p.angle);

                std::cout << "x: " << x << " y: " << y << std::endl;
            }
        }
    }

    laser.turnOff();
    laser.disconnecting();

    return 0;
}