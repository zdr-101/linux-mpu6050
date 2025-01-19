#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "kalman.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

int main(int argc, char *argv[])
{
    short resive_data[6]; // 保存收到的 mpu6050转换结果数据，依次为 AX(x轴加速度), AY, AZ 。GX(x轴角速度), GY ,GZ

    /*打开文件*/
    int fd = open("/dev/I2C1_mpu6050", O_RDWR);
    if (fd < 0)
    {
        return -1;
    }

    // 创建 socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        std::cerr << "Failed to create socket" << std::endl;
        close(fd);
        return -1;
    }

    // 设置目标服务器的 IP 地址和端口号
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(1347);
    if (inet_pton(AF_INET, "192.168.2.108", &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "Invalid IP address" << std::endl;
        close(fd);
        close(sock);
        return -1;
    }

    // 连接到服务器
    if (connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "Connection failed" << std::endl;
        close(fd);
        close(sock);
        return -1;
    }

    while (true)
    {
        /*读取数据*/
        read(fd, resive_data, 12);

        double ax = resive_data[0];
        double ay = resive_data[1];
        double az = resive_data[2];
        double gx = resive_data[3];
        double gy = resive_data[4];
        double gz = resive_data[5];

        // 计算角度
        double roll, pitch, yaw;
        double gyroXrate = gx / 131.0; // 陀螺仪转换为角速度
        double gyroYrate = gy / 131.0;
        double gyroZrate = gz / 131.0;

        // 加速度计数据计算角度
        roll = atan2(ay, az) * 180.0 / M_PI;                        // x 轴角度
        pitch = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / M_PI; // y 轴角度

        // 假设初始时 yaw 角度为 0，陀螺仪数据用于计算 yaw
        yaw = atan2(ay, ax) * 180.0 / M_PI; // z 轴角度，简单使用加速度计计算yaw

        // Kalman 滤波器计算
        double dt = 0.01;                                          // 假设是 10 毫秒采样一次
        double kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // 使用 Kalman 滤波器获取 x 轴角度
        double kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // 使用 Kalman 滤波器获取 y 轴角度
        double kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);   // 使用 Kalman 滤波器获取 z 轴角度

        // 输出结果
        std::cout << "Roll (X angle): " << kalAngleX << "Pitch (Y angle): " << kalAngleY << "Yaw (Z angle): " << kalAngleZ << std::endl;

        // 构造要发送的字符串
        char dataToSend[100];
        snprintf(dataToSend, sizeof(dataToSend), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", kalAngleX, kalAngleY, kalAngleZ, roll, pitch, yaw);

        // 发送数据
        if (send(sock, dataToSend, strlen(dataToSend), 0) < 0)
        {
            std::cerr << "Failed to send data" << std::endl;
            break;
        }

        usleep(10000);
    }

    // 关闭连接
    close(fd);
    close(sock);

    return 0;
}
