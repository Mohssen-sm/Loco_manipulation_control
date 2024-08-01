
#include "IOSDK.h"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

IOSDK::IOSDK(UNITREE_LEGGED_SDK::LeggedType robot, int cmd_panel_id, int port) : _control(robot),
#ifdef GO1
                                                                                 _udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007),
#else
                                                                                 _udp(UNITREE_LEGGED_SDK::LOWLEVEL),
#endif
                                                                                 loop_loco_manipulation("loco_manipulation", 0.001, boost::bind(&IOSDK::socketSendRecv, this))
{

    _udp.InitCmdData(_lowCmd);
    if (cmd_panel_id == 1)
    {
        cmdPanel = new WirelessHandle(&_lowState);
    }
    else if (cmd_panel_id == 2)
    {
        cmdPanel = new KeyBoard();
    }
    rotmat = Eigen::Matrix3d::Identity();

    std::thread socketThread(&IOSDK::setupSocket, this, port);
    socketThread.detach();
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    _udp.Recv();
    _udp.GetRecv(_lowState);
    for (int i(0); i < 12; ++i)
    {
        _lowCmd.motorCmd[i].mode = 0X0A;
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
    }

    for (int i(0); i < 12; ++i)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }

    for (int i(0); i < 3; ++i)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    Eigen::Quaterniond quat(state->imu.quaternion[0], state->imu.quaternion[1], state->imu.quaternion[2], state->imu.quaternion[3]);
    rotmat = quat.toRotationMatrix();

    for (int i = 0; i < 4; i++)
    {
        state->FootForce[i] = _lowState.footForce[i];
    }

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();

    for (int i = 0; i < 2; i++)
    {
        state->userValue.manipulation_force[i] = Highcmd.manipulation_force(i);
    }
    state->userValue.vx = Highcmd.velocity_cmd[0];
    state->userValue.vy = Highcmd.velocity_cmd[1];
    state->userValue.turn_rate = Highcmd.omega_cmd[2];

    // _control.PowerProtect(_lowCmd, _lowState, 10);

    _udp.SetSend(_lowCmd);
    _udp.Send();
}

int IOSDK::setupSocket(int port)
{
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    // Bind the socket to an IP address and port
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port); // Change to your desired port
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
    {
        std::cerr << "Error binding socket" << std::endl;
        return -1;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 5) == -1)
    {
        std::cerr << "Error listening on socket" << std::endl;
        return -1;
    }

    // Accept a connection
    clientSocket = accept(serverSocket, NULL, NULL);

    if (clientSocket == -1)
    {
        std::cerr << "Error accepting connection" << std::endl;
        return -1;
    }

    // start the loop
    loop_loco_manipulation.start();

    return 0;
}

void IOSDK::socketSendRecv()
{
    ssize_t bytesRead = recv(clientSocket, &_dataRecv, sizeof(_dataRecv), 0);
    if (bytesRead == -1)
    {
        std::cerr << "Error receiving data" << std::endl;
    }
    else
    {
        std::cout << "Received: " << _dataRecv.force[0] << std::endl;

        // Eigen::Vector3d force_body = (Eigen::Vector3d() << _dataRecv.force[0], _dataRecv.force[1], 0).finished();
        // Highcmd.manipulation_force = rotmat * force_body;

        // Highcmd.omega_cmd[2] = _dataRecv.omega[2];
        // Highcmd.velocity_cmd[1] = _dataRecv.velocity[1]; // + 3 * (distance_body[1]);
        // // Highcmd.velocity_cmd[0] = 2 * (distance_body[0] - _quad.leg_offset_x);
    }
}