#ifndef SOCKET_LIRALAB
#define SOCKET_LIRALAB

#include <iostream>
#include <string>
#include <RobotState.hpp>
#include <unistd.h>
#include <arpa/inet.h>
#include <kdl/kdl.hpp>

namespace KinovaLiralab
{
    class SocketLiralab
    {
    private:
        int _server;
        int _client;
        char buffer[1024] = {0};
        std::function<void()> _brokenPipeCallback;

    public:
        SocketLiralab(uint16_t, std::function<void()>);
        ~SocketLiralab();
        void CloseSocket();
        int WriteRobotState(const KinovaLiralab::RobotState&, double* wrench, bool sendForce);
        int Write(const std::string&);
        auto Read() -> std::string;
        int ReadFrame(KDL::Frame&);
        float ReadGridStep();
    };
    
    SocketLiralab::SocketLiralab(uint16_t port, std::function<void()> brokenPipeCallback)
    {
        _brokenPipeCallback = brokenPipeCallback;
        _server = socket(AF_INET, SOCK_STREAM, 0);
        if (_server < 0) {perror("socket"); return;}

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(_server, (sockaddr*)&addr, sizeof(addr)) < 0) {perror("bind");return;}
        listen(_server, 1);
        std::cout << "Server in ascolto ..." << std::endl;
        _client = accept(_server, nullptr, nullptr);
        if (_client < 0) {perror("accept");return;}
    }

    int SocketLiralab::WriteRobotState(const KinovaLiralab::RobotState& robotState, double* wrench, bool sendForce = false)
    {
        std::string msg{""};
        for(auto f : robotState._eePose)
        {   
            msg.append(std::to_string(f));
            msg.append(";");
        }

        if(!sendForce) return Write(msg);

        for(int i = 0; i < 3; i++)
        {
            msg.append(std::to_string(wrench[i]));
            msg.append(";");
        }
        //std::cout << ">>> " << msg << "\n";
        return Write(msg);
    }

    int SocketLiralab::Write(const std::string& msg)
    {
        ssize_t ret = send(_client, (msg + "\n").c_str(), msg.size(), MSG_NOSIGNAL);
        if(ret < 0)
        {
            std::cout << "Errore nella send della socket." << std::endl;
            _brokenPipeCallback();
        }
        return 0;
    }

    std::string SocketLiralab::Read()
    {
        ssize_t bytes = recv(_client, buffer, sizeof(buffer) - 1, 0);
        if (bytes <= 0) { perror("recv");return ""; }
        std::string received(buffer);
        //std::cout << "<<< " << received << "\n";
        return received;
    }

    int SocketLiralab::ReadFrame(KDL::Frame& outFrame)
    {
        std::string resultAsString = Read();

        std::vector<double> v;
        std::stringstream ss(resultAsString);
        std::string token;

        while (std::getline(ss, token, ';')) {
        v.push_back(std::stod(token));
        }

        if (v.size() != 12) {
            std::cout << "String must contain exactly 12 values:\nREAD: " << resultAsString << std::endl;
            _brokenPipeCallback();
            return -1;
        }

        KDL::Vector p(v[0], v[1], v[2]);

        KDL::Rotation R(
        v[3],  v[4],  v[5],
        v[6],  v[7],  v[8],
        v[9],  v[10], v[11]
        );

        outFrame.p = p;
        outFrame.M = R;
        return 0;
    }

    float SocketLiralab::ReadGridStep()
    {
        std::string resultAsString = Read();

        std::vector<double> v;
        std::stringstream ss(resultAsString);
        std::string token;

        while (std::getline(ss, token, ';')) {
            v.push_back(std::stod(token));
        }
        if(v.size() == 0) return -1.0;
        return v[0];
    }
    
    void SocketLiralab::CloseSocket()
    {
        close(_server);
        close(_client);
    }

    SocketLiralab::~SocketLiralab()
    {
        CloseSocket();
    }
    
}

#endif SOCKET_LIRALAB