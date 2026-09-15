/*
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

constexpr uint16_t PORT = 49152;       // Porta Net F/T
constexpr uint16_t COMMAND = 2;        // Start streaming
constexpr uint32_t NUM_SAMPLES = 1;    // Numero di campioni

struct Response
{
    uint32_t rdt_sequence;
    uint32_t ft_sequence;
    uint32_t status;
    std::array<int32_t, 6> FTData;
};

int main(int argc, char** argv)
{
    const std::array<const char*, 6> axes =
    {
        "Fx", "Fy", "Fz",
        "Tx", "Ty", "Tz"
    };

    // Apertura socket UDP
    int socketHandle = socket(AF_INET, SOCK_DGRAM, 0);

    if (socketHandle < 0)
    {
        std::cerr << "Unable to create socket.\n";
        return 1;
    }

    // Costruzione richiesta
    std::array<uint8_t, 8> request{};

    *reinterpret_cast<uint16_t*>(&request[0]) = htons(0x1234);
    *reinterpret_cast<uint16_t*>(&request[2]) = htons(COMMAND);
    *reinterpret_cast<uint32_t*>(&request[4]) = htonl(NUM_SAMPLES);

    // Risoluzione hostname/IP
    hostent* he = gethostbyname("192.168.1.1");

    if (he == nullptr)
    {
        std::cerr << "Unable to resolve host\n";
        close(socketHandle);
        return 2;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    std::memcpy(
        &addr.sin_addr,
        he->h_addr_list[0],
        he->h_length);

    // Connessione UDP
    if (connect(socketHandle,
                reinterpret_cast<sockaddr*>(&addr),
                sizeof(addr)) < 0)
    {
        std::cerr << "Connection failed.\n";
        close(socketHandle);
        return 3;
    }

    // Invio richiesta
    if (send(socketHandle,
             request.data(),
             request.size(),
             0) < 0)
    {
        std::cerr << "Send failed.\n";
        close(socketHandle);
        return 4;
    }

    // Ricezione risposta
    std::array<uint8_t, 36> rawResponse{};

    ssize_t received = recv(socketHandle,
                            rawResponse.data(),
                            rawResponse.size(),
                            0);

    if (received != static_cast<ssize_t>(rawResponse.size()))
    {
        std::cerr << "Receive failed.\n";
        close(socketHandle);
        return 5;
    }

    Response resp{};

    resp.rdt_sequence =
        ntohl(*reinterpret_cast<uint32_t*>(&rawResponse[0]));

    resp.ft_sequence =
        ntohl(*reinterpret_cast<uint32_t*>(&rawResponse[4]));

    resp.status =
        ntohl(*reinterpret_cast<uint32_t*>(&rawResponse[8]));

    for (int i = 0; i < 6; ++i)
    {
        resp.FTData[i] =
            ntohl(*reinterpret_cast<int32_t*>(
                &rawResponse[12 + i * 4]));
    }

    // Stampa risultati
    std::cout << "Status: 0x"
              << std::hex << resp.status
              << std::dec << '\n';

    for (int i = 0; i < 6; ++i)
    {
        std::cout << axes[i]
                  << ": "
                  << resp.FTData[i]
                  << '\n';
    }

    close(socketHandle);

    return 0;
}

*/

#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include "KinovaLiralab.hpp"
#include "ftSenseHandler.hpp"
#include "zForceControl.hpp"
#include <DatasetRecorder.hpp>
#include <TerminationHandler.hpp>
#include <SocketLiralab.hpp>
#include <AurovasSocket.hpp>
#include <thread>
#include <chrono>

#define PORT 10000
#define PORT_REALTIME 10001

namespace KORTEX = Kinova::Api;


int main(int argc, char **argv)
{   
    // Uncomment the application
    
    // ******************************************** 
    // *********** REGISTER NEW EPISODES **********
    // ********************************************
    {
        /*
        if(argc < 2) {std::cerr << "\nMissing argument: ['Dataset Name']\n\n"; return -1;}
        
        TerminationHandler t;
        KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
        DatasetRecorder datasetRecorder(static_cast<string>(argv[1]), robot);

        // Subscribe callbacks for CTRL-C signal
        TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
        TerminationHandler::RegisterCallback([&datasetRecorder](){datasetRecorder.StopRecord();});
        
        robot->StartHandGuidance();
        std::cin.get();
        datasetRecorder.StartRecord(600);   
        std::cin.get();
        datasetRecorder.StopRecord();
        robot->StopApp();
        */
    } 

    // ********************************************
    // **************** RUN ACT *******************
    // ********************************************
    {
        /*
        bool useForceSensor = true;
        TerminationHandler t;
        CanDevice* forceSensor;
        if(useForceSensor) forceSensor = new CanDevice();
        KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
        KinovaLiralab::SocketLiralab socket{5009, [&robot]{robot->StopApp();}};

        // Subscribe callbacks for CTRL-C signal
        TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
        TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

        std::cout << "Position the probe on belly and press ENTER" << std::endl;
        robot->StartHandGuidance();
        std::cin.get();

        // ---------- Connect CAN
        if(useForceSensor)
        {
            if(!forceSensor->Open("can0")) {std::cout << "Cannot open CAN\n" << std::endl;return -1;}
            else std::cout << "CAN Opened\n" << std::endl;
            forceSensor->InitCompensation();

            while (!forceSensor->IsWrenchReady())
            {
                forceSensor->Receive();
                std::cout << "Waiting for can msgs...\n";
            }
        }
        bool setTare = true;  
        double sensorWrench[6];

        // ---------- Send initial position
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        KinovaLiralab::RobotState state = robot->GetRobotState();
        socket.WriteRobotState(state, sensorWrench, useForceSensor); // sensorWrench is not usefull

        // ---------- Wait acknoledge from python
        while(socket.Read() != "RUN");
        robot->StopApp();
        robot->TorqueControl();
        std::cout << "Wait for torque control... [3]" << std::flush;
        for(int i = 0; i < 3; i++)
        {
            std::cout << "\b\b";
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << 2-i << "]" << std::flush;
        }
        std::cout << "\b\bTORQUE CONTROL READY]" << std::endl;

        KDL::Frame newFrame{};
        while(true)
        {
            if(useForceSensor)
            {
                if(setTare)
                {
                    setTare = false;
                    forceSensor->SetTare();
                }
            }

            state = robot->GetRobotState();
            if(useForceSensor)
            {
                forceSensor->ReceiveAllForceAndTorque();   
                forceSensor->GetWrenchCompensated(sensorWrench, state);
            }
            std::cout << sensorWrench[0] << ", " << sensorWrench[1] << ", " << sensorWrench[2] << "\n------\n";
            socket.WriteRobotState(state, sensorWrench, useForceSensor);
            
            if(socket.ReadFrame(newFrame) < 0) break;

            robot->SetEquilibriumPose(newFrame);
        }
        
        std::cin.get();
        robot->StopApp();
        */
    }
    

    // ********************************************
    // ************** Z FORCE CONTROL *************
    // ********************************************
    
    {
        /*
        TerminationHandler t;
        KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
        TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
        //ZForceControl* zForceControl = new ZForceControl(robot);
        robot->StartHandGuidance();
        while(true)
        {
            //zForceControl->RunControl(-12.0);
        }
        robot->StopApp();
        */
        /*
        TerminationHandler t;
        KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
        TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});

        CanDevice* forceSensor = new CanDevice();
        if(!forceSensor->Open("can0")) {"Cannot open CAN";return 0;}
        forceSensor->InitCompensation();
        double sensorWrench[6];
        KinovaLiralab::RobotState state;
        while (!forceSensor->IsWrenchReady())
        {
            forceSensor->Receive();
            std::cout << "Waiting for can msgs...\n";
        }

        robot->StartHandGuidance();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        forceSensor->SetTare();

        while(true)
        {
            state = robot->GetRobotState();
            forceSensor->ReceiveAllForceAndTorque();   
            forceSensor->GetWrenchCompensated(sensorWrench, state);
            std::cout << "Comp: " << sensorWrench[0] << ", " << sensorWrench[1] << ", " << sensorWrench[2] << "\n";          
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        }
        robot->StopApp();
        */
    }
    
    // *****************************************
    // **************** GRID SEARCH ************
    // *****************************************
    {
        TerminationHandler t;
        KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
        KinovaLiralab::SocketLiralab socket{5022, [&robot]{robot->StopApp();}};
        
        // Subscribe callbacks for CTRL-C signal
        TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
        TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

        std::cout << "Position the probe on belly and press ENTER" << std::endl;
        robot->StartHandGuidance();
        std::cin.get();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // ---------- Wait acknoledge from python
        while(socket.Read() != "RUN");
        robot->StopApp();
        robot->TorqueControl();
        std::cout << "Wait for torque control... [3]" << std::flush;
        for(int i = 0; i < 3; i++)
        {
            std::cout << "\b\b";
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << 2-i << "]" << std::flush;
        }
        std::cout << "\b\bTORQUE CONTROL READY]" << std::endl;

        int GRID_SIZE = 2;
        float CELL_SIZE_X = 0.02;
        float CELL_SIZE_Y = 0.02;
        float CELL_SIZE_Z = 0.02;
        KDL::Frame nextFrame = robot->GetEEFrame();
        KDL::Frame initialFrame = nextFrame;
        float Z0 = initialFrame.p.z();

        for(int z = 0; z < GRID_SIZE; z++)
        {
            printf("Layer %d\n", z);
            for(int x = 0; x < GRID_SIZE; x++)
            {
                for(int y = 0; y < GRID_SIZE; y++)
                {
                    printf("(%d, %d)\n", x, y);

                    robot->SetEquilibriumPoseWithCustomVelocity(nextFrame, 0.01);
                    robot->WaitUntilReachedPosition(nextFrame);

                    socket.Write("MEASURE");
                    float diameter = socket.ReadGridStep();
        
                    if(diameter > 0)
                    {
                        std::cout << "AORTA DIAMETER: " << diameter << std::endl;
                        robot->StopApp();
                        return 0;
                    }      

                    nextFrame.p[2] = Z0 + 0.02;
                    robot->SetEquilibriumPoseWithCustomVelocity(nextFrame, 0.01);
                    robot->WaitUntilReachedPosition(nextFrame);

                    nextFrame.p = initialFrame.p + KDL::Vector(CELL_SIZE_X * x, CELL_SIZE_Y * y, 0.02);
                    robot->SetEquilibriumPoseWithCustomVelocity(nextFrame, 0.01);
                    robot->WaitUntilReachedPosition(nextFrame);

                    nextFrame.p[2] = Z0 - CELL_SIZE_Z * z;
                }
            }
        }

        std::cout << "AORTA NOT FOUND" << std::endl;
        robot->StopApp();

    }

    // ********************************************
    // **************** AUROVAS KINOVA ************
    // ********************************************
    {
    /*
    TerminationHandler t;
    AurovasSocket socket;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe

    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

    std::vector<std::string> message = socket.Read();
    socket.Write("WAITING;STARTING");
    socket.Write("STARTING;WAITING");

    socket.Read();  // HANDGUIDING;POSITION
    robot->StartHandGuidance();
    std::cin.get();
    robot->StopApp();
    std::vector<string> msg = socket.Read();  // GLOBALSEARCH;5,5,8,8,35,10
    int row = std::stoi(msg[1]);
    int col = std::stoi(msg[2]);
    int dist = std::stoi(msg[3]);
    int x = 0;
    int dir = 1; // 1 = avanti, -1 = indietro

    socket.Write("HANDGUIDING;WAITING");
    socket.Write("WAITING;GLOBALSEARCH");

    robot->TorqueControl();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    KDL::Frame eeInitial = robot->GetEEFrame();
    for(int y = 0; y < col; y++)
    {
        for(int step = 0; step < row; step++)
        {
            std::cout << "Moving " << x << ", " << y << std::endl;

            KDL::Frame eeFrame = eeInitial;
            eeFrame.p[0] -= dist/100.0 * x;
            eeFrame.p[1] += dist/100.0 * y;

            robot->SetEquilibriumPose(eeFrame);
            std::cout << "Segmentation" << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cin.get();
            if(socket.Read()[0] == "True") break;
            socket.Write("TEST;TEST");       
            if(step < 4)
                x += dir;
        }

        dir *= -1;
    }
    
    socket.Write("GLOBALSEARCH;LOCALSEARCH");
    // LOCAL SEARCH
    socket.Read();
    socket.Write("LOCALSEARCH;WAITING");
    
    robot->StopApp();
    */
    }
    // robot->StopApp();

    /*
    TerminationHandler t;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe
    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});

    //std::cout << "EXAMPLE" << std::endl;    
    robot->TorqueControl();
    std::cin.get();
    
    for(int i = 0; i < 3; i++)
    {
        std::cout << i << std::endl;
        KDL::Frame eeFrame = robot->GetEEFrame();
        eeFrame.p[2] += 0.05;
        robot->SetEquilibriumPose(eeFrame);
    
        std::this_thread::sleep_for(std::chrono::seconds(2));
    
        eeFrame = robot->GetEEFrame();
        eeFrame.p[2] -= 0.05;
        robot->SetEquilibriumPose(eeFrame);
    }
    robot->StopApp();
    */
}
