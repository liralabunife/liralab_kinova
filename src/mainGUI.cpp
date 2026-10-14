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
    TerminationHandler t;
    CanDevice* forceSensor = new CanDevice();
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/liralab_kinova/urdf/gen3_ESAOTE_FTSense.urdf"); // _ESAOTE_convex_probe
    KinovaLiralab::SocketLiralab socket{5026, [&robot]{robot->StopApp();}};

    // Subscribe callbacks for CTRL-C signal
    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

    while (true)
    {
        std::cout << "WAITING FOR APP:\n\t-[AORTA]\n\t-[JUGULAR]\n\t-[QUIT]" << std::endl;
        std::string app = socket.Read();
        while(app != "AORTA" && app != "JUGULAR" && app != "QUIT");
    
        // ********************************************
        // ************ GRID CONTROL AORTA ************
        // ********************************************
        if(app == "AORTA")
        {
            std::cout << "Position the probe on belly and press START" << std::endl;
            robot->StartHandGuidance();
            while(socket.Read() != "START");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // ---------- Wait acknoledge from python
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
            int GRID_SIZE = 5;
            float CELL_SIZE_X = 0.02;
            float CELL_SIZE_Y = 0.02;
            float CELL_SIZE_Z = 0.02;
            KDL::Frame nextFrame = robot->GetEEFrame();
            KDL::Frame initialFrame = nextFrame;
            float Z0 = initialFrame.p.z() - 0.01;
    
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
                            while(socket.Read() != "QUIT");
                            goto end_app;   // fuck yea a goto to perform multiple breaks
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
        }
        
        // *****************************************
        // **************** GRID SEARCH ************
        // *****************************************
        if(app == "JUGULAR")
        {   
            std::cout << "Position the probe on the neck and press START" << std::endl;
            robot->StartHandGuidance();
            while(socket.Read() != "START");
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
    
            int GRID_SIZE = 5;
            float CELL_SIZE_X = 0.02;
            float CELL_SIZE_Y = 0.02;
            float CELL_SIZE_Z = 0.02;
            KDL::Frame nextFrame = robot->GetEEFrame();
            KDL::Frame initialFrame = nextFrame;
            float Z0 = initialFrame.p.z() - 0.01;
    
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
                            goto end_app;   // fuck yea a goto to perform multiple breaks
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
        }

        if(app == "QUIT")
        {
            robot->StopApp();
        }

        end_app: robot->StopApp();
    }
    
}
