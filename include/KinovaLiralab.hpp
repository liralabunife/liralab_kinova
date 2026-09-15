#ifndef KINOVA_LIRALAB
#define KINOVA_LIRALAB

#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <google/protobuf/text_format.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <RobotState.hpp>

#include <math.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <iomanip>

#define PORT 10000
#define PORT_REALTIME 10001

namespace KinovaLiralab
{
    namespace KORTEX = Kinova::Api;

    class Robot
    {
        private:
        KORTEX::Session::CreateSessionInfo _sessionInfo;
        Eigen::MatrixXd _jacobian{6,7};
        const std::chrono::seconds ACTION_WAITING_TIME = std::chrono::seconds(1);

        // High Level
        KORTEX::TransportClientTcp* _transport;
        KORTEX::RouterClient* _router;
        KORTEX::SessionManager* _session;
        KORTEX::Base::BaseClient* _base;
        //-------
        // Low level (real time)
        KORTEX::TransportClientUdp* _transportRealTime;
        KORTEX::RouterClient* _routerRealTime;
        KORTEX::SessionManager* _sessionRealTime;
        KORTEX::BaseCyclic::BaseCyclicClient* _baseRealTime;
        KORTEX::ActuatorConfig::ActuatorConfigClient* _actuatorConfig;
        //-------
        // static Callbacks
        static void ErrorCallback(KORTEX::KError e) {std::cout << "_________ callback error _________" << e.toString();}
        static std::function<void(KORTEX::Base::ActionNotification)> CheckForEndOrAbort(bool& finished)
        {
            return [&finished](KORTEX::Base::ActionNotification notification)
            {
                std::cout << "EVENT : " << KORTEX::Base::ActionEvent_Name(notification.action_event()) << std::endl;

                // The action is finished when we receive a END or ABORT event
                switch(notification.action_event())
                {
                case KORTEX::Base::ActionEvent::ACTION_ABORT:
                case KORTEX::Base::ActionEvent::ACTION_END:
                    finished = true;
                    break;
                default:
                    break;
                }
            };
        }
        //-------
        // private functions
        void Print(std::string_view s) { std::cout << s << std::endl;}
        int64_t GetTickUs();
        void PrintException(KORTEX::KDetailedException& ex);
        void UpdateRobotState(
            const KORTEX::BaseCyclic::Feedback& baseFeedback,
            const KDL::JntArray& q);
        //-------
        // For Jacobian
        urdf::Model _urdfModel;
        KDL::Chain _robotChain;
        KDL::Frame _eeFrame;
        KDL::Tree _kdlTree;
        KDL::ChainFkSolverPos_recursive* _fkSolver;
        KDL::ChainIkSolverPos_LMA* _ikSolver;

        //-------
        // Thread
        thread _realtimeThread;
        std::atomic<bool> _stopApp{false};
        std::mutex _mRobotState;
        std::mutex _meeEquilibriumPose;
        KDL::JntArray _equilibriumJointPosition;
        KDL::Frame _equilibriumEEPosition;          // The one followed by joints
        KDL::Frame _virtualEquilibriumEEPosition;   // The one followed by _equilibriumEEPosition

        KinovaLiralab::RobotState _robotState{
            std::vector<float>(7, 0.0f),   // _jointPositions
            std::vector<float>(7, 0.0f),   // _jointTorques
            std::vector<float>(7, 0.0f),   // _jointVels
            std::vector<float>(12, 0.0f),  // _eePose
            std::vector<float>(3, 0.0f),     // _eeVel
        };
        std::chrono::steady_clock::time_point _previousTime;
        bool _firstDt = true;


        public:
            Robot(std::string urdf_file);
            ~Robot();
            void GoHome();
            void EvaluateJacobian();
            void EvaluateJacobianKDL();
            void EvaluateJacobianKDLNumerically();
            void VelocityControl();
            void VelocityControlHighLevel();
            void TorqueControlExample();
            void TorqueControl();
            void SetEquilibriumPose(KDL::Frame ee);
            void SetEquilibriumPoseWithCustomVelocity(KDL::Frame ee,float maxLinearVelocity);
            void WaitUntilReachedPosition(const KDL::Frame& finalPosition, double treshold = 0.01, int msPollingTime = 100);
            void StartHandGuidance();
            void StopApp();
            auto GetRobotState()    -> KinovaLiralab::RobotState;
            auto GetEEFrame()       -> KDL::Frame;
    };

}


#endif // KINOVA_LIRALAB
