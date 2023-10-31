/* Includes ------------------------------------------------------------------*/
#include "CoppeliaSim.h"
#include "matplotlibcpp.h"
#include "sys_log.h"
#include "two_wheel.h"
#include "Eigen/Dense"

/* Usr defines ---------------------------------------------------------------*/
using namespace std;
using Eigen::MatrixXd;
namespace plt = matplotlibcpp;
enum
{
    yaw = 0,
    pitch,
    roll
};
enum
{
    gimble_yaw = 0,
};
_simObjectHandle_Type *Body;
_simObjectHandle_Type *Joint[4];
_simSignalHandle_Type *EulerAngle[3];
LogFilter_t Filters[3];
CChassis Gimble(1, 0, 0.15, 9999, 9999, 0);
/* Founctions ----------------------------------------------------------------*/
uint32_t getSimTime();

/**
* @brief This is the main function for user.
*/
int8_t P, M, target;
float a0[15] = {0.042, 0.1125, 0.175, 0.234, 0.2865, 0.3335, 0.3765, 0.415, 0.45, 0.482, 0.5106, 0.5365, 0.56, 0.5812, 0.6};
float alpha;
float u = 0;
float Y_now;
int32_t angle_now;
float t_now;
void Usr_Main()
{
    //MPC
    //initial
    P = 15;
    M = 2;
    alpha = 0.1;
    
    static MatrixXd Y_ref = MatrixXd::Zero(P, 1);
    static MatrixXd Y_cor = MatrixXd::Zero(P, 1);
    static MatrixXd Y0 = MatrixXd::Zero(P, 1);
    static MatrixXd A = MatrixXd::Zero(P, M);
    static MatrixXd du = MatrixXd::Zero(M, 1);
    static MatrixXd s = MatrixXd::Zero(P, P);
    static MatrixXd h = MatrixXd::Zero(P, 1);
    static MatrixXd Q = MatrixXd::Zero(P, P);
    static MatrixXd R = MatrixXd::Zero(M, M);
    static MatrixXd H = MatrixXd::Zero(M, M);

    //dongtai juzhen
    for (int i = 0; i < P; i++)
    {
        A(i, 0) = a0[i];
    }
    
    for (int i = 1; i < P; i++)
    {
        for (int j = 1; j < M; j++)
        {
            if (i >= j)
                A(i, j) = A(i - 1, j - 1);
        }
    }
    //yiweijuzhen
    for (int i = 0; i < (P - 1); i++)
    {
        s(i, i + 1) = 1;
    }
    s(P - 1, P - 1) = 1;
    //H juzhen
    for (int i = 0; i < P; i++)
    {
        h(i, 0) = 0.5;
    }
    //Q juzhen
    for(int i = 0; i < P; i++)
    {
        Q(i,i) = 1;
    }
    //R juzhen
    for(int i = 0; i < M; i++)
    {
        R(i,i) = 1;
    }
    Y_now = EulerAngle[0]->data;
    target = EulerAngle[1]->data;
    if (Y_now == 0)
        ;
    else
    {
        //reference
        Y_ref(0, 0) = alpha * Y_now + (1 - alpha) * target;
        for (int i = 1; i < P; i++)
        {
            Y_ref(i, 0) = alpha * Y_ref(i - 1, 0) + (1 - alpha) * target;
        }
        //correct the reference
        Y_cor = Y0 + h * (Y_now - Y0(0,0));
        //yiwei
        Y0 = s * Y_cor;
        //Predict
        Y0 = A * du + Y0;
        //Optimization
        H = A.transpose() * Q * A + R;
        du = H.inverse() * A.transpose() * Q * (Y_ref - Y0);

        u += du(0, 0);
    }
}

/**
* @brief User can config simulation client in this function.
* @note  It will be called before entering the main loop.   
*/
void Usr_ConfigSimulation()
{//SIM_VELOCITY | CLIENT_RO 
    Body = CoppeliaSim->Add_Object("Body", OTHER_OBJECT, {SIM_ORIENTATION | CLIENT_RO, SIM_VELOCITY | CLIENT_RO});
    Joint[gimble_yaw] = CoppeliaSim->Add_Object("GIMBLE", JOINT, {SIM_VELOCITY | CLIENT_RW,SIM_POSITION | CLIENT_RW,SIM_FORCE | CLIENT_RW});
    EulerAngle[0] = CoppeliaSim->Add_Object("gyrovelocity", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
    EulerAngle[1] = CoppeliaSim->Add_Object("target", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RW});
}

/**
* @brief These two function will be called for each loop.
*        User can set their message to send or read from sim enviroment.
*/
void Usr_SendToSimulation()
{

    Joint[gimble_yaw]->obj_Target.angVelocity_f = (u >= 0) ? -2000 : 2000;
    Joint[gimble_yaw]->obj_Target.torque_f = fabs(u);
}

void Usr_ReadFromSimulation()
{
    //Gimble.joint_angle[gimble_yaw] = Joint[gimble_yaw]->obj_Data.angle_f;
    //Gimble.joint_rpm[gimble_yaw] = Joint[gimble_yaw]->obj_Data.angVelocity_f;
    //Gimble.joint_torque[gimble_yaw] = Joint[gimble_yaw]->obj_Data.torque_f;
}

/**
* @brief It's NOT recommended that user modefies this function.
*        Plz programm the functions with the prefix "Usr_". 
*/
int main(int argc, char *argv[])
{
    CoppeliaSim_Client *hClient = &CoppeliaSim_Client::getInstance();
    /*
        System Logger tool init.
    */
    std::cout << "[System Logger] Configuring... \n";
    SysLog->getMilliTick_regist(getSimTime);
    std::cout << "[System Logger] Logger is ready ! \n";

    /*
        Simulation connection init.
    */
    std::cout << "[CoppeliaSim Client] Connecting to server.. \n";
    while (!hClient->Start("127.0.0.1", 5000, 5, true))
    {
    };
    std::cout << "[CoppeliaSim Client] Successfully connected to server, configuring...\n";
    Usr_ConfigSimulation();
    std::cout << "[CoppeliaSim Client] Configure done, simulation is ready ! \n";

    while (1)
    {
        if (hClient->Is_Connected())
        {
            hClient->ComWithServer();
        }
        Usr_ReadFromSimulation();
        Usr_Main();
        Usr_SendToSimulation();
    };
}

uint32_t getSimTime()
{
    return 0;
}
/************************* END-OF-FILE SCUT-ROBOTLAB **************************/