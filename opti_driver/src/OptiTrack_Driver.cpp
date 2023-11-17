/*
 * Authors: Hyunchul Bae @ KAIST
 */

#include <iostream>
#include <string>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <math.h>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

static std::mutex mInit;
static std::mutex mTopic;
static std::mutex mPub;
static std::condition_variable cv;

geometry_msgs::msg::PointStamped opti_msg_;

NATNET_API::NatNetClient* g_pClient = NULL;

rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr* publisher_;

std::vector<std::string> Init_name;
int n_rigid_body = 0;
bool is_init = false;

using namespace std;
using namespace std::chrono_literals;
using namespace std::chrono;

void NATNET_CALLCONV dataHandler(sFrameOfMocapData* data, void* pUserData);

class RosPublisher : public rclcpp::Node, public NatNetClient
{
    public:
        RosPublisher()
        :Node("Opti_Node")
        {           
            RCLCPP_INFO(this->get_logger(), "Create OptiTrack Publisher Node.\n");

            // NatNEt Initialize
            NatNetInit();

            // Trying Connect OptiTrack until success...
            int retCode = OptiConnect(g_pClient);
            RCLCPP_INFO(this->get_logger(), "Successly Connecting OptiTrack!\n");

            // Get dataHandler data for publisher initialize

            g_pClient->GetDataDescriptionList(&pDataDefs);

            for(int i = 0; i < pDataDefs->nDataDescriptions; i++)
            {
                if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
                {
                    sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                    Init_name.emplace_back(pRB->szName);
                }
            }

            n_rigid_body = Init_name.size();
            
            RCLCPP_INFO(this->get_logger(), "OptiTrack Capture %d Rigidbodies...", n_rigid_body);
            
            publisher_ = new rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr[n_rigid_body];
            
            for(int i = 0; i < n_rigid_body; i++)
            {
                std::string topic_name = Init_name[i];
                RCLCPP_INFO(this->get_logger(), "Now %s Publisher Allocating...", topic_name);

                publisher_[i] = this->create_publisher<geometry_msgs::msg::PointStamped>(topic_name, 50);

                RCLCPP_INFO(this->get_logger(), "%s Publisher Allocation Finish!\n", topic_name);
            }

            is_init = true;
        }

    private:
        NATNET_API::sDataDescriptions* pDataDefs;
        NATNET_API::sNatNetClientConnectParams g_connectParams;
        
        void NatNetInit()
        {
            g_connectParams.connectionType = ConnectionType_Multicast;
            g_connectParams.serverAddress = "127.0.0.1";
            g_connectParams.localAddress = "127.0.0.1";
            g_pClient = new NATNET_API::NatNetClient();

            unsigned char ver[4];
            NATNET_API::NatNet_GetVersion(ver);
            RCLCPP_INFO(this->get_logger(), "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);
        }

        int OptiConnect(NATNET_API::NatNetClient* client)
        {       
            ErrorCode ret = ErrorCode_OK;

            client->Disconnect();

            int retCode = client->Connect(g_connectParams);

            while(retCode == ErrorCode_OK)
            {
                RCLCPP_WARN(this->get_logger(), "Unable to connect, retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                return 1;
            }
            return retCode;
        }
}; 

int main( int argc, char* argv[] )
{
    // ROS Initialize
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<RosPublisher>();

    // Spin Node~!~!~!
    RCLCPP_INFO(node->get_logger(), "Starting Publish Rigidbody Information...!");
    
    while(rclcpp::ok()){
        mPub.lock();
        g_pClient->SetFrameReceivedCallback(dataHandler, g_pClient);
        rclcpp::spin_some(node);
        mPub.unlock();
    }

    RCLCPP_INFO(node->get_logger(), "Bye Bye~! :)");

    g_pClient->Disconnect();

    delete [] publisher_;
    delete g_pClient;

    return 0;
}

void NATNET_CALLCONV dataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NATNET_API::NatNetClient* pClient = (NATNET_API::NatNetClient*) pUserData;

    // int num = data->nRigidBodies;
    sRigidBodyData* bodies_ptr = data->RigidBodies;

    mTopic.lock();

    for(int i=0; i < n_rigid_body; i++)
    {
        rclcpp::Time topic_time = rclcpp::Clock(RCL_ROS_TIME).now();
        std::string name = Init_name[i];
        
        tf2::Quaternion tmp_quat;
        tmp_quat.setValue(bodies_ptr[i].qx, bodies_ptr[i].qy, bodies_ptr[i].qz, bodies_ptr[i].qw);
        
        tf2::Matrix3x3 eular(tmp_quat);
        tf2::Matrix3x3 T_opti_to_world(1,  0,  0,
                                        0,  0,  -1,
                                        0,  1,  0);

        eular = T_opti_to_world * eular;

        double roll, pitch, yaw;
        eular.getRPY(roll, pitch, yaw);
        
        opti_msg_.header.frame_id = name;
        opti_msg_.header.stamp = topic_time;

        opti_msg_.point.x = bodies_ptr[i].x * 100;
        opti_msg_.point.y = -bodies_ptr[i].z * 100;
        opti_msg_.point.z = yaw;

        publisher_[i]->publish(opti_msg_);
    }

    mTopic.unlock();
}
