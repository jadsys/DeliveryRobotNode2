// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// comp_tutrial/adder.h　adder.msgから生成されたメッセージを定義しているヘッダ
//#include "comp_tutorial/adder.h"

#include "delivery_robot/r_state.h"   // 状態報告メッセージ
#include "delivery_robot/r_emergency_command.h"  // 緊急停止メッセージ
#include "delivery_robot/r_emergency_result.h"   // 緊急停止応答メッセージ

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Path.h>
#include "nav_msgs/OccupancyGrid.h"
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>

#include "delivery_robot/r_size.h"
#include "delivery_robot/r_navi_result.h"
#include "delivery_robot/r_navi_command.h"
#include "delivery_robot/r_info.h"
#include "delivery_robot/r_costmap.h"
#include "delivery_robot/r_corner.h"

// GridMapコスト
#define GRIDMAP_FREE_SPACE_COST		0
#define GRIDMAP_UNKNOWN_COST		-1

ros::Subscriber subs_original_costmap_;	//他ロボットの経路コストマップのサブスクライバ
ros::Subscriber subs_navi_cmd_res;	//navi_command応答メッセージ用サブスクライバ
ros::Subscriber subs_eme_cmd_res; //emergency command応答メッセージ用サブスクライバ
ros::Subscriber subs_robo_info;	//info用メッセージ用サブスクライバ
ros::Subscriber subs_robo_state; // stateメッセージ用サブスクライバ

ros::Publisher para_pub;
ros::Publisher para_emg;

nav_msgs::OccupancyGrid plan_costmap;	//他ロボット経路コストマップ

unsigned char cost_trans_table[101]; //コストマップへ反映するコストの変換テーブル
bool isRecvCostmap; //コストマップ受信フラグ
bool isRecvNaviCMDResult; //移動指示結果受信フラグ
bool isRecvEmgCMDResult;

std::string iso8601ex(void)
{
    int ch;
    char iso_time[40];
    char time_zone[10];
    char dest[70];
    struct timeval myTime; 
    struct tm *time_st;  
    
    memset(iso_time, 0, sizeof(iso_time));
    memset(time_zone, 0, sizeof(time_zone));
    memset(dest, 0, sizeof(dest));
    
    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);
    
    ch = strftime(iso_time, sizeof(iso_time)-1,"%FT%T", time_st);
    ch = strftime(time_zone, sizeof(time_zone)-1,"%z", time_st); 
    
    sprintf(dest, "%s.%03lu%s", iso_time, (myTime.tv_usec+500)/1000, time_zone);
    ROS_INFO("%s", dest);

    std::string time_str = dest;
//    ROS_INFO("%s", time_str.c_str());

    return( time_str );
}

void recvRobotState(const delivery_robot::r_state& msg)
{
    /*****************************
    * string id
    * string type
    * string time
    * string mode
    * string[] errors
    * r_pose pose
    * r_pose_optional destination
    * float64[36] covariance
    * r_battery battery
    *****************************/
    ROS_INFO("Subscribe state msg");
    ROS_INFO("id(%s) type(%s) time(%s) mode(%s) errors.size()(%d)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.mode.c_str(), (int)msg.errors.size());
    

    for(int i=0; i<msg.errors.size(); i++)
    {
        ROS_ERROR("errors[%d](%s)", i, msg.errors[i].c_str());
    }
    ROS_INFO("x(%f) y(%f) z(%f) roll(%f) pitch(%f) yaw(%f) volt(%f)", msg.pose.point.x, msg.pose.point.y, msg.pose.point.z, msg.pose.angle.roll, msg.pose.angle.pitch, msg.pose.angle.yaw, msg.battery.voltage);

    ROS_INFO("current_optional.valid(%s) current(%f)",(msg.battery.current_optional.valid == true)? "true":"false", msg.battery.current_optional.current); 

    return;
}

void recvinfo(const delivery_robot::r_info& msg)
{
    /***************************************
    * string id
    * string type
    * string time
    * r_size robot_size
    *   float64 robot_radius
    *   float64 inflation_radius
    *   r_corner[] footprint
    *     float64 x
    *     float64 y
    ***************************************/
    ROS_INFO("Subscribe info msg");
    ROS_INFO("id(%s) type(%s) time(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str());

    std::string str_param = "footprint :";
    
    for(int i = 0; i < msg.robot_size.footprint.size(); i++)
    {
        str_param = str_param + "[" + std::to_string(msg.robot_size.footprint[i].x) + "," +  std::to_string(msg.robot_size.footprint[i].y) + "] ";
    }
    
    
    ROS_INFO("robot_radius(%fl) inflation_radius(%fl) %s", msg.robot_size.robot_radius, msg.robot_size.inflation_radius, str_param.c_str());

    return;
}

void recvNaviCMDResult(const delivery_robot::r_navi_result& msg)
{
    /****************************************
    * string id
    * string type
    * string time
    * string received_time
    * string received_cmd
    * r_pose_optional received_destination
    *   geometry_msgs/Point point
    *       float64 x
    *       float64 z
    *       float64 y
    *   r_angle_optional angle_optional
    *       bool valid
    *       r_angle angle
    *           float64 roll
    *           float64 pitch
    *           float64 yaw
    * r_costmap received_costmap
    *   float64 resolution
    *   uint16 width
    *   uint16 height
    *   r_pose origin
    *       geometry_msgs/Point point
    *           float64 x
    *           float64 z
    *           float64 y
    *       r_angle angle
    *           float64 roll
    *           float64 pitch
    *           float64 yaw
    *   uint8[] cost_value
    * string result
    * string[] errors
    *****************************************/

    ROS_INFO("Subscribe navi_cmd_answer msg");
    ROS_INFO("id(%s) type(%s) time(%s) received_time(%s) received_cmd(%s) result(%s) errors.size()(%d)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.received_time.c_str(), msg.received_cmd.c_str(), msg.result.c_str(), (int)msg.errors.size());
    ROS_INFO("!!!!!!!!!!RESULT (%s)!!!!!!!!!!!!!!!",msg.result.c_str());
    for(int i=0; i<msg.errors.size(); i++){
        ROS_ERROR("errors[%d](%s)", i, msg.errors[i].c_str());
    }
    ROS_INFO("received_destination : x(%f) y(%f) z(%f) roll(%f) pitch(%f) yaw(%f)", msg.received_destination.point.x, msg.received_destination.point.y, msg.received_destination.point.z, msg.received_destination.angle_optional.angle.roll, msg.received_destination.angle_optional.angle.pitch, msg.received_destination.angle_optional.angle.yaw);
    
    // コストマップのデータチェック
    if( msg.received_costmap.cost_value.size() >= 1 )
    {
        unsigned int map_size = msg.received_costmap.width *  msg.received_costmap.height; //コストマップのサイズを求める
        bool isMatchData = true;
        unsigned int cost 				= 0;	//設定するコスト
        unsigned int cost_table_idx 	= 0;	//コスト変換テーブルのインデックス

        for(unsigned int idx = 0; idx <  map_size; idx++)
        {
            if(	plan_costmap.data[idx] != GRIDMAP_FREE_SPACE_COST && // FREE_SPACE = 0
                plan_costmap.data[idx] != GRIDMAP_UNKNOWN_COST)		 //UNKNOWN_COST = -1
            {
                cost_table_idx 	= plan_costmap.data[idx];
                cost 			= cost_trans_table[cost_table_idx];
                // 比較
                if(msg.received_costmap.cost_value[idx] != cost)
                { // コストが一致していない場合
                    isMatchData = false;
                    break;
                }
            }
        }
        if(isMatchData)
        {
            ROS_INFO("costmap data match!!!");
        }
        else
        {
            ROS_WARN("costmap data unmatch...");
        }
    }
    else
    {
        ROS_INFO("No costmap data");
    }

    isRecvNaviCMDResult = true;

    return;
}
void recvEmergencyCMDResult(const delivery_robot::r_emergency_result& msg)
{
    /****************************************
    * string id
    * string type
    * string time
    * string received_time
    * string received_emergency_cmd
    * string result
    * string[] errors
    *****************************************/
    ROS_INFO("Subscribe emergency_cmd_answer msg");
    ROS_INFO("id(%s) type(%s) time(%s) received_time(%s) received_emergency_cmd(%s) result(%s) errors.size()(%d)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.received_time.c_str(), msg.received_emergency_cmd.c_str(), msg.result.c_str(), (int)msg.errors.size());
    ROS_INFO("!!!!!!!!!!RESULT (%s)!!!!!!!!!!!!!!!",msg.result.c_str());
    for(int i=0; i<msg.errors.size(); i++){
        ROS_ERROR("errors[%d](%s)", i, msg.errors[i].c_str());
    }

    isRecvEmgCMDResult = true;

    return;
}

void recvCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("Subscribe plan_costmap msg");

    // クラスメンバ
    plan_costmap			= *msg;

    ROS_INFO("original costmap frame_id (%s)",	plan_costmap.header.frame_id.c_str());
    ROS_INFO("original costmap cell_x (%d)",	plan_costmap.info.width); //uint32 width
    ROS_INFO("original costmap cell_y (%d)",	plan_costmap.info.height); //uint32 height
    ROS_INFO("original costmap resolution (%fl)",plan_costmap.info.resolution); //float32 resolution
    ROS_INFO("original costmap position.x (%fl)", plan_costmap.info.origin.position.x); //float64 x
    ROS_INFO("original costmap position.y (%fl)", plan_costmap.info.origin.position.y); //float64 y
    ROS_INFO("original costmap position.z (%fl)", plan_costmap.info.origin.position.z);
    ROS_INFO("original costmap orientation.x (%fl)", plan_costmap.info.origin.orientation.x);
    ROS_INFO("original costmap orientation.y (%fl)", plan_costmap.info.origin.orientation.y);
    ROS_INFO("original costmap orientation.z (%fl)", plan_costmap.info.origin.orientation.z);
    ROS_INFO("original costmap orientation.w (%fl)", plan_costmap.info.origin.orientation.w);

 
    isRecvCostmap = true; // コストマップ受信フラグON

    return;
}

int main(int argc, char **argv)
{
  //Publisherとしての定義 
  // n.advertise<comp_tutorial::adder>("para_input", 1000);
  // comp_tutorial::adder型のメッセージをpara_inputというトピックへ配信する
  //"1000"はトピックキューの最大値

    int mode = 0;
    std::string entity_id = "default_id";
    std::string entity_type = "default_type";
    std::string str_mode = "mode_no";

    ROS_INFO("argc=%i" , argc ); 

    memset( &cost_trans_table, 0, sizeof(cost_trans_table));
    cost_trans_table[0] = 0;  // NO obstacle
    cost_trans_table[99] = 253;  // INSCRIBED obstacle
    cost_trans_table[100] = 254;  // LETHAL obstacle

    for (int i = 1; i < 99; i++)
    {
        cost_trans_table[ i ] = u_char(1 + (251 * (i - 1)) / 97); // ceil関数で小数点以下繰り上げし、1~252→1~98へ変換
        // ROS_INFO("index%d : %d", i, cost_trans_table[i]);
    }
    if( argc >= 2 ) entity_id = argv[1];
    if( argc >= 3 ) entity_type = argv[2];
    if( argc >= 4 ) mode = atoi(argv[3]);
    if( argc >= 4 ) str_mode = argv[3];

    ROS_INFO("entity_id:%s", entity_id.c_str());
    ROS_INFO("entity_type:%s", entity_type.c_str());
    ROS_INFO("mode:%i", mode ); 

    // 初期化のためのAPI
    ros::init(argc, argv, "edge" + entity_id + str_mode);

    // ノードハンドラの宣言
    ros::NodeHandle n;

    // パブリッシャ
    para_pub = n.advertise<delivery_robot::r_navi_command>("/robot_bridge/"+ entity_id +"/navi_cmd", 100, true);
    para_emg = n.advertise<delivery_robot::r_emergency_command>("/robot_bridge/"+ entity_id +"/emg", 100, true);

    // サブスクライバ
    subs_original_costmap_  = n.subscribe("/" + entity_id + "/map_original", 10, &recvCostmap);	                // map_serverからのoriginal_costmapトピックメッセージのサブスクライバ
    subs_navi_cmd_res       = n.subscribe("/robot_bridge/"+ entity_id +"/navi_cmdexe", 10, &recvNaviCMDResult);	// naviコマンドの応答メッセージサブスクライバ
    subs_eme_cmd_res        = n.subscribe("/robot_bridge/"+ entity_id +"/emgexe", 10, &recvEmergencyCMDResult);	// emergencyコマンドの応答メッセージサブスクライバ
    subs_robo_info          = n.subscribe("/robot_bridge/"+ entity_id +"/robo_info", 10, &recvinfo);	        // infoメッセージのサブスクライバ
    subs_robo_state         = n.subscribe("/robot_bridge/"+ entity_id +"/state", 100, &recvRobotState);        // stateメッセージのサブスクライバ


// tst
    std::string tst = "/robot_bridge/"+ entity_id +"/navi_cmd";
    ROS_INFO("tst:%s", tst.c_str()); 

    //delivery_robot::r_req型のオブジェクトを定義
    delivery_robot::r_navi_command msg;
    delivery_robot::r_emergency_command emg_msg; // 緊急停止コマンド

    msg.id = entity_id;
    msg.type = entity_type;
    msg.time = iso8601ex();

    if(mode == 999 || mode == 888 || mode == 777){
        // 緊急停止コマンド
        emg_msg.id   = msg.id;
        emg_msg.type = msg.type;
        emg_msg.time = msg.time;
        if(mode == 999)
        {
           emg_msg.emergency_cmd = "stop";
        }
        else if(mode == 888)
        {
           emg_msg.emergency_cmd = "suspend";
        }
        else
        {
           emg_msg.emergency_cmd = "resume";
        }

        ROS_INFO("Emergency_command id(%s) type(%s) time(%s) cmd(%s)", emg_msg.id.c_str(), emg_msg.type.c_str(), emg_msg.time.c_str(), emg_msg.emergency_cmd.c_str() );
        para_emg.publish(emg_msg);

        // 緊急停止コマンド結果受信処理
        isRecvEmgCMDResult = false;
        ros::Rate loop_rate(20);
        while ( ros::ok() && !isRecvEmgCMDResult)
        {
            ROS_INFO("Waiting for emergency command result...");
            loop_rate.sleep();// スリープ
            ros::spinOnce();// コールバック処理
        }
    }
    else
    {

        // 配信用コストマップ受信処理
        isRecvCostmap = false;
        ros::Rate loop_rate(20);
        while ( ros::ok() && !isRecvCostmap)
        {
            loop_rate.sleep();// スリープ
            ros::spinOnce();// コールバック処理
        }

        
        // 目的地格納処理
        switch(mode)
        {
#if 0 //20201130テストコード
        //①ステータス:standby　cmd：navi
            // １，コストマップ正常(経路コストマップ投げる)
            case 1:// standby中にnaviコマンド送信
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;
                break;
            // ２，コストマップ異常(空のコストマップを投げる)
            case 2:// standby中にnaviコマンド送信
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;

                isRecvCostmap = false;
                break;
        //②ステータス：navi，suspend　cmd：navi
            case 3: //navi中にnaviコマンド送信
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;
                break;

            case 4: //suspend中にnaviコマンド送信
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;
                break;


            case 5: //navi中にnaviコマンド送信(コストマップ異常)
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;

                isRecvCostmap = false;

                break;
            case 6: //suspend中にnaviコマンド送信(コストマップ異常)
                msg.cmd = "navi";

                msg.destination.point.x =  0.907;    // ②tb3_1初期位置（右向き）
                msg.destination.point.y =  2.081;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw =-1.57;

                isRecvCostmap = false;
                break;

        // ③ステータス：navi，suspend　cmd：refresh
            case 7: //navi中にnaviコマンド送信
                msg.cmd = "refresh";

                msg.destination.point.x = -0.986;    // ①tb3_0初期位置（左向き）
                msg.destination.point.y = -1.900;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw = 1.57;
                break;

            case 8: //suspend中にrefreshコマンド送信
                msg.cmd = "refresh";

                msg.destination.point.x = -0.986;    // ①tb3_0初期位置（左向き）
                msg.destination.point.y = -1.900;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw = 1.57;
                break;

            case 9: //navi中にrefreshコマンド送信(コストマップ異常)
                msg.cmd = "refresh";

                msg.destination.point.x = -0.986;    // ①tb3_0初期位置（左向き）
                msg.destination.point.y = -1.900;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw = 1.57;

                isRecvCostmap = false;
                break;

            case 10: //suspend中にrefreshコマンド送信(コストマップ異常)
                msg.cmd = "refresh";

                msg.destination.point.x = -0.986;    // ①tb3_0初期位置（左向き）
                msg.destination.point.y = -1.900;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw = 1.57;

                isRecvCostmap = false;
                break;

        // ④ステータス：navi，suspend　cmd：standby
            case 11: //navi中にstandbyコマンド送
            case 12: //suspend中にstandbyコマンド送信
                msg.cmd = "standby";
                break;

            case 13: //navi中にstandbyコマンド送信(コストマップ異常)
            case 14: //suspend中にstandbyコマンド送信(コストマップ異常)
                msg.cmd = "standby";
                isRecvCostmap = false;
                break;
            case 111:
                msg.cmd = "navi";

                msg.destination.point.x = -0.986;    // ①tb3_0初期位置（左向き）
                msg.destination.point.y = -1.900;
                msg.destination.point.z =  0.0;
                msg.destination.angle_optional.valid = true;
                msg.destination.angle_optional.angle.yaw = 1.57;
                break;  
#endif
            case 401:// navi tb3_stage_4 テストケース２:
            msg.cmd = "navi";

            msg.destination.point.x = -0.986;    // ①右下（左向き）
            msg.destination.point.y = -1.900;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 1.57;
            break;

        case 402:// navi tb3_stage_4 テストケース２:
            msg.cmd = "navi";

            msg.destination.point.x =  0.907;    // ②左上（右向き）
            msg.destination.point.y =  2.081;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =-1.57;
            break;

        case 403:// navi tb3_stage_4 テストケース２:
            msg.cmd = "navi";

            msg.destination.point.x =  1.862;    // ③左上（右向き）
            msg.destination.point.y =  1.429;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =-1.57;
            break;

        // 20200925 LICTiA環境
        case 151:// navi   A	受付前
            msg.cmd = "navi";
            
            msg.destination.point.x =  0.0;
            msg.destination.point.y =  0.0;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 0.0;
            break;

        case 152:// navi   B	ロッカー前１
            msg.cmd = "navi";
            
            msg.destination.point.x = -2.100;
            msg.destination.point.y =  0.550;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -1.57;
            break;

        case 153:// navi   C	ロッカー前２
            msg.cmd = "navi";
            
            msg.destination.point.x = -2.900;
            msg.destination.point.y =  0.550;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -1.57;
            break;

        case 154:// navi   D	事務室前通路
            msg.cmd = "navi";
            
            msg.destination.point.x =  6.804;
            msg.destination.point.y = -2.221;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  3.14;
            break;

        case 155:// navi   E	個室前
            msg.cmd = "navi";
            
            msg.destination.point.x =  1.961;
            msg.destination.point.y = -9.877;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.57;
            
            break;

        case 156:// navi   F	会議室３手前
            msg.cmd = "navi";
            
            msg.destination.point.x =  5.938;
            msg.destination.point.y = -8.118;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 1.57;
            break;

        case 157:// navi   G	会議室３奥
            msg.cmd = "navi";
            
            msg.destination.point.x =  5.882;
            msg.destination.point.y = -9.302;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.57;
            break;

        case 158:// navi   H	会議室２手前
            msg.cmd = "navi";
            
            msg.destination.point.x =  9.38;
            msg.destination.point.y = -8.146;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.57;
            break;

        case 159:// navi   I	会議室２奥
            msg.cmd = "navi";
            
            msg.destination.point.x =  9.212;
            msg.destination.point.y = -9.438;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.205;
            break;

        case 160:// navi   J	会議室１手前
            msg.cmd = "navi";
            
            msg.destination.point.x =  10.195;
            msg.destination.point.y = -8.21;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.928;
            break;

        case 161:// navi   K	会議室１奥
            msg.cmd = "navi";
            
            msg.destination.point.x =  10.622;
            msg.destination.point.y =  -9.442;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.915;
            break;

        // 20201001 LICTiA環境

        case 163:// navi   	広場
            msg.cmd = "navi";
            
            msg.destination.point.x = -2.835;
            msg.destination.point.y = -10.278;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 1.57;
            
            break;

        case 164:// navi   	会議室３中
            msg.cmd = "navi";
            
            msg.destination.point.x =  4.771;
            msg.destination.point.y = -11.274;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.57;
            
            break;

        case 165:// navi   	会議室２中
            msg.cmd = "navi";
            
            msg.destination.point.x =  7.396;
            msg.destination.point.y = -9.495;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  0.00;
            
            break;

        case 166:// navi   	会議室１中
            msg.cmd = "navi";
            
            msg.destination.point.x =  10.362;
            msg.destination.point.y = -12.051;
            msg.destination.point.z =  0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw =  1.57;
            
            break;


    //社内テスト用コード開始（2020/09/30実施）
        case 3:// 初期位置  
            msg.cmd = "navi";

            
            msg.destination.point.x = 0.00;
            msg.destination.point.y = 0.00;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 0.000;
            break;

        case 4:// シュレッダー前  
            msg.cmd = "navi";

            
            msg.destination.point.x = -1.501;
            msg.destination.point.y = -1.570;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 0.000;
            break;

        case 5:// 本棚付近  
            msg.cmd = "navi";

            
            msg.destination.point.x = -0.643;
            msg.destination.point.y = 1.343;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -1.564;
            break;

        case 6:// 課長席中央  
            msg.cmd = "navi";

            
            msg.destination.point.x = 5.398;
            msg.destination.point.y = 0.111;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -3.141;
            break;

        case 7:// 会議室
            msg.cmd = "navi";

            
            msg.destination.point.x = -1.806;
            msg.destination.point.y = -0.064;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -0.373;
            break;
        case 8:// 卓球台横
            msg.cmd = "navi";

            
            msg.destination.point.x = -0.156;
            msg.destination.point.y = -1.824;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 1.809;
            break;
        case 9:// コピー機前
            msg.cmd = "navi";

            
            msg.destination.point.x = -3.067;
            msg.destination.point.y = -2.097;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = 1.570;
            break;
        case 11:// 課長席中央  
            msg.cmd = "navi";

            msg.destination.point.x = 5.398;
            msg.destination.point.y = 0.111;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -3.141;
            isRecvCostmap = false;
            {
                ROS_INFO("create costmap data...");
                unsigned int map_size = plan_costmap.info.width * plan_costmap.info.height; //コストマップのサイズを求める
                unsigned int cost 				= 0;	//設定するコスト
                unsigned int cost_table_idx 	= 0;	//コスト変換テーブルのインデックス

                msg.costmap.resolution = 11;
                msg.costmap.width  = 21;
                msg.costmap.height = 2;
                msg.costmap.origin.point.x = 3;
                msg.costmap.origin.point.y = 1;
                
                msg.costmap.cost_value.resize(map_size);

                for(unsigned int idx = 0; idx <  map_size; idx++)
                {
                    if(	plan_costmap.data[idx] != GRIDMAP_FREE_SPACE_COST && // FREE_SPACE = 0
                    plan_costmap.data[idx] != GRIDMAP_UNKNOWN_COST)		 //UNKNOWN_COST = -1
                    {
                        cost_table_idx 	= plan_costmap.data[idx];
                        cost 			= cost_trans_table[cost_table_idx];
                        msg.costmap.cost_value[idx] = cost;
                    }
                }
            }
            break;
        case 10:// 課長席中央  
            msg.cmd = "refresh";


            msg.destination.point.x = 5.398;
            msg.destination.point.y = 0.111;
            msg.destination.point.z = 0.0;
            msg.destination.angle_optional.valid = true;
            msg.destination.angle_optional.angle.yaw = -3.141;
            isRecvCostmap = false;
            {
                ROS_INFO("create costmap data...");
                unsigned int map_size = plan_costmap.info.width * plan_costmap.info.height; //コストマップのサイズを求める
                unsigned int cost 				= 0;	//設定するコスト
                unsigned int cost_table_idx 	= 0;	//コスト変換テーブルのインデックス
                
                msg.costmap.resolution = 11;
                msg.costmap.width  = 21;
                msg.costmap.height = 2;
                msg.costmap.origin.point.x = 3;
                msg.costmap.origin.point.y = 1;

                msg.costmap.cost_value.resize(map_size);

                for(unsigned int idx = 0; idx <  map_size; idx++)
                {
                    if(	plan_costmap.data[idx] != GRIDMAP_FREE_SPACE_COST && // FREE_SPACE = 0
                    plan_costmap.data[idx] != GRIDMAP_UNKNOWN_COST)		 //UNKNOWN_COST = -1
                    {
                        cost_table_idx 	= plan_costmap.data[idx];
                        cost 			= cost_trans_table[cost_table_idx];
                        msg.costmap.cost_value[idx] = cost;
                    }
                }
            }
            break;



            default:// standby
                msg.cmd = "standby";            
                break;
        }

        // コストマップ格納処理
        if(isRecvCostmap)
        {
            ROS_INFO("Create costmap data...");
            unsigned int map_size = plan_costmap.info.width * plan_costmap.info.height; //コストマップのサイズを求める
            unsigned int cost 				= 0;	//設定するコスト
            unsigned int cost_table_idx 	= 0;	//コスト変換テーブルのインデックス

            msg.costmap.resolution = plan_costmap.info.resolution;
            msg.costmap.width  = plan_costmap.info.width;
            msg.costmap.height = plan_costmap.info.height;
            msg.costmap.origin.point.x = plan_costmap.info.origin.position.x;
            msg.costmap.origin.point.y = plan_costmap.info.origin.position.y;
            
            msg.costmap.cost_value.resize(map_size);

            for(unsigned int idx = 0; idx <  map_size; idx++)
            {
                if(	plan_costmap.data[idx] != GRIDMAP_FREE_SPACE_COST && // FREE_SPACE = 0
                    plan_costmap.data[idx] != GRIDMAP_UNKNOWN_COST)		 //UNKNOWN_COST = -1
                {
                    cost_table_idx 	= plan_costmap.data[idx];
                    cost 			= cost_trans_table[cost_table_idx];
                    msg.costmap.cost_value[idx] = cost;
                }
            }
        }

        
        ROS_INFO("id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.cmd.c_str() );

        ROS_INFO("x(%f) y(%f) z(%f)", msg.destination.point.x, msg.destination.point.y, msg.destination.point.z);
        
        para_pub.publish(msg);//PublishのAPI
        

        // 移動指示結果受信処理
        isRecvNaviCMDResult = false;
        while ( ros::ok() && !isRecvNaviCMDResult)
        {
            ROS_INFO("Waiting for navi command result...");
            loop_rate.sleep();// スリープ
            ros::spinOnce();// コールバック処理
        }
    }

  return 0;
}




