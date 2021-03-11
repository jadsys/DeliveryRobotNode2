/*
    ロボットノード

トピック名称 メモ 
    ENTITY_IDは、ロボットごとに定めるユニークな文字列
> 　①ロボットへの移動指示
> 　　/robot_bridge/$(arg ENTITY_ID)/navi_cmd
> 　②移動指示受信結果
> 　　/robot_bridge/$(arg ENTITY_ID)/navi_cmdexe
> 　③ロボットの状態報告
> 　　/robot_bridge/$(arg ENTITY_ID)/state
> 　④ロボットへの緊急停止
> 　　/robot_bridge/$(arg ENTITY_ID)/emg
> 　⑤緊急停止受信結果
> 　　/robot_bridge/$(arg ENTITY_ID)/emgexe
>   ⑥ロボットの情報通知
>     /robot_bridge/$(arg ENTITY_ID)/robo_info

*/

#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>// 初期位置トピックの型 https://demura.net/lecture/14011.html
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/BatteryState.h> //バッテリーステータス     TB3
#include <std_msgs/Int16MultiArray.h> //バッテリーステータス     メガローバ
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include "RobotDriver.cpp" // ロボット制御
#include "delivery_robot/r_state.h"   // 状態報告メッセージ
#include "delivery_robot/r_emergency_command.h"  // 緊急停止メッセージ
#include "delivery_robot/r_emergency_result.h"   // 緊急停止応答メッセージ

// 2020/09/28追加
#include "delivery_robot/r_size.h"
#include "delivery_robot/r_navi_result.h"  // 移動指示応答メッセージ
#include "delivery_robot/r_navi_command.h" // 移動指示メッセージ
#include "delivery_robot/r_info.h"
#include "delivery_robot/r_costmap.h"
#include "delivery_robot/r_corner.h"


//  MODE種別
#define     MODE_STANDBY        "standby"
#define     MODE_NAVI           "navi"
#define     MODE_SUSPEND        "suspend"
#define     MODE_ERROR          "error"
//  COMMAND種別
#define     CMD_NAVI            "navi"
#define     CMD_REFRESH         "refresh"
#define     CMD_STANDBY         "standby"
//  RESULT種別
#define     RESULT_ACK          "ack"
#define     RESULT_IGNORE       "ignore"
#define     RESULT_ERROR        "error"
// ID,TYPE
#define     DEFAULT_ROBOT_ID    "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE  "turtlebot"
// エマージェンシーコマンド
#define     EMERGENCY_STOP      "stop"
#define     EMERGENCY_SUSPEND   "suspend"
#define     EMERGENCY_RESUME    "resume"
// move base ステータスインデックス
#define     MOVE_BASE_INIT      -1
#define     MOVE_BASE_PENDING   0
#define     MOVE_BASE_ACTIVE    1 
#define     MOVE_BASE_SUCCEEDED 3 
#define     MOVE_BASE_ABORTED   4 

// Queue size(最新のデータが欲しい場合は小さく，取りこぼしたくない場合は大きくする)
#define     ROS_QUEUE_SIZE_5    5
#define		ROS_QUEUE_SIZE_10	10
#define		ROS_QUEUE_SIZE_100	100

// メガローバ 電圧係数
#define     MR_VOLTAGE_FACTOR   137.8

// コスト定義
#define     UNKNOWN_COST_GRIDMAP                -1
#define     FREESPACE_COST_GRIDMAP              0
#define     INSCRIBED_COST_GRIDMAP              99
#define     OBSTACLE_COST_GRIDMAP               100
#define     STUCK_AVOID_COST                    51
#define     OBSTACLE_COST                       254

// 動作周波数
#define     ROS_RATE_10HZ       10.0
#define     ROS_RATE_20HZ       20.0
#define     ROS_RATE_30HZ       30.0

// 時間定義
#define     ROS_TIME_0S         0
#define     ROS_TIME_50MS       0.05
#define     ROS_TIME_500MS      0.5
#define     ROS_TIME_1S         1.0
#define     ROS_TIME_3S         3.0
#define     ROS_TIME_5S         5.0

// 角度（Degree）定義
#define     ANGLE_OF_0_DEGREES      0
#define     ANGLE_OF_3_DEGREES      3.0
#define     ANGLE_OF_360_DEGREES    360

// 更新済みコストの差分のしきい値
#define     DIFFERENCIAL_COST_THRESHOLD 30

typedef struct DestinationPoint 
{
    double x;
    double y;

}stExclusionPoint;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


    void waypointCopy(const delivery_robot::r_pose_optional waypointOrg, delivery_robot::r_pose_optional &waypoint)
    {
        waypoint.point.x = waypointOrg.point.x;
        waypoint.point.y = waypointOrg.point.y;
        waypoint.point.z = waypointOrg.point.z;

        waypoint.angle_optional.valid       = waypointOrg.angle_optional.valid; 
        waypoint.angle_optional.angle.roll  = waypointOrg.angle_optional.angle.roll; 
        waypoint.angle_optional.angle.pitch = waypointOrg.angle_optional.angle.pitch;
        waypoint.angle_optional.angle.yaw   = waypointOrg.angle_optional.angle.yaw;

        return;
    }

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
        //  ROS_INFO("%s", dest);

        std::string time_str = dest;
        //  ROS_INFO("%s", time_str.c_str());

        return( time_str );
    }

    void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    }

    geometry_msgs::Quaternion GetQuaternionMsg(double roll, double pitch, double yaw )
    {
        geometry_msgs::Quaternion q;
        tf::Quaternion quat =tf::createQuaternionFromRPY(roll,pitch,yaw);
        quaternionTFToMsg( quat, q);

        return(q); 
    }

class RobotNode
{
private:

    RobotDriver *_driver;

    // パブ
    ros::Publisher pub_initial;
    ros::Publisher pub_cancel;          // move_baseキャンセル送信用パブリッシャ
    ros::Publisher pub_goal;            // 目標地点地点の送信用パブリッシャ
    ros::Publisher pub_robot_sts;       // ロボットのステータス通知用パブリッシャ
    ros::Publisher pub_answer;          // 移動指示受信応答用パブリッシャ
    ros::Publisher pub_emergency_ans;   // 緊急停止指示受信応答用パブリッシャ
    ros::Publisher pub_plan_costmap;    // 他ロボットの経路コストマップのパブリッシャ
    ros::Publisher pub_info;            // ロボットの情報通知用パブリッシャ

    // サブ
    ros::Subscriber sub_move_base_status;   // move_baseのステータス情報受信用サブスクライバ
    ros::Subscriber sub_command_recv;       // 移動指示コマンド受信用サブスクライバ
    ros::Subscriber sub_battery_state_recv; // バッテリー情報受信用サブスクライバ
    ros::Subscriber sub_amclpose_recv;      // amcl_pose受信受信用サブスクライバ
    ros::Subscriber sub_emergency_recv;     // 緊急停止指示受信用サブスクライバ
    ros::Subscriber sub_sociomap;           // ソシオ地図サブスクライバ（2020/10/13追加）

    ros::Timer      timer;
    ros::Timer      stuck_timer;	//stuckチェック用(2020/11/26追加)
    ros::WallTimer  goal_timer;

    tf::TransformListener tfl;
    geometry_msgs::PoseWithCovarianceStamped _initial_pose;          // 初期位置
    geometry_msgs::Point                     _Past_Position;         // 過去の位置(2020/11/26追加)
    delivery_robot::r_pose_optional          _current_destination;   // 現在の目的地(角度情報あり)
    delivery_robot::r_costmap                _navi_cmd_costmap;      // 移動指示コマンドのコストマップデータ(2020/11/26追加)
 
    std::string _mode_status;   // 現在のmode保持
    std::string _entityId;      // ロボットのユニークID
    std::string _entity_type;   // ロボットの種別の識別子

    std::vector<stExclusionPoint> _exclusion_range_coordinate_list;   // 旋回対象外座標範囲リスト
    std::vector<delivery_robot::r_corner> _footprint;               // footprint情報(2020/09/28追加)
    std::list<delivery_robot::r_pose_optional> _destinations;       // 目的値リスト


    bool _navi_flg;                 // 自動走行中かを判定
    bool _calibration_flg;          // キャリブレーション中かを判定
    bool _turn_busy_flg;            // 旋回中かを判定
    bool _navi_node;                // 自作ナビノードを使用するかどうか
    bool _goal_allowable_flg;       // goalポイント許容範囲圏内通知フラグ
    bool _update_current_destination; // 目的地更新フラグ(2020/10/05追加)
    bool _is_pub_ori_plan_costmap;  // オリジナルの経路コストマップはパブリッシュ済みか(2020/11/26追加)
    char _cost_trans_table[256];    // コストの変換テーブル
    int8_t _replacing_cost;         // 送信するコストマップのコスト値
    int _move_base_sts;             // movebaseがゴールに着いたかを受信する
    int _move_base_status_id;       // move_baseのステータス値(2020/10/05追加)
    unsigned int _sociomap_width;   // ソシオ地図の幅(2020/10/13追加)
    unsigned int _sociomap_height;  // ソシオ地図の幅(2020/10/13追加)
    float _volt_sts;                // バッテリー電圧値
    double _wp_sleep_time;          // wayポイント停止時間
    double _goal_tolerance_range;   // goalポイント許容範囲
    double _goal_allowable_range;   // ゴール地点到達時のタイムアウトタイマー開始半径
    double _goal_allowable_time;    // ゴール地点到達時のタイムアウト時間
    double _goal_allowable_angle;   // ゴール地点到達時のangle許容角度
    double _g_covariance[36];       // 共分散値
    double _robot_radius;           // ロボットの幅(2020/09/28追加)
    double _inflation_raidus;       // マージンを考慮したロボットの幅(2020/09/28追加)
    double _navigation_turn_speed;  // 目的地への旋回速度(2020/10/13追加)
    double _sociomap_origin_x;      // ソシオ地図の原点座標(2020/10/13追加)
    double _sociomap_origin_y;      // ソシオ地図の原点座標(2020/10/13追加)
    double _sociomap_resolution;    // ソシオ地図の解像度(2020/10/13追加)
    double _stuck_check_time;       // スタックチェックタイマー処理の時間間隔(2020/11/26追加)
    double _stuck_threshold_length; // ロボットがスタックしていると判定する距離(2020/11/26追加)

public:
    //------------------------------------------------------------------------------
    //  コンストラクタ
    //------------------------------------------------------------------------------
    RobotNode(RobotDriver& driver)
    {
        _driver = &driver;

        _volt_sts = -FLT_MAX;

        _mode_status = MODE_STANDBY;
        _navi_flg = false;
        _calibration_flg = true;
        _move_base_sts = MOVE_BASE_PENDING;
        _goal_allowable_flg = false;
        _turn_busy_flg = false;
        _is_pub_ori_plan_costmap = false;

        // 共分散値
        memset( &_g_covariance, 0, sizeof(_g_covariance)); 
        _g_covariance[0] = 0.25;
        _g_covariance[7] = 0.25;
        _g_covariance[35] = 0.06853891945200942;

		//コストマップテーブル作成
        memset( &_cost_trans_table, 0, sizeof(_cost_trans_table));
        _cost_trans_table[0]   = FREESPACE_COST_GRIDMAP;    // NO obstacle
        _cost_trans_table[253] = INSCRIBED_COST_GRIDMAP;    // INSCRIBED obstacle
        _cost_trans_table[254] = OBSTACLE_COST_GRIDMAP;     // LETHAL obstacle
        _cost_trans_table[255] = UNKNOWN_COST_GRIDMAP;      // UNKNOWN

        for (int i = 1; i < 253; i++)
        {
            _cost_trans_table[ i ] = char(1 + (97 * (i - 1)) / 251); // 1~252→1~98
        }

        // 2020/10/05追加
        // move_baseのステータス値初期化
        _move_base_status_id    = -1;

        // 2020/10/12追加
        // ソシオ地図情報の保持変数を初期化
        _sociomap_width         = 0;
        _sociomap_height        = 0;
        _sociomap_resolution    = 0.0; 
        _sociomap_origin_x      = 0.0;
        _sociomap_origin_y      = 0.0;

        // 2020/12/04追加
        // 置き換えるコストの初期化
        _replacing_cost = STUCK_AVOID_COST;

    }

    //------------------------------------------------------------------------------
    //  デストラクタ
    //------------------------------------------------------------------------------
    ~RobotNode()
    {
		// do nothing
    }

    //------------------------------------------------------------------------------
    //  初期設定
    //------------------------------------------------------------------------------
    bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
    {

        // ロボットのユニークID読み込み
        if (privateNode.getParam("entity_id", _entityId))
        {
            ROS_INFO("entity_id (%s)", _entityId.c_str());
        }
        else
        {
            _entityId = DEFAULT_ROBOT_ID;
        }

        // ロボットの種別の識別子読み込み
        if (privateNode.getParam("entity_type", _entity_type))
        {
            ROS_INFO("entity_type (%s)", _entity_type.c_str());
        }
        else
        {
            _entity_type = DEFAULT_ROBOT_TYPE;
        }
        
        // naviノード使用の可否読み込み
        if (privateNode.getParam("navi_node", _navi_node))
        {
            ;
        }
        else
        {
            _navi_node = false;
        }

        if( _navi_node==true )
        {
            ROS_INFO("_navi_node==true");
        }
        else
        {
            ROS_INFO("_navi_node==false");
        }

        // wayポイント停止時間の読み込み
        if (privateNode.getParam("wp_sleep_time", _wp_sleep_time))
        {
            ROS_INFO("wp_sleep_time (%lf)", _wp_sleep_time);
        }
        else
        {
            _wp_sleep_time = 5;//[sec]
        }

        // goalポイント許容範囲の読み込み
        if (privateNode.getParam("goal_tolerance_range", _goal_tolerance_range))
        {
            ROS_INFO("goal_tolerance_range (%lf)", _goal_tolerance_range);
        }
        else
        {
            _goal_tolerance_range = 0.05;//[m]
        }

        // ゴール地点到達時のタイムアウトタイマー開始半径の読み込み
        if (privateNode.getParam("goal_allowable_range", _goal_allowable_range))
        {
            ROS_INFO("goal_allowable_range (%lf)", _goal_allowable_range);
        }
        else
        {
            _goal_allowable_range = 0.20;//[m]
        }

        // ゴール地点到達時のタイムアウト時間の読み込み
        if (privateNode.getParam("goal_allowable_time", _goal_allowable_time))
        {
            ROS_INFO("goal_allowable_time (%lf)", _goal_allowable_time);
        }
        else
        {
            _goal_allowable_time = 4;//[sec]
        }

        // ゴール地点のangleの許容角度
        if (privateNode.getParam("goal_allowable_angle", _goal_allowable_angle))
        {
            ROS_INFO("goal_allowable_angle (%lf)", _goal_allowable_angle);
        }
        else
        {
            _goal_allowable_angle = 0.0349066;  // 2度
        }
        // スタック頻度チェック間隔時間
        if (privateNode.getParam("stuck_check_time", _stuck_check_time))
        {
            ROS_INFO("stuck_check_time (%lf)", _stuck_check_time);
        }
        else
        {
            _stuck_check_time = 10;  // 10秒
        }
        // スタック判定の距離の閾値
        if (privateNode.getParam("stuck_threshold_length", _stuck_threshold_length))
        {
            ROS_INFO("stuck_threshold_length (%lf)", _stuck_threshold_length);
        }
        else
        {
            _stuck_threshold_length = 0.1;  // 半径10cm
        }

        // 2020/09/28追加
        // ロボットのfootprint情報読み込み
        delivery_robot::r_corner footprint_;
        XmlRpc::XmlRpcValue footprint_param_member;
        bool isReadParam = true;
        if (privateNode.getParam("footprint", footprint_param_member))
        {
            if(footprint_param_member.getType() != XmlRpc::XmlRpcValue::TypeArray) //footprint情報が配列で与えられているか
            {
                isReadParam = false;
            }
            ROS_INFO("footprint size: %i", (int)footprint_param_member.size());
            for (int32_t i = 0; i < footprint_param_member.size(); ++i)
            {
                if (!footprint_param_member[i]["x"].valid() || !footprint_param_member[i]["y"].valid())
                {
                    ROS_WARN("Not found footprint param \"x\" or \"y\" ");
                    isReadParam = false;
                    break;
                }
                if (footprint_param_member[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) //footprint情報はDouble形式か
                {
                    footprint_.x = static_cast<double>(footprint_param_member[i]["x"]);
                }
                else
                {
                    isReadParam = false;
                }
                if (footprint_param_member[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    footprint_.y = static_cast<double>(footprint_param_member[i]["y"]);
                }
                else
                {
                    isReadParam = false;
                }
                ROS_INFO("[%i] x: %fl, y: %fl", i, footprint_.x, footprint_.y);
                _footprint.push_back(footprint_);
            }
        }
        else
        {
            ROS_INFO("No parameters were found for footprint");
            isReadParam = false;
        }
        // デフォルトパラメータの読み込み
        if(!isReadParam)   
        { //パラメータ読み込み失敗
            double footprint_def[4][2]  = {{-0.205, -0.155}, {-0.205, 0.155}, {0.077, 0.0155}, {0.077, -0.0155}};
            for(int i=0; i<4; i++)
            {
                footprint_.x = footprint_def[i][0];
                footprint_.y = footprint_def[i][1];
                _footprint.push_back(footprint_);
            }
        }

        // ロボットの半径の読み込み
        if (privateNode.getParam("robot_radius", _robot_radius))
        {
            ROS_INFO("robot_radius (%lf)", _robot_radius);
        }
        else
        {
            _robot_radius = 0.3;  // 30[cm]
        }

        // ロボットのマージン込み半径の読み込み
        if (privateNode.getParam("inflation_raidus", _inflation_raidus))
        {
            ROS_INFO("inflation_raidus (%lf)", _inflation_raidus);
        }
        else
        {
            _inflation_raidus = 0.4;  // 40[cm]
        }
        
        // 目的地への旋回速度読み込み
        if (privateNode.getParam("navigation_turn_speed", _navigation_turn_speed))
        {
            ROS_INFO("navigation_turn_speed (%f)", _navigation_turn_speed);
        }
        else
        {
            _navigation_turn_speed = 0.4;  // 0.4[rad/sec]
        }

        stExclusionPoint exclusion_range_coordinate; // 旋回除外範囲の座標
        XmlRpc::XmlRpcValue exclusion_range_member;
        isReadParam = true; // パラメータ読み込み成功フラグ
        if (privateNode.getParam("turn_control_exclusion_range", exclusion_range_member)) //パラメータ取得
        {
            if(exclusion_range_member.getType() != XmlRpc::XmlRpcValue::TypeArray) //exclusion_range_coordinate情報が配列で与えられているか
            {
                isReadParam = false;
            }

            ROS_INFO("turn_control_exclusion_range member size: %i", (int)exclusion_range_member.size());

            for (int32_t i = 0; i < exclusion_range_member.size(); ++i)
            {
                // パラメータが存在するかチェック
                if (!exclusion_range_member[i]["x"].valid() || !exclusion_range_member[i]["y"].valid())
                {
                    ROS_WARN("Not found exclusion_range_member param \"x\" or \"y\" ");
                    isReadParam = false;
                    break;
                }
                // パラメータの型のチェック
                if (exclusion_range_member[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) //footprint情報はDouble形式か
                { // 型が一致している場合
                    exclusion_range_coordinate.x = static_cast<double>(exclusion_range_member[i]["x"]);
                }
                else
                {
                    isReadParam = false;
                    break;
                }
                if (exclusion_range_member[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    exclusion_range_coordinate.y = static_cast<double>(exclusion_range_member[i]["y"]);
                }
                else
                {
                    isReadParam = false;
                    break;
                }

                ROS_INFO("[%i] x: %fl, y: %fl", i, exclusion_range_coordinate.x, exclusion_range_coordinate.y);
                _exclusion_range_coordinate_list.push_back(exclusion_range_coordinate);
            }
        }
        else
        { //パラメータ取得できなかった場合
            ROS_INFO("No parameters were found for exclusion_range_coordinate");
            isReadParam = false;
        }
        // デフォルトパラメータの読み込み
        if(!isReadParam)   
        { 
            stExclusionPoint def_exclusion_range_coordinate[] = {{ 14.624, -8.920}, {14.624, -13.690}, {0.863, -8.920}, {0.863, -13.690}};
            
            for(int i=0; i<4; i++)
            {
                _exclusion_range_coordinate_list.push_back(def_exclusion_range_coordinate[i]);
            }
            
        }

        // --- パブ ---
        // 初期位置
        pub_initial = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + _entityId + "/initialpose", ROS_QUEUE_SIZE_5, true);       // true：ラッチ[オプション] 最後に発行されたメッセージが保存される
        // cancel                                     
        pub_cancel = node.advertise<actionlib_msgs::GoalID>("/" + _entityId + "/move_base/cancel", ROS_QUEUE_SIZE_5, true);
        // goal
        pub_goal = node.advertise<geometry_msgs::PoseStamped>("/" + _entityId + "/move_base_simple/goal", ROS_QUEUE_SIZE_5, false);
        // ロボットステータス
        pub_robot_sts = node.advertise<delivery_robot::r_state>("/state", ROS_QUEUE_SIZE_10, true);
        timer = node.createTimer(ros::Duration(ROS_TIME_1S), &RobotNode::robotStatusSend, this);// 1Hz
        // answer
        pub_answer = node.advertise<delivery_robot::r_navi_result>("/navi_cmdexe", ROS_QUEUE_SIZE_100, false); // 2020/09/30修正
        // 緊急停止応答
        pub_emergency_ans = node.advertise<delivery_robot::r_emergency_result>("/emgexe", ROS_QUEUE_SIZE_100, false);
        // 他ロボットの経路情報反映済みコストマップ配信
        pub_plan_costmap = node.advertise<nav_msgs::OccupancyGrid>("/" + _entityId + "/plan_costmap", ROS_QUEUE_SIZE_100, true);
        // ロボットの情報通知配信(2020/09/28追加)
        pub_info = node.advertise<delivery_robot::r_info>("/robo_info", ROS_QUEUE_SIZE_100, true);

        // --- サブ ---
        // move_baseステータス
        sub_move_base_status = node.subscribe("/" + _entityId + "/move_base/status", ROS_QUEUE_SIZE_10, &RobotNode::movebaseStatusRecv, this);
        // 移動指示受信
        sub_command_recv = node.subscribe("/navi_cmd", ROS_QUEUE_SIZE_10, &RobotNode::commandRecv, this);
        // バッテリーステータス受信
        if( _entity_type == DEFAULT_ROBOT_TYPE ){
            sub_battery_state_recv = node.subscribe("/" + _entityId + "/battery_state", ROS_QUEUE_SIZE_10, &RobotNode::batteryStateRecv, this);
        }else{
            sub_battery_state_recv = node.subscribe("/" + _entityId + "/rover_sensor", ROS_QUEUE_SIZE_10, &RobotNode::batteryStateRecvMr, this);
        }
        // amcl_pose受信
        sub_amclpose_recv = node.subscribe("/" + _entityId + "/amcl_pose", ROS_QUEUE_SIZE_10, &RobotNode::amclPoseRecv, this);
        // 緊急停止受信
        sub_emergency_recv = node.subscribe("/emg", ROS_QUEUE_SIZE_10, &RobotNode::emergencyRecv, this);
        // ゴール地点到達時のタイムアウトタイマー
        goal_timer = node.createWallTimer(ros::WallDuration(_goal_allowable_time), &RobotNode::goal_allowable_time, this, true, false);
        // ソシオ地図受信
        sub_sociomap = node.subscribe("/" + _entityId + "/map_movebase", ROS_QUEUE_SIZE_10 ,  &RobotNode::sociomapRecv, this);

        //　スタックチェックタイマー
        stuck_timer = node.createTimer(ros::Duration(_stuck_check_time), &RobotNode::robotStuckCheck, this, false, false);

        return(true);
    }

    //--------------------------------------------------------------------------
    //  ソシオ地図受信
    //--------------------------------------------------------------------------
    void sociomapRecv(const nav_msgs::OccupancyGrid& msg)
    {
        _sociomap_width         = msg.info.width;  //ソシオ地図の高さ
        _sociomap_height        = msg.info.height; //ソシオ地図の幅
        _sociomap_resolution    = (double)msg.info.resolution; //ソシオ地図の解像度
        _sociomap_origin_x      = msg.info.origin.position.x; //ソシオ地図の原点のx座標
        _sociomap_origin_y      = msg.info.origin.position.y; //ソシオ地図の原点のy座標
        
        return;
    }

    //--------------------------------------------------------------------------
    //  空のコストマップの送信
    //--------------------------------------------------------------------------
    void emptyCostmapSend(void)
    {
        nav_msgs::OccupancyGrid empty_cost_map; //パブリッシュする空の経路コストマップ（OccupancyGrid型）
        unsigned int map_size = _sociomap_width * _sociomap_height; //コストマップのサイズを求める
        
        //コストマップの情報をセット
        empty_cost_map.header.stamp             = ros::Time::now();
		empty_cost_map.header.frame_id          = "/map"; // frame_id = map
        empty_cost_map.info.resolution          = _sociomap_resolution; //コストマップの解像度
        empty_cost_map.info.width               = _sociomap_width;  //コストマップの幅
        empty_cost_map.info.height              = _sociomap_height;  //コストマップの高さ
        empty_cost_map.info.origin.position.x   = _sociomap_origin_x; //mapの原点座標
        empty_cost_map.info.origin.position.y   = _sociomap_origin_y; //mapの原点座標

        // コストマップのリサイズ
        empty_cost_map.data.resize(map_size);

        // コストを0で埋める
        for(int idx = 0; idx < map_size; idx++)
        {
            empty_cost_map.data[idx] = FREESPACE_COST_GRIDMAP;    //コスト変換テーブルを参照し、コストの値(0~255)に対応する値(-1~100)を取り出す
        }

        // コストマップをパブリッシュ
        pub_plan_costmap.publish(empty_cost_map);

        _is_pub_ori_plan_costmap = false; // オリジナルの経路コストマップは未パブリッシュ

        return;
    }

    //--------------------------------------------------------------------------
    //  コストマップの送信
    //--------------------------------------------------------------------------
    void costmapSend(const delivery_robot::r_costmap costmap_data)
    {
        /* コストマップデータを変換(r_costmap → OccupancyGrid)し/plan_costmapとしてパブリッシュする */
        nav_msgs::OccupancyGrid plan_cost_grid_map; //他ロボットの経路コストマップ（OccupancyGrid型）
        unsigned int costmap_width              = costmap_data.width; //コストマップの幅
        unsigned int costmap_height             = costmap_data.height; //コストマップの高さ
        unsigned int map_size                   = costmap_width * costmap_height; //コストマップのサイズを求める
        unsigned int cost_table_idx             = 0;
        float costmap_resolution                = (float)costmap_data.resolution; // 解像度
        delivery_robot::r_pose costmap_origin   = costmap_data.origin; //原点座標

        //コストマップの情報をセット
        plan_cost_grid_map.header.stamp         = ros::Time::now();
		plan_cost_grid_map.header.frame_id      = "/map"; // frame_id = map
        plan_cost_grid_map.info.resolution      = costmap_resolution; //コストマップの解像度
        plan_cost_grid_map.info.width           = costmap_width;
        plan_cost_grid_map.info.height          = costmap_height;
        plan_cost_grid_map.info.origin.position = costmap_origin.point; //mapの原点座標

        plan_cost_grid_map.data.resize(costmap_width * costmap_height);

        // コストの変換（0~255→-1~100）
        for(int idx = 0; idx < map_size; idx++)
        {
            cost_table_idx = costmap_data.cost_value[idx];    // コスト値を取り出す
            plan_cost_grid_map.data[idx] = _cost_trans_table[cost_table_idx];    //コスト変換テーブルを参照し、コストの値(0~255)に対応する値(-1~100)を取り出す
        }

        // コストマップをパブリッシュ
        pub_plan_costmap.publish(plan_cost_grid_map);

        _is_pub_ori_plan_costmap = true; // オリジナルの経路コストマップをパブリッシュ済み

        return;
    }

    //--------------------------------------------------------------------------
    //  コストマップの送信
    //--------------------------------------------------------------------------
    void costmapSend(const delivery_robot::r_costmap costmap_data, int8_t cost )
    {
        /* コストマップデータを変換(r_costmap → OccupancyGrid)し/plan_costmapとしてパブリッシュする */
        nav_msgs::OccupancyGrid plan_cost_grid_map; //他ロボットの経路コストマップ（OccupancyGrid型）
        unsigned int costmap_width              = costmap_data.width; //コストマップの幅
        unsigned int costmap_height             = costmap_data.height; //コストマップの高さ
        unsigned int map_size                   = costmap_width * costmap_height; //コストマップのサイズを求める
        unsigned int cost_table_idx             = 0;
        float costmap_resolution                = (float)costmap_data.resolution; // 解像度
        delivery_robot::r_pose costmap_origin   = costmap_data.origin; //原点座標

        //コストマップの情報をセット
        plan_cost_grid_map.header.stamp         = ros::Time::now();
		plan_cost_grid_map.header.frame_id      = "/map"; // frame_id = map
        plan_cost_grid_map.info.resolution      = costmap_resolution; //コストマップの解像度
        plan_cost_grid_map.info.width           = costmap_width;
        plan_cost_grid_map.info.height          = costmap_height;
        plan_cost_grid_map.info.origin.position = costmap_origin.point; //mapの原点座標

        plan_cost_grid_map.data.resize(costmap_width * costmap_height);


        // コストの格納
        for(int idx = 0; idx < map_size; idx++)
        {
            if(costmap_data.cost_value[idx] == OBSTACLE_COST)
            {
                plan_cost_grid_map.data[idx] = cost;    // 引数のコスト値に変換する
            }
        }

        // コストマップをパブリッシュ
        pub_plan_costmap.publish(plan_cost_grid_map);

        _is_pub_ori_plan_costmap = false; // オリジナルの経路コストマップは未パブリッシュへ

        return;
    }
    
    //--------------------------------------------------------------------------
    //  受信したコストマップの情報チェック
    //--------------------------------------------------------------------------
    bool checkCostmapInfo(const delivery_robot::r_costmap costmap_data)
    {
        bool isMatchInfo = true; // 地図情報が一致の場合true
        
        // チェック1：ソシオ地図の幅とコストマップの幅
        if(costmap_data.width != _sociomap_width)
        {
            isMatchInfo = false;
        }

        // チェック2：ソシオ地図の高さとコストマップの高さ
        if(costmap_data.height != _sociomap_height)
        {
            isMatchInfo = false;
        }

        // チェック3：ソシオ地図の解像度はコストマップの解像度
        if(fabsf(costmap_data.resolution - _sociomap_resolution) > FLT_EPSILON) // costmap_data.resolution = double, _sociomap_resolution = float
        {
            isMatchInfo = false;
        }

        // チェック4：ソシオ地図の解像度はコストマップの原点（x）
        if(fabs(costmap_data.origin.point.x - _sociomap_origin_x) > DBL_EPSILON)
        {
            isMatchInfo = false;
        }

		// チェック5：ソシオ地図の解像度はコストマップの原点（y）
        if(fabs(costmap_data.origin.point.y - _sociomap_origin_y) > DBL_EPSILON)
        {
            isMatchInfo = false;
        }

        return isMatchInfo;
    }

    //------------------------------------------------------------------------------
    //  更新用のコストマップの全コストを走査し、差分の数を求める
    //------------------------------------------------------------------------------
    unsigned int getCostDifferencialCount(const delivery_robot::r_costmap costmap_data)
    {
        unsigned int costmap_size; // コストマップ上の座標値
        unsigned int cost_diff_counter = 0; // コストの差異の総数

        // コストマップのサイズを求める
        costmap_size = costmap_data.width * costmap_data.height;

        //コストマップのデータをチェックする
        for(unsigned int idx = 0; idx < costmap_size; idx++)
        {
            if(costmap_data.cost_value[idx] != _navi_cmd_costmap.cost_value[idx])
            { // コストが一致しない場合
                // コストの差異カウンターをカウントアップ
                cost_diff_counter++;
            }
        }

        if(cost_diff_counter == 0)
        {
            ROS_INFO("costmap data match!!!");
        }
        else
        {
            ROS_WARN("costmap data unmatch...%d", cost_diff_counter);
        }

        return( cost_diff_counter );
    }
    
    //------------------------------------------------------------------------------
    //  移動指示受信
    //------------------------------------------------------------------------------
    void commandRecv(const delivery_robot::r_navi_command msg)
    {
        ROS_INFO("commandRecv id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.cmd.c_str() );
        std::vector<std::string> err_list;

        // コマンド取得 
        std::string cmd_status = msg.cmd; // 受信したCMD


        if( _mode_status == MODE_STANDBY && cmd_status == CMD_NAVI)
        { // 待機中のnavi受信

            removeAllGoals();  // goal全削除
            // 目的地
            _destinations.push_back(msg.destination);

            if(_calibration_flg == true)
            {
                // キャリブレーション中
                err_list.push_back("during calibration");
                commandAnswer( msg, RESULT_ERROR, err_list);
            }
            else
            {
                // ナビ（自動走行）
                if( msg.costmap.cost_value.size() >= 1 && checkCostmapInfo(msg.costmap))
                {
                    // コストマップの送信
                    costmapSend(msg.costmap);

                    _navi_cmd_costmap = msg.costmap; // メッセージのコストマップをコピー
                    
                }
                else
                {
                    if(msg.costmap.cost_value.size() == 0)
                    {
                        ROS_WARN("The cost map data is empty"); // コストマップのデータが空です
                    }
                    if(!checkCostmapInfo(msg.costmap))
                    {
                        ROS_WARN("Map information doesn't match"); // 地図情報が一致しません
                    }
                    _is_pub_ori_plan_costmap = false; // オリジナルの経路コストマップは未パブリッシュ
                }

                // navi開始
                ROS_INFO("commandRecv destination point x: (%fl), y: (%fl)", msg.destination.point.x,  msg.destination.point.y);
                _mode_status = MODE_NAVI;  // mode naviセット
                _navi_flg = true;

                // 移動指示結果応答
                commandAnswer( msg, RESULT_ACK, err_list);
            }
        }
        else if( (_mode_status == MODE_NAVI || _mode_status == MODE_SUSPEND) && cmd_status == CMD_NAVI)
        { // 移動中のNavi受信(コストマップ更新)
            // 現在の目的地が更新され、メッセージのコマンドが一致しているか
            if( fabs(_current_destination.point.x - msg.destination.point.x) < DBL_EPSILON &&
                fabs(_current_destination.point.y - msg.destination.point.y) < DBL_EPSILON && 
                _update_current_destination)
            {
                // 一致していれば更新するコストマップを送信
                if( msg.costmap.cost_value.size() >= 1 && checkCostmapInfo(msg.costmap))
                {
                    ROS_INFO("update costmap");
                    
                    // スタック検知済みの場合、コストの値を下げているのでそれに合わせる
                    if(_is_pub_ori_plan_costmap)
                    { // オリジナルの経路コストマップがパブリッシュされている場合
                        // コストマップの送信
                        costmapSend(msg.costmap); // コスト置き換えなし
                    }
                    else
                    { // コストを下げた経路コストマップがパブリッシュされている場合
                        // コストマップの送信
                        costmapSend(msg.costmap, _replacing_cost); // コスト置き換えあり
                    }

                    if(_mode_status == MODE_NAVI)
                    { // navi中
                        // 更新前と更新されるコストマップの差異をチェック
                        if(getCostDifferencialCount(msg.costmap) >= DIFFERENCIAL_COST_THRESHOLD)
                        {
                            // ロボットのナビゲーション停止
                            movebaseCancel();   // 走行中断
                            _driver->stopOdom();// いったん停止

                            // 目的地を削除
                            removeAllGoals();   // goal全削除

                            // リルート
                            simpleGoalSend();
                        }

                        // オリジナルの経路コストマップがプッシュ済みの場合のみスタックタイマーを再開する
                        if(_is_pub_ori_plan_costmap)
                        { // オリジナルの経路コストマップを送信の場合
                            stuck_timer.start();
                        }
                    }

                    _navi_cmd_costmap = msg.costmap; // メッセージのコストマップをコピー
                }
                else
                {
                    if(msg.costmap.cost_value.size() == 0)
                    {
                        ROS_WARN("The cost map data is empty"); // コストマップのデータが空です
                    }
                    if(!checkCostmapInfo(msg.costmap))
                    {
                        ROS_WARN("Map information doesn't match"); // 地図情報が一致しません
                    }
                    
                    // コストマップが空になる為stuckのチェック処理は必要なし
                    stuck_timer.stop();
                    
                    // 空のコストマップの送信
                    emptyCostmapSend();
                }

                // 結果応答
                commandAnswer( msg, RESULT_ACK, err_list);

            }
            else
            { // 目的地が一致しない場合は無視
                // コマンド無視
                ROS_INFO("commandRecv ignore because the current destination does not match the msg destination");
                ROS_INFO("commandRecv current destination x: (%fl), y: (%fl), msg destination x: (%fl), y: (%fl)", _current_destination.point.x, _current_destination.point.y, msg.destination.point.x,  msg.destination.point.y);
                commandAnswer( msg, RESULT_IGNORE, err_list);
            }
        }
        else if( (_mode_status == MODE_NAVI || _mode_status == MODE_SUSPEND) && cmd_status == CMD_REFRESH)
        { // 移動中のRefresh受信
            stuck_timer.stop();
            movebaseCancel();   // 走行中断
            removeAllGoals();  // goal全削除
            
            // 目的地
            _destinations.push_back( msg.destination);

            _mode_status = MODE_NAVI; // サスペンド中、NAVI状態に復帰させる
            
            // コストマップ情報チェック
            if( msg.costmap.cost_value.size() >= 1 && checkCostmapInfo(msg.costmap)) 
            { // コストマップのサイズ及びコストマップの情報が正常な場合
                // コストマップの送信
                costmapSend(msg.costmap);

                _navi_cmd_costmap = msg.costmap; // メッセージのコストマップをコピー
                
            }
            else
            {
                if(msg.costmap.cost_value.size() == 0)
                {
                    ROS_WARN("The cost map data is empty"); // コストマップのデータが空です
                }
                if(!checkCostmapInfo(msg.costmap))
                {
                    ROS_WARN("Map information doesn't match"); // 地図情報が一致しません
                }

                // 空のコストマップの送信
                emptyCostmapSend();
            }

            // navi継続
            ROS_INFO("commandRecv destination point x: (%fl), y: (%fl)", msg.destination.point.x,  msg.destination.point.y);
            commandAnswer( msg, RESULT_ACK, err_list);
            goalSend();
        
        }
        else if( (_mode_status == MODE_NAVI || _mode_status == MODE_SUSPEND) && cmd_status == CMD_STANDBY)
        { // 移動中のstandby受信
            stuck_timer.stop();
            movebaseCancel();   // 走行中断
            removeAllGoals();  // goal全削除
            
            // 空のコストマップの送信
            emptyCostmapSend();

            _mode_status = MODE_STANDBY;  // mode standbyセット
            _navi_flg = false;
            commandAnswer( msg, RESULT_ACK, err_list);
        }
        else
        {
            // コマンド無視
            ROS_INFO("commandRecv ignore MODE:%s CMD:%s", _mode_status.c_str(), cmd_status.c_str());
            commandAnswer( msg, RESULT_IGNORE, err_list);
        }

        return;
    }

    //------------------------------------------------------------------------------
    //  移動指示    結果応答
    //------------------------------------------------------------------------------
    void commandAnswer(const delivery_robot::r_navi_command& cmd_msg, std::string result_kind, std::vector<std::string>& err_list)
    {
        delivery_robot::r_navi_result ans_msg;

        ans_msg.id = _entityId;
        ans_msg.type = _entity_type;
        ans_msg.time =  iso8601ex();
        ans_msg.received_time = cmd_msg.time;
        ans_msg.received_cmd = cmd_msg.cmd;

        // 目標地点のコピー
        waypointCopy(cmd_msg.destination, ans_msg.received_destination);

        // コストマップをコピー
        ans_msg.received_costmap = cmd_msg.costmap;
    
        // 処理結果を格納
        ans_msg.result = result_kind;

        // エラーメッセージ
        int err_cnt = err_list.size();
        ans_msg.errors.resize(err_cnt);
        for( int i=0; i<err_cnt; i++){
            ans_msg.errors[i] = err_list[i];
        }

        // パブ 
        pub_answer.publish(ans_msg);

        return;
    }

    //--------------------------------------------------------------------------
    //  ロボットの情報通知
    //--------------------------------------------------------------------------
    void robotInfoSend(void)
    {
        delivery_robot::r_info info_msg; //パブリッシュするメッセージ

        info_msg.id                             = _entityId;        // ロボットのユニークID
        info_msg.type                           = _entity_type;     // ロボットの種類
        info_msg.time                           = iso8601ex();      // 現在時刻
        info_msg.robot_size.robot_radius        = _robot_radius;    // ロボットの幅
        info_msg.robot_size.inflation_radius    = _inflation_raidus; // マージンを考慮したロボットの幅
        info_msg.robot_size.footprint           = _footprint;       // footprint情報

        pub_info.publish(info_msg);

        return;
    }

    //--------------------------------------------------------------------------
    //  緊急停止    受信
    //--------------------------------------------------------------------------
    void emergencyRecv(const delivery_robot::r_emergency_command& msg)
    {
        ROS_INFO("emergencyRecv id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.emergency_cmd.c_str() );

        // コマンド取得 
        std::string emg_cmd = msg.emergency_cmd; // 受信したCMD
        std::string result_kind = RESULT_ACK;
        std::vector<std::string> err_list;

        if(emg_cmd == EMERGENCY_STOP)
        {
            if(_calibration_flg == true)
            { // キャリブレーション中
                result_kind = RESULT_ERROR;
                err_list.push_back("during calibration"); //　エラーリストへ格納
            }
            else
            {
                movebaseCancel();   // 走行中断
                _driver->stopOdom();// いったん停止
                removeAllGoals();   // goal全削除
                stuck_timer.stop(); // スタックタイマー停止

                //navi中またはsuspend中に受け取った場合は空のコストマップを投げる
                if(_mode_status == MODE_NAVI || _mode_status == MODE_SUSPEND)
                {
                    // 空のコストマップの送信
                    emptyCostmapSend();
                }

                _mode_status = MODE_STANDBY;  // mode standbyセット
                _navi_flg = false;
            }
        }
        else if( emg_cmd == EMERGENCY_SUSPEND)
        {
            if(_mode_status == MODE_NAVI)
            {  // navi中
                movebaseCancel();   // 走行中断
                _driver->stopOdom();// いったん停止
                _destinations.push_front(_current_destination); // 現在の目的値を保持
                goal_timer.stop();
                stuck_timer.stop();
                _mode_status = MODE_SUSPEND;
            }
            else
            { // ナビ走行中ではない
                // コマンド無視
                result_kind = RESULT_IGNORE;
            }
        }
        else if(emg_cmd == EMERGENCY_RESUME)
        {
            if(_mode_status == MODE_SUSPEND)
            {  // サスペンド中
                _mode_status = MODE_NAVI;
                goalSend();
            }
            else
            { // サスペンド中ではない
                // コマンド無視
                result_kind = RESULT_IGNORE;
            }
        }
        else
        { // 異常コマンドの場合（STOP、SUSPEND、RESUME以外）
            // コマンド無視
            result_kind = RESULT_IGNORE;
            ROS_WARN("The command '%s' is invalid and the received command will be ignored.", emg_cmd.c_str());
        }

        emergencyAnswer(msg, result_kind, err_list);
        
        return;
    }

    //--------------------------------------------------------------------------
    //  緊急停止    応答
    //--------------------------------------------------------------------------
    void emergencyAnswer(const delivery_robot::r_emergency_command& cmd_msg, std::string result_kind, std::vector<std::string>& err_list)
    {
        delivery_robot::r_emergency_result ans_msg;

        ans_msg.id                      = _entityId;
        ans_msg.type                    = _entity_type;
        ans_msg.time                    = iso8601ex();
        ans_msg.received_time           = cmd_msg.time;
        ans_msg.received_emergency_cmd  = cmd_msg.emergency_cmd;

        // 処理結果を格納
        ans_msg.result = result_kind;

        // エラーメッセージ
        int err_cnt = err_list.size();
        ans_msg.errors.resize(err_cnt);
        for(int i = 0; i < err_cnt; i++)
        {
            ans_msg.errors[i] = err_list[i];
        }

        ROS_INFO("emergencyAns id(%s) type(%s) time(%s) rtime(%s) rcmd(%s) result(%s)", ans_msg.id.c_str(), ans_msg.type.c_str(), ans_msg.time.c_str(), ans_msg.received_time.c_str(), ans_msg.received_emergency_cmd.c_str(), ans_msg.result.c_str());

        pub_emergency_ans.publish(ans_msg);
        
        return;
    }

   
    //------------------------------------------------------------------------------
    //  バッテリステータス受信  30Hz
    //------------------------------------------------------------------------------
    void batteryStateRecv(const sensor_msgs::BatteryState msg)
    {
        _volt_sts = msg.voltage;
        
        return;
    }

    //------------------------------------------------------------------------------
    //  バッテリステータス受信  メガローバ  90Hz
    //------------------------------------------------------------------------------
    void batteryStateRecvMr(const std_msgs::Int16MultiArray msg)
    {
        if(msg.data.size() >= 10){
            _volt_sts = msg.data[9] / MR_VOLTAGE_FACTOR;
        }
        
        return;
    }

    //------------------------------------------------------------------------------
    //  amcl_pose受信
    //------------------------------------------------------------------------------
    void amclPoseRecv(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
        try
        {
            memcpy( &_g_covariance, &msg.pose.covariance, sizeof(_g_covariance));//共分散
        }
        catch(const std::exception &e)
        {
            ROS_WARN("amcl_pose conversion errer[%s]", e.what());
        }
        
        return;
    }

    //--------------------------------------------------------------------------
    //  move_baseステータスのエラーチェック
    //--------------------------------------------------------------------------
    void robotStatusErrCheck(std::vector<std::string>& sts_err_list)
    {
        if( MOVE_BASE_ABORTED == _move_base_sts ){
            sts_err_list.push_back("I can not reach the goal");// 到達できません
        } 
        
        return;
    }


    //--------------------------------------------------------------------------
    //  ロボットがスタックしていないかチェック
    //--------------------------------------------------------------------------
    void robotStuckCheck(const ros::TimerEvent&)
    {
        double current_x, current_y, current_yaw;
        double distance;

        // 現在位置の取得
        currentCoordinates(current_x, current_y, current_yaw);

        // 過去の位置との比較
        distance = hypotf(_Past_Position.x - current_x, _Past_Position.y - current_y);

        if( distance - DBL_EPSILON <= _stuck_threshold_length )
        { // スタック判定距離より短い場合（スタック時）
            if(_is_pub_ori_plan_costmap)
            { // オリジナルの経路コストマップ反映時
                // コストを低くしたコストマップを送信する
                costmapSend(_navi_cmd_costmap, _replacing_cost);
                stuck_timer.stop(); // 一度コストを投げたらチェック終了
                ROS_INFO("Within the threshold distance. (%fl [m])", distance);
            }
            else
            { // オリジナルの経路コストマップ未反映時
                stuck_timer.stop(); //コストマップ未反映時はチェック終了
            }
            
        }
        else
        { // 判定距離以上の場合
            // 1回前との距離を表示
            ROS_INFO("Above the threshold distance. (%fl [m])", distance);
        }

        // 過去位置を現在位置での更新
        _Past_Position.x = current_x;
        _Past_Position.y = current_y;
        _Past_Position.z = 0;

        return;
    }

    //--------------------------------------------------------------------------
    //  ロボットステータス送信
    //--------------------------------------------------------------------------
    void robotStatusSend(const ros::TimerEvent&)
    {
        double x, y, z, roll, pitch, yaw;
        std::vector<std::string> sts_err_list;
        try
        {
            // エラーチェック
            robotStatusErrCheck(sts_err_list);
            int err_cnt = sts_err_list.size();
            // 座標取得
            tf::StampedTransform trans;
            ros::Time now = ros::Time::now();
            tfl.waitForTransform("map", _entityId + "/base_link",
                    now, ros::Duration(ROS_TIME_3S));
            tfl.lookupTransform("map", _entityId + "/base_link",   // mapからbase_link、(world座標のロボットの位置)
                    ros::Time(ROS_TIME_0S), trans);
            x = trans.getOrigin().x();                // X座標
            y = trans.getOrigin().y();                // Y座標
            z = 0.0;                                  // Z座標(※ 0固定)
            tf::Matrix3x3 m = trans.getBasis();
            m.getRPY(roll, pitch, yaw);

            // --- ロボットステータス ---
            delivery_robot::r_state msg;

            msg.id   = _entityId;
            msg.type = _entity_type;
            msg.time = iso8601ex();
            if(err_cnt <= 0){ 
                msg.mode = _mode_status;
            }else{
                msg.mode = MODE_ERROR;// エラー応答
                msg.errors.resize(err_cnt);
                for( int i = 0; i < err_cnt; i++){
                    msg.errors[i] = sts_err_list[i];// エラーメッセージ
                }
            } 
            // 現在の位置・向き
            msg.pose.point.x = x;
            msg.pose.point.y = y;
            msg.pose.point.z = z;
            msg.pose.angle.roll = roll;
            msg.pose.angle.pitch = pitch;
            msg.pose.angle.yaw = yaw;

            //  現在の目的値
            if( _navi_flg == true ){
                memcpy( &msg.destination, &_current_destination, sizeof(msg.destination));
            }

            memcpy( &msg.covariance, &_g_covariance, sizeof(msg.covariance));// 共分散

            // バッテリー情報
            msg.battery.voltage = _volt_sts;
            msg.battery.current_optional.valid = false;
            msg.battery.current_optional.current  = 0.0;

            // パブリッシュ
            pub_robot_sts.publish(msg);
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("robot status send err[%s]", e.what());
        }
        
        return;
    }
    
    //--------------------------------------------------------------------------
    //  初期位置送信
    //--------------------------------------------------------------------------
    void initialPoseSend(const float x, const float y, const float yaw)
    {
        _initial_pose.header.frame_id = "map";
        _initial_pose.pose.pose.position.x = x;
        _initial_pose.pose.pose.position.y = y;
        _initial_pose.pose.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

        memset(&_initial_pose.pose.covariance[0], 0, sizeof(_initial_pose.pose.covariance));//共分散行列（散らばり具合を表す指標）
        _initial_pose.pose.covariance[0] = 0.25;
        _initial_pose.pose.covariance[7] = 0.25;
        _initial_pose.pose.covariance[35] = 0.06853891945200942;
        pub_initial.publish(_initial_pose);

        return;
    }

    //--------------------------------------------------------------------------
    //  目的値全削除
    //--------------------------------------------------------------------------
    void removeAllGoals(void)
    {
        while ( _destinations.size() >= 1) _destinations.pop_front();
        _destinations.clear();

        return;
    }

    //--------------------------------------------------------------------------
    //  movebase cancel 送信
    //--------------------------------------------------------------------------
    void movebaseCancel(void)
    {
        if(_navi_node == false){
             // アクションクライアント  
            _driver->stopOdom();  // いったん停止
            //tell the action client that we want to spin a thread by default
            MoveBaseClient ac( _entityId + "/move_base", true);
            
            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(ROS_TIME_5S))){
                ROS_INFO("Waiting for the move_base action server to come up");     // move_baseアクションサーバが起動するのを待っています
            }

            ac.cancelAllGoals(); 
            return;

        }else{
            // 現在地点を目標地点とする
            float x, y, z, yaw;
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "/base_link",
                     ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_3S));
            tfl.lookupTransform("map", _entityId + "/base_link",   // mapからbase_link、(world座標のロボットの位置)
                     ros::Time(ROS_TIME_0S), trans);
            x = trans.getOrigin().x();                // X座標
            y = trans.getOrigin().y();                // Y座標
            z = trans.getOrigin().z();                // Z座標
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得

            geometry_msgs::PoseStamped stop_goal;

            stop_goal.header.frame_id = "map";
            stop_goal.pose.position.x = x;
            stop_goal.pose.position.y = y;
            stop_goal.pose.orientation = GetQuaternionMsg( 0, 0, yaw);
            pub_goal.publish(stop_goal);
        }
        
        return;
    } 

    //--------------------------------------------------------------------------
    //  move base ステータス受信
    //--------------------------------------------------------------------------
    void movebaseStatusRecv(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
    {
        int status_id = MOVE_BASE_INIT; // MOVE_BASE_INIT = -1
        //uint8 PENDING         = 0  // まだアクションサーバによって処理されていない
        //uint8 ACTIVE          = 1  // ゴールは現在アクションサーバによって処理されている
        //uint8 PREEMPTED       = 2  // 実行開始後、キャンセル要求を受け取りました
        //uint8 SUCCEEDED       = 3  // 目標は、アクション・サーバ（終端状態）で成功裏に達成した
        //uint8 ABORTED         = 4  // 目標は、アクション・サーバによるによって実行中に中止されました
        //uint8 REJECTED        = 5  // 目標が処理できずにアクションサーバーによって拒否されました。目標が到達不能または無効だったため
        //uint8 PREEMPTING      = 6  // 目標は実行開始後、キャンセル要求を受け取り、実行は完了していません
        //uint8 RECALLING       = 7  // ゴールは実行を開始する前にキャンセル要求を受け取りました
        //uint8 RECALLED        = 8  // ゴールは実行を開始する前にキャンセル要求を受け取って＃正常にキャンセルされました
        //uint8 LOST            = 9  // アクションクライアントは、ゴールがLOSTであると判断できます。

        if (!status->status_list.empty())
        {
            actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
            status_id = goalStatus.status;

            _move_base_sts = status_id;
        }

        if(status_id == MOVE_BASE_ACTIVE)
        { //移動中
            // ROS_INFO("movebase START(%d)", status_id);
        }
        else if((status_id == MOVE_BASE_SUCCEEDED)||(status_id == MOVE_BASE_PENDING))
        { //ゴールに到達・もしくはゴールに到達して待機中。
            if(_move_base_status_id != MOVE_BASE_SUCCEEDED && _move_base_status_id != MOVE_BASE_PENDING)
            { // ステータス更新前の初回動作
                ROS_INFO("movebase GOAL(%d)", status_id);
            }
        }
        else if( status_id==MOVE_BASE_ABORTED )
        { // スタック時
            if( _mode_status == MODE_NAVI && _move_base_status_id != MOVE_BASE_ABORTED)
            { // ステータス更新前の初回動作
                ROS_INFO("movebase ABORTED!!");
                
                // ナビ走行中であれば走行停止
                movebaseCancel();   // 走行中断
                removeAllGoals();  // goal全削除

                stuck_timer.stop();

                // 空のコストマップの送信
                emptyCostmapSend();

                _mode_status = MODE_STANDBY;  // mode standbyセット
                _navi_flg = false;
            }
        }
        _move_base_status_id = status_id;
        
        return;
    }

    //--------------------------------------------------------------------------
    //  旋回角度と向きを取得する
    //--------------------------------------------------------------------------
    bool getTurnAngle(const double rad_base, const double rad , double &turn_rad)
    {
        turn_rad = 0;
        bool blsts = true;
        double current_rad = rad_base + M_PI;
        double goal_rad    = rad      + M_PI;

        if( current_rad >= goal_rad ){
            if( (current_rad - goal_rad) <= M_PI ){
                blsts = true;   // 右旋回
                turn_rad = current_rad - goal_rad;
            }else{
                blsts = false;  // 左旋回
                turn_rad = 2*M_PI - (current_rad - goal_rad);
            }
        }else{
            if( (goal_rad - current_rad) <= M_PI ){
                turn_rad = goal_rad - current_rad;
                blsts = false;  // 左旋回
            }else{
                turn_rad = 2*M_PI - (goal_rad - current_rad);
                blsts = true;   // 右旋回
            }
        }

        return( blsts );
    }

    //--------------------------------------------------------------------------
    //  wayポイント方向に向きを変える
    //--------------------------------------------------------------------------
    double turnTowardsGoal()
    {
        double x, y, yaw;
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
            tfl.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);// mapから見たbase_link、(world座標のロボットの位置)
            x = trans.getOrigin().x();                // Ｘ座標
            y = trans.getOrigin().y();                // Ｙ座標
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("turnTowardsGoal() err(%s)", e.what());
            return(0.0);
        }

        // goalに向かう向きを取得    
        double yaw_way = atan2((double)(_current_destination.point.y - y), (double)(_current_destination.point.x - x));

        // 旋回角度を取得
        double turn_rad = 0; //旋回角度
        bool turn_direction = getTurnAngle( yaw, yaw_way, turn_rad);

        // 旋回させる
        ros::Rate rate(ROS_RATE_30HZ);   // 30Hz処理

        for(;;)
        {
            // 旋回させる
            _driver->moveTurn(turn_direction, _navigation_turn_speed);  
 
            rate.sleep();
            ros::spinOnce();

            try
            {
                // 現在のロボットの向きを取得
                tf::StampedTransform trans;
                tfl.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
                tfl.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);// mapから見たbase_link、(world座標のロボットの位置)
                x = trans.getOrigin().x();                // Ｘ座標
                y = trans.getOrigin().y();                // Ｙ座標
                yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
            }
            catch(tf::TransformException &e)
            {
                ROS_WARN("turnTowardsGoal() err(%s)", e.what());
                continue;
            }

            // goalに向かう向きを取得    
            yaw_way = atan2((double)(_current_destination.point.y - y), (double)(_current_destination.point.x - x));

            // 旋回角度を取得
            turn_rad = ANGLE_OF_0_DEGREES; //旋回角度
            bool turn_direction_now = getTurnAngle( yaw, yaw_way, turn_rad);

            if(turn_direction_now != turn_direction){
                ROS_INFO("turn_direction reverse");
                turn_direction = turn_direction_now;
            } 

            // WP方向までの角度差が3度以下か判定する
            if(RAD2DEG(turn_rad) <= ANGLE_OF_3_DEGREES){
                break;
            }

            if(_navi_flg == false || _mode_status == MODE_SUSPEND){
                break;
            } 
        } 

        _driver->stopOdom();  // いったん停止

        return( yaw_way );
    }

    //--------------------------------------------------------------------------
    //  現在の座標,向きを取得する
    //--------------------------------------------------------------------------
    bool currentCoordinates( double& cur_x, double& cur_y, double& cur_yaw )
    {
        bool ret_sts = true;
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_500MS));
            tfl.lookupTransform("map", _entityId + "/base_link", ros::Time(ROS_TIME_0S), trans);// mapから見たbase_link、(world座標のロボットの位置)
            cur_x = trans.getOrigin().x();                // Ｘ座標
            cur_y = trans.getOrigin().y();                // Ｙ座標
            cur_yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("currentCoordinates() err(%s)", ex.what());
            ret_sts = true;
        }

        return( ret_sts );
    }

    //--------------------------------------------------------------------------
    //  狙った方向に向きを変える
    //--------------------------------------------------------------------------
    bool turnAngle( double angle_yaw, double allowable_angle )
    {
        double cur_x, cur_y, cur_yaw;
        double turn_rad = 0; //旋回角度
        int err_cnt = 0;
        bool ret_sts = true;
        bool loop_end = false;

        ros::Rate rate(ROS_RATE_30HZ);   // 30Hz処理

        while(!loop_end)
        {
            rate.sleep();
            ros::spinOnce();

            if( true == currentCoordinates( cur_x, cur_y, cur_yaw )){   // 現在の向きを取得
                bool turn_direction = getTurnAngle( cur_yaw, angle_yaw, turn_rad ); // 角度のずれを取得
                ROS_INFO( "turn_rad(%f) allowable_angle(%f)", turn_rad, allowable_angle);
                if( turn_rad > allowable_angle ){   // 許容範囲よりずれているか
                    // 旋回させる
                    _driver->moveTurn(turn_direction, _navigation_turn_speed);
                }else{
                    loop_end = true;
                }
            }else{
                err_cnt++;
                if( err_cnt >= 10 ){
                    ret_sts = false;
                    loop_end = true;
                }
            }
        }

        _driver->stopOdom();  // いったん停止

        return( ret_sts );
    }

    //------------------------------------------------------------------------------
    //  ゴール地点到達時のタイマータイムアウト
    //------------------------------------------------------------------------------
    void goal_allowable_time(const ros::WallTimerEvent&) 
    {
        ROS_INFO("!!!!goal_allowable_time Out!!!!");
        _goal_allowable_flg = true;
        movebaseCancel();//停止させる
        
        return;
    }

    //------------------------------------------------------------------------------
    //  Sleep処理
    //------------------------------------------------------------------------------
    void sleepFunc(double sleep_time)
    {
        double sleepTotal = 0;
        
        ros::Rate rate(ROS_RATE_20HZ);   // 20Hz処理

        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();
            sleepTotal += ROS_TIME_50MS;    // 0.05[s]
            if(sleepTotal >= sleep_time){
                break;
            }
        }
        
        return;
    } 

    //--------------------------------------------------------------------------
    //  PoseStampedメッセージ作成
    //--------------------------------------------------------------------------
    geometry_msgs::PoseStamped makePoseStamped( double x, double y, double yaw)
    {
        geometry_msgs::PoseStamped  posePoseStamped;

        posePoseStamped.header.frame_id = "map";
        posePoseStamped.pose.position.x = x;
        posePoseStamped.pose.position.y = y;
        posePoseStamped.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

        return( posePoseStamped );
    }

    //--------------------------------------------------------------------------
    //  旋回処理実行対象かチェックを行う
    //--------------------------------------------------------------------------
    bool checkCurrentPosition(void)
    {
        bool isTurnControl = true; // 旋回制御実行フラグ

        double current_x, current_y, current_yaw;
        double x_min= 1e6;
        double y_min= 1e6;
        double x_max= -1e6;	
        double y_max= -1e6;	

        // 現在地情報を取得する
        currentCoordinates(current_x, current_y, current_yaw);
        
        // ナビゲーション開始地点を格納
        _Past_Position.x = current_x;
        _Past_Position.y = current_y;
        _Past_Position.z = 0;

        // 現在座標の表示
        ROS_INFO("current point x: %fl, y: %fl", current_x, current_y);

        // 旋回除外範囲の最小座標、
        for(std::vector<stExclusionPoint>::const_iterator ite = _exclusion_range_coordinate_list.begin ();
							ite != _exclusion_range_coordinate_list.end (); ite++)
		{
            x_min = std::min(x_min, ite->x);
            y_min = std::min(y_min, ite->y);
            x_max = std::max(x_max, ite->x);
            y_max = std::max(y_max, ite->y);
        }

        // 現在位置の座標が旋回制御実行対象外かチェック
        if( (x_min - DBL_EPSILON) <= current_x &&   // x座標の最小以上 (- DBL_EPSILONは計算機イプシロン分の誤差分を保証)
            current_x <= (x_max + DBL_EPSILON) &&   // x座標の最大以下 (+ DBL_EPSILONは計算機イプシロン分の誤差分を保証)
            (y_min - DBL_EPSILON) <= current_y &&   // y座標の最小以上
            current_y <= (y_max + DBL_EPSILON) )    // y座標の最大以下
        { // 旋回制御実行対象外の場合
        	
            isTurnControl = false;
        }

        return( isTurnControl );

    }

    //--------------------------------------------------------------------------
    //  wayポイント送信
    //--------------------------------------------------------------------------
    bool goalSend(void)
    {
        double yaw;

        bool retSts = false;

        goal_timer.stop();
        _goal_allowable_flg = false;

        if(_destinations.size() == 0) return false;

        //現在の目的地更新
        _current_destination = _destinations.front();
        //目的地更新フラグON
        _update_current_destination = true;
        _destinations.pop_front();

        if(_turn_busy_flg == false)
        {

            _turn_busy_flg = true; 

            // 現在位置が移動前の旋回制御対象かチェック
            if(checkCurrentPosition())
            { // 旋回制御対象の場合
                ROS_INFO("Enable turning control");
                // ロボットの姿勢をgoalの方向へ向ける
                yaw = turnTowardsGoal();
            }
            else
            {
                // スリープ（コストマップ反映前に経路を取らないようにする為）
                sleepFunc(ROS_TIME_1S);

                ROS_INFO("Disable turning control");
            }
            

            // wayポイント到着時の向き指定ありか
            if( _current_destination.angle_optional.valid == true )
            {
                yaw = _current_destination.angle_optional.angle.yaw;
            }

            geometry_msgs::PoseStamped way_goal = makePoseStamped( _current_destination.point.x, _current_destination.point.y, yaw);

            // パブリッシュ
            if(_navi_flg == true && _mode_status == MODE_NAVI)
            {
                pub_goal.publish(way_goal);
                ROS_INFO("Applying goal x:%0.3f y:%0.3f yaw:%0.3f",
                    way_goal.pose.position.x,
                    way_goal.pose.position.y,
                    tf::getYaw(way_goal.pose.orientation));
                if(_is_pub_ori_plan_costmap)
                { //コストマップが反映済みの場合はスタック監視スタート
                    stuck_timer.start();
                }
                ROS_INFO("!!! stuck_timer start !!!");
                retSts = true;
            }
            else
            {
                retSts = false;
            }
            _turn_busy_flg = false;
        }

        return( retSts );
    }

    //--------------------------------------------------------------------------
    //  wayポイント送信（リルートの実施用）
    //--------------------------------------------------------------------------
    void simpleGoalSend(void)
    {
        double yaw;

        // wayポイント送信前チェック
        if( _turn_busy_flg == false ) // 目的地への旋回動作中か
        { // 旋回中ではない場合
            
            // 現在位置がゴール付近ではないか
            if(!_goal_allowable_flg)
            {
                // wayポイント到着時の向き指定ありか
                if( _current_destination.angle_optional.valid == true )
                {
                    yaw = _current_destination.angle_optional.angle.yaw;
                }

                geometry_msgs::PoseStamped way_goal = makePoseStamped( _current_destination.point.x, _current_destination.point.y, yaw);
            
                // パブリッシュ
                if(_navi_flg == true && _mode_status == MODE_NAVI)
                {
                    pub_goal.publish(way_goal);
                }
            }
        }

        return;
    }

    //--------------------------------------------------------------------------
    //  １回転
    //--------------------------------------------------------------------------
    void turn360(double turn_speed) 
    {
        double yaw;

        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            // Durationは実機:0.5, シミュレータ:5.0
            tfl.waitForTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_5S));
            tfl.lookupTransform(_entityId + "/base_link", _entityId + "/odom", ros::Time(ROS_TIME_0S), trans);// base_linkから見たodom、(ロボットの向き)
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &e)
        {
            ROS_ERROR("turn360() err(%s)", e.what());
            _calibration_flg = false;
            return;
        }

        double turn_threshold = turn_speed / 5;// 回転量のしきい値 0.2s時の最大回転rad

        double yaw_before = 0;
        double yaw_current = yaw;
        double yaw_total = 0;
        double turn_val;

        ROS_INFO("turn360 (%f rad/s) start ---------->",turn_speed);

		ros::Rate rate(ROS_RATE_10HZ);

        while(ros::ok())
        {
            _driver->moveTurn(true, turn_speed); // 旋回させる(右回転)

            rate.sleep();
            ros::spinOnce();
 
            try
            {
                // 現在のロボットの向きを取得
                tf::StampedTransform trans;
                ros::Time now = ros::Time::now();
                tfl.waitForTransform(_entityId + "/base_link", _entityId + "/odom", now, ros::Duration(ROS_TIME_1S));
                tfl.lookupTransform(_entityId + "/base_link", _entityId + "/odom", now, trans);// base_linkodom見たbase_link、(ロボットの向き)
                yaw_current = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
            }
            catch(tf::TransformException &e)
            {
                ROS_ERROR("turn360() err(%s)", e.what());
                return;
            }

            if ( fabs(yaw_current-yaw_before) < 1.0e-2) continue;// 小さすぎる値はスルー // fabs=絶対値

            // 右回転     
            if(yaw_current >= yaw_before){
               turn_val = yaw_current - yaw_before;
                ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
            }else{  
               turn_val = (M_PI - yaw_before) + (M_PI + yaw_current);
               ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
            }

            if(turn_val <= turn_threshold){
                yaw_total += turn_val; // 加算
            }else{
                //ROS_WARN("turn_threshold(%f) turn_val(%f)",turn_threshold, turn_val);
            }

            yaw_before = yaw_current; // 前回値保持

            if (yaw_total >= DEG2RAD(ANGLE_OF_360_DEGREES)){
                break;
            }
        }

        _driver->stopOdom();  // いったん停止
        _calibration_flg = false;

        ROS_INFO("turn360 end <-------------");
        
        return;
    }

    //--------------------------------------------------------------------------
    //  メインループ
    //--------------------------------------------------------------------------
    void mainloop(void)
    {
        ROS_INFO("Standby...");

        ros::Rate rate(ROS_RATE_10HZ);   // 10Hz処理

        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();

            if( _mode_status == MODE_NAVI && _navi_flg == true ){
                if(!goalSend() && _mode_status == MODE_NAVI) //  wayポイント送信
                {
                    ROS_ERROR("No goal specified");
                    _navi_flg = false;
                    continue;   // 移動指示を待つ
                }
                ROS_INFO("Navi Start");
            }else{
                continue;   // 移動指示待ち
            }

            while(ros::ok())
            {
                rate.sleep();
                ros::spinOnce();

                float x, y, yaw;
                try
                {
                    tf::StampedTransform trans;
                    tfl.waitForTransform("map", _entityId + "/base_link",
                            ros::Time(ROS_TIME_0S), ros::Duration(ROS_TIME_50MS));
                    tfl.lookupTransform("map", _entityId + "/base_link",   // mapから見たbase_link、(world座標のロボットの位置)
                            ros::Time(ROS_TIME_0S), trans);
                    x = trans.getOrigin().x();                // Ｘ座標
                    y = trans.getOrigin().y();                // Ｙ座標
                    yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
                }
                catch(tf::TransformException &e)
                {
                    ROS_WARN("%s", e.what());
                    continue;
                }

                ROS_INFO("hypotf(%f)", hypotf(x - _current_destination.point.x, y - _current_destination.point.y) );

                if(_mode_status == MODE_SUSPEND){
                    ROS_INFO("Suspend..."); // サスペンド中
                    continue;
                }

                if(hypotf(x - _current_destination.point.x, y - _current_destination.point.y) <= _goal_allowable_range){  //  目的値まで近づいたらタイマー開始する
                    if(_navi_flg == true){
                        ROS_INFO("Goal Timer Start");
                        goal_timer.start(); //タイマースタート
                    }
                }

                if( ((_move_base_sts == MOVE_BASE_SUCCEEDED || _move_base_sts == MOVE_BASE_PENDING) //ゴールに到達・もしくはゴールに到達して待機中。
                     && hypotf(x - _current_destination.point.x, y - _current_destination.point.y) <= _goal_tolerance_range) //  目的値までの許容範囲以内
                || _goal_allowable_flg == true ) {  // ゴール到達時タイムアウト

                    ROS_INFO("Goal Timer Stop");
                    goal_timer.stop();
                    stuck_timer.stop();
                    ROS_INFO("_move_base_sts(%d)",_move_base_sts);

                    // 角度判定
                    if( _current_destination.angle_optional.valid == true ){ // 角度指定あり
                        double goal_angle_yaw = _current_destination.angle_optional.angle.yaw;
                        turnAngle( goal_angle_yaw, _goal_allowable_angle );
                    }

                    ROS_INFO("way point Goal");
                    
                    // 目的地到達のため、更新フラグをオフ
                    _update_current_destination = false;

                    if( _current_destination.angle_optional.valid == true && _destinations.size() >= 1 ){
                        // ちょっと止まる
                        sleepFunc(_wp_sleep_time);
                    }

                    bool goal_snd_sts = goalSend();

                    if(_mode_status == MODE_NAVI){
                        if(goal_snd_sts == false){
                            // ナビ中の次のwayポイントなし
                            ROS_INFO("Finished");        // wayポイントが無ければ終了

                            // 空のコストマップの送信
                            emptyCostmapSend();

                            _mode_status = MODE_STANDBY;  // mode standbyセット
                            _navi_flg = false;
                        }else{
                            // wayポイント送信
                            ROS_INFO("Next goal applied");
                        }
                    }
                }

                if(_navi_flg == false){
                    ROS_INFO("navi end. Standby... ");
                    break;  // 自動走行終了
                }
            }
        }
        
        return;
    }
};

/******************************************************************************/
/* main                                                                       */
/******************************************************************************/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "delivery_robot_node");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    // 初期位置
    std::vector<double> init_pose;
    if (privateNode.getParam("initial_pose", init_pose)){
        ROS_INFO("initial_pose(%lf,%lf,%lf)", init_pose[0], init_pose[1], init_pose[2]);
    }else{
        ROS_ERROR("not initial pose");
        return(-1);
    }
    // # キャリブレーション 回転速度
    double turn_speed;
    if (privateNode.getParam("initial_turn_speed", turn_speed)){
        ROS_INFO("initial_turn_speed (%lf)", turn_speed);
    }else{
        turn_speed = 0.4; // rad
    }

    RobotDriver driver(node, privateNode);

    RobotNode robot_node(driver);
    robot_node.setup(node, privateNode); 

    // 初期位置送信 
    robot_node.initialPoseSend(init_pose[0], init_pose[1], init_pose[2]);//initialPose送信

    //1回転
    robot_node.turn360(turn_speed);

    // ロボットの情報送信
    robot_node.robotInfoSend();

    ROS_INFO("stanby");

    // メインループ
    robot_node.mainloop();

    
    return(0);
}
