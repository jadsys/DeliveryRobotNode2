#ifndef MULTI_ROBOT_LAYER_H_
#define MULTI_ROBOT_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

// Queue size(最新のデータが欲しい場合は小さく，取りこぼしたくない場合は大きくする)
#define		ROS_QUEUE_SIZE				10

// GridMapコスト
#define     FREESPACE_COST_GRIDMAP		0
#define     UNKNOWN_COST_GRIDMAP		-1

// Costmapコスト
#define     FREESPACE_COST_COSTMAP2D    0
#define     INSCRIBED_COST_COSTMAP2D	253
#define		OBSTACLE_COST_COSTMAP2D		254
#define		UNKNOWN_COST_COSTMAP2D		255
#define     PLAN_COST_MAX				INSCRIBED_COST_COSTMAP2D

// ID,TYPE
#define     DEFAULT_ROBOT_ID        "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE      "turtlebot"

namespace multi_robot_layer_namespace
{
	class MultiRobotLayer : public costmap_2d::Layer
	{
		public:
			MultiRobotLayer();
			~MultiRobotLayer();

			virtual void onInitialize();
			virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
									  double* min_x, double* min_y, double* max_x,double* max_y);
			virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
									 int min_i, int min_j, int max_i, int max_j);

		private:
			typedef struct cell_index
			{
				unsigned int width_idx;
				unsigned int height_idx;
			}stCellIdx;

			ros::Subscriber sub_plan_costmap;		// 経路コストマップのサブスクライバ
			ros::Subscriber sub_my_plan_info;   	// 自己経路情報のサブスクライバ
    		costmap_2d::Costmap2D *buf_costmap;		// masterへ反映用のバッファ
			nav_msgs::OccupancyGrid plan_costmap;	// 他ロボット経路コストマップ
			dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

			std::string entityId; 							// getParamで取得する他ロボットの固有名詞
			std::vector<stCellIdx> *vct_plan_costmap_cell; 	// 経路コストマップの高コストのセルの幅及び高さを格納する

			unsigned char cost_trans_table[101]; 	// コストマップへ反映するコストの変換テーブル
			unsigned int plan_costmap_width;		// 経路コストマップのセルの幅
			unsigned int plan_costmap_height;		// 経路コストマップのセルの高さ
			unsigned int plan_costmap_width_min;	// 経路コストマップのセルの更新する幅の最小値
			unsigned int plan_costmap_width_max;	// 経路コストマップのセルの更新する幅の最大値
			unsigned int plan_costmap_height_min;	// 経路コストマップのセルの更新する高さの最小値
			unsigned int plan_costmap_height_max;	// 経路コストマップのセルの更新する高さの最大値
			double plan_costmap_resolution;			// 経路コストマップの解像度
			double plan_costmap_origin_x;			// グローバルフレーム内のマップのxの原点をメートル単位で指定する
			double plan_costmap_origin_y;			// グローバルフレーム内のマップのyの原点をメートル単位で指定する
			bool stay_costmap;						// 経路コストマップの反映フラグ
			bool update_costmap;					// 経路コストマップのアップデート監視フラグ
	
			void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
			void recvCostmap(const nav_msgs::OccupancyGridConstPtr& msg);
			void recvMyPlan(const nav_msgs::Path &msg);
			void convertCost(void);
			bool checkCostmapInfo(costmap_2d::Costmap2D grid_data);
	};
}
#endif

