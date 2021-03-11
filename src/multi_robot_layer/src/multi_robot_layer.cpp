#include <multi_robot_layer/multi_robot_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(multi_robot_layer_namespace::MultiRobotLayer, costmap_2d::Layer)

namespace multi_robot_layer_namespace
{
	MultiRobotLayer::MultiRobotLayer() 
	{
		// do nothing
	}

	MultiRobotLayer::~MultiRobotLayer() 
	{
		delete(buf_costmap);
	}

	void MultiRobotLayer::onInitialize(void)
	{
		// ノードハンドル宣言 
		ros::NodeHandle nh("~/" + name_);
		ros::NodeHandle source_node("~"); //param読み込み用ノードハンドル

		current_ = true;
		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&MultiRobotLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);


		// 固有IDを取得
		if (source_node.getParam("entity_id", entityId))
		{
            ROS_INFO("entity_id (%s)", entityId.c_str());
        }
		else
		{
            entityId = DEFAULT_ROBOT_ID;
        }
		
		// 経路コストマップのサブスクライバ 
		sub_plan_costmap	= nh.subscribe<nav_msgs::OccupancyGrid>("/" + entityId + "/plan_costmap", ROS_QUEUE_SIZE, &MultiRobotLayer::recvCostmap, this);	// delivery_robot_nodeからのplan_costmapトピックメッセージのサブスクライバ
		
		// 経路情報受信
        sub_my_plan_info 	= nh.subscribe("/" + entityId + "/move_base/NavfnROS/plan", ROS_QUEUE_SIZE, &MultiRobotLayer::recvMyPlan, this);
       
		// コストマップテーブル作成
        memset( &cost_trans_table, 0, sizeof(cost_trans_table));
        cost_trans_table[0] 	= FREESPACE_COST_COSTMAP2D;  	// フリースペース
        cost_trans_table[99]	= INSCRIBED_COST_COSTMAP2D;  	// 内接コスト
        cost_trans_table[100]	= OBSTACLE_COST_COSTMAP2D;  	// 障害物コスト

        for (int i = 1; i < 99; i++)
        {
            cost_trans_table[ i ] = u_char(1 + (251 * (i - 1)) / 97);
        }

		stay_costmap		= false;	// コストマップ反映中フラグ
		update_costmap		= false;	//　経路コストマップの更新フラグ

		// コストマップの初期化 
        buf_costmap = new costmap_2d::Costmap2D();	// 経路コストマップの保持用

		// コストマップも境界値情報の初期化
		plan_costmap_width_min	= 1e6;	// 初期値
		plan_costmap_height_min	= 1e6;	// 初期値
		plan_costmap_width_max	= 0;	// 初期値
		plan_costmap_height_max	= 0;	// 初期値

		return;
	}

	void MultiRobotLayer::recvMyPlan(const nav_msgs::Path &msg)
	{
		unsigned int my_plan_size   = msg.poses.size();

		// if(my_plan_size <= 10 )
		// {
		// 	ROS_INFO("stay_costmap_false");
		// 	stay_costmap = false;
		// }
		
		return;
	}

	void MultiRobotLayer::recvCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
	{
		ROS_INFO("subscrive_plan_costmap_msg");

		// クラスメンバ
		plan_costmap			= *msg;
		
		plan_costmap_width		= plan_costmap.info.width;
		plan_costmap_height		= plan_costmap.info.height;
		plan_costmap_origin_x	= plan_costmap.info.origin.position.x;
		plan_costmap_origin_y	= plan_costmap.info.origin.position.y;
		plan_costmap_resolution = (double)plan_costmap.info.resolution;

		ROS_INFO("original costmap frame_id (%s)",	plan_costmap.header.frame_id.c_str());
		ROS_INFO("original costmap cell_x (%d)",	plan_costmap.info.width);
		ROS_INFO("original costmap cell_y (%d)",	plan_costmap.info.height);
		ROS_INFO("original costmap resolution (%fl)",plan_costmap.info.resolution);
		ROS_INFO("original costmap position.x (%fl)", plan_costmap.info.origin.position.x);
		ROS_INFO("original costmap position.y (%fl)", plan_costmap.info.origin.position.y);
		ROS_INFO("original costmap position.z (%fl)", plan_costmap.info.origin.position.z);
		ROS_INFO("original costmap orientation.x (%fl)", plan_costmap.info.origin.orientation.x);
		ROS_INFO("original costmap orientation.y (%fl)", plan_costmap.info.origin.orientation.y);
		ROS_INFO("original costmap orientation.z (%fl)", plan_costmap.info.origin.orientation.z);
		ROS_INFO("original costmap orientation.w (%fl)", plan_costmap.info.origin.orientation.w);

		// コストマップをリサイズ
		buf_costmap->resizeMap(plan_costmap.info.width, plan_costmap.info.height, plan_costmap.info.resolution, plan_costmap.info.origin.position.x, plan_costmap.info.origin.position.y);

		update_costmap = true;
		
		stay_costmap = true;
	
		return;
	}

	void MultiRobotLayer::convertCost(void)
	{
		unsigned int cost 				= 0;	//設定するコスト
        unsigned int cost_table_idx 	= 0;	//コスト変換テーブルのインデックス
		unsigned int costmap_x_idx 		= 0;	//コストマップのセルの幅のインデックス
		unsigned int costmap_y_idx 		= 0; 	//コストマップのセルの高さのインデックス

		for(int idx = 0; idx < plan_costmap.data.size(); idx++)
		{
			if(	plan_costmap.data[idx] != FREESPACE_COST_GRIDMAP &&  // FREESPACE = 0
				plan_costmap.data[idx] != UNKNOWN_COST_GRIDMAP)		 // UNKNOWN	  = -1
			{
				buf_costmap->indexToCells(idx, costmap_x_idx, costmap_y_idx);	// RAM上のインデックスに相当する、セルのインデックスへ変換

				// 更新範囲の境界値更新
				plan_costmap_width_min	= std::min(costmap_x_idx, plan_costmap_width_min);
				plan_costmap_height_min	= std::min(costmap_y_idx, plan_costmap_height_min);
				plan_costmap_width_max	= std::max(costmap_x_idx, plan_costmap_width_max);
				plan_costmap_height_max	= std::max(costmap_y_idx, plan_costmap_height_max);

				// コスト情報変換 
				cost_table_idx 	= plan_costmap.data[idx];    // 経路コストマップのコスト値を取り出す

				cost 			= cost_trans_table[cost_table_idx];    //コスト変換テーブルを参照し、コストの値(-1~100)に対応する値(0~255)を取り出す

				buf_costmap->setCost(costmap_x_idx, costmap_y_idx, cost); // バッファコストマップのセルへコストをコピー
			}
			else
			{
				continue;
			}
		}


		return;
	}

	bool MultiRobotLayer::checkCostmapInfo(costmap_2d::Costmap2D grid_data)
    {
        bool isMatchInfo = true; // 経路コストマップの情報が不一致でfalse

        // チェック1：ソシオ地図の幅とコストマップの幅
        if(grid_data.getSizeInCellsX() != plan_costmap_width)
        {
            isMatchInfo = false;
        }

        // チェック2：ソシオ地図の高さとコストマップの高さ
        if(grid_data.getSizeInCellsY() != plan_costmap_height)
        {
            isMatchInfo = false;
        }

        // チェック3：ソシオ地図の解像度はコストマップの解像度
        if(fabsf(grid_data.getResolution() - plan_costmap_resolution) > FLT_EPSILON) // OccupancyGridのメッセージはfloat型の為,最大誤差はfloatの計算機イプシロンとなる
        {
            isMatchInfo = false;
        }

		// チェック4：ソシオ地図の解像度はコストマップの原点（x）
        if(fabs(grid_data.getOriginX() - plan_costmap_origin_x) > DBL_EPSILON)
        {
            isMatchInfo = false;
        }

		// チェック5：ソシオ地図の解像度はコストマップの原点（y）
        if(fabs(grid_data.getOriginY() - plan_costmap_origin_y) > DBL_EPSILON)
        {
            isMatchInfo = false;
        }

        return(isMatchInfo);
    }

	void MultiRobotLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
		
		return;
	}

	void MultiRobotLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
								   double* min_x,double* min_y, double* max_x, double* max_y)
	{

		double wx, wy; // セルに相当する座標値

		if (!enabled_)
		{
			return;
		}


		if(stay_costmap)
		{
			if (update_costmap)
			{ // 経路コストマップの更新が合った場合
				convertCost();	// OccupancyGrid→Costmap2Dへコストを変換する
				update_costmap = false;
			}

			// 境界値の最小最大値を反映
			buf_costmap->mapToWorld(plan_costmap_width_min, plan_costmap_height_min, wx, wy); //x,yのセル座標を/mapフレームの座標（距離）へ直す
			*min_x = std::min(wx, *min_x);
			*min_y = std::min(wy, *min_y);

			buf_costmap->mapToWorld(plan_costmap_width_max, plan_costmap_height_max, wx, wy);
			*max_x = std::max(wx, *max_x);
			*max_y = std::max(wy, *max_y);
		}
		else
		{
			// コスト情報反映終了後に境界値をリセット
			plan_costmap_width_min	= 1e6;
			plan_costmap_height_min	= 1e6;
			plan_costmap_width_max	= 0;
			plan_costmap_height_max	= 0;

			buf_costmap->resetMap(0, 0, buf_costmap->getSizeInCellsX(), buf_costmap->getSizeInCellsY());

		}

		return;
	}

	void MultiRobotLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
	{

		unsigned char *plan_costmap_data 		= buf_costmap->getCharMap();	// 経路コストマップのデータ
		unsigned char *master_costmap_data 		= master_grid.getCharMap();		// master_gridのデータ
		unsigned int master_cell_x_idx			= 0; // master_gridのセルの幅
		unsigned int master_cell_y_idx			= 0; // master_gridのセルの高さ
		unsigned int plan_costmap_cost			= 0; // 経路コストマップのコスト値
		unsigned int master_costmap_cost		= 0; // master_gridのコスト値
		unsigned int sum_cost					= 0; // コストの合計値
		unsigned int plan_costmap_total_cost 	= 0; // 経路コストマップのコストの合計値

		if (!enabled_)
		{
			return;
		}

		if(stay_costmap)
		{ 
			if(checkCostmapInfo(master_grid)) // plan_costmapの地図情報検証
			{ // 地図情報が正しい場合

				for(int idx = 0; idx < plan_costmap.data.size(); idx++)
				{
					
					// インデックスに相当するmaster_gridのコストマップセルの幅、高さを求める 
					master_grid.indexToCells(idx, master_cell_x_idx, master_cell_y_idx);

					// UNKNOWN領域は予めセットしておく 
					if( plan_costmap.data[idx] == UNKNOWN_COST_GRIDMAP)
					{
						master_grid.setCost(master_cell_x_idx, master_cell_y_idx,  UNKNOWN_COST_COSTMAP2D);
						continue;
					}

					// planコストマップのコスト値が0（コスト無し）の場合はスキップ 
					if( plan_costmap.data[idx] == FREESPACE_COST_COSTMAP2D)
					{
						continue;
					}

					// 元々のmasterの障害物等のコストは無視する
					if(	master_costmap_data[idx] == OBSTACLE_COST_COSTMAP2D ||	// 254 
						master_costmap_data[idx] == INSCRIBED_COST_COSTMAP2D )	// 253 既に253の場合は以下計算の必要がない為省略
					{
						continue;
					}

					// コストの合算
					plan_costmap_cost = (unsigned int)cost_trans_table[plan_costmap.data[idx]];
					master_costmap_cost = (unsigned int)master_costmap_data[idx];
					sum_cost = plan_costmap_cost + master_costmap_cost;
					
					// コストの合算値が253より大きい場合、253に置き換える 
					if(sum_cost > PLAN_COST_MAX)
					{
						sum_cost = PLAN_COST_MAX;
					}

					// 経路コストマップのコスト値の合計を計算
					plan_costmap_total_cost = plan_costmap_total_cost + plan_costmap_cost;

					// 合算値の反映 
					master_grid.setCost(master_cell_x_idx, master_cell_y_idx,  (unsigned char)sum_cost);
				}

				// 空のコストマップは1度のみ反映
				if(plan_costmap_total_cost == 0)
				{
					stay_costmap = false;
				}

			}
		}

		return;
	}

} // end namespace
