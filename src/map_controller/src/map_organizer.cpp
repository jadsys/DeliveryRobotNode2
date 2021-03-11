/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include <nav_msgs/Path.h>
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

#define OBSTACLE_COST			100
#define MOVING_BODY_COST		120
#define ADDITIONAL_AREA_COST	-100

/**
 * @brief Map Organization node.（0. 他ロボット情報なし）
 */
class MapOrganizer
{

  public:
	std::string mapname_;
	ros::Subscriber egomap_sub_;
	ros::Subscriber sociomap_sub_;
	ros::Subscriber costmap_sub_;
    ros::Subscriber costmap_update_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    nav_msgs::MapMetaData meta_data_message_;
	nav_msgs::OccupancyGrid current_egomap_;
	nav_msgs::OccupancyGrid current_sociomap_;
	nav_msgs::OccupancyGrid current_costmap_;
	unsigned short *manage_out_cost_;
	unsigned short *manage_exist_cost_;
	nav_msgs::Path path_;
    int pub_count_;
    int write_wait_count_;
	bool initial_saved_egomap_;
	bool initial_saved_sociomap_;

	int costmap_updates_wait_count_;
	double program_cycle_frequency_;
	int publish_cycle_count_;
	int cost_output_count_;
	int cost_existence_check_count_;

    MapOrganizer(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
		ros::NodeHandle n;
		ros::NodeHandle source_node("~");
//      ROS_INFO("Waiting for the costmap");
		egomap_sub_ = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &MapOrganizer::incomingEgoMap, this);
		sociomap_sub_ = n.subscribe<nav_msgs::OccupancyGrid>("/map_movebase", 1, &MapOrganizer::incomingSocioMap, this);
		costmap_sub_ = n.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &MapOrganizer::incomingCostmap, this);
		costmap_update_sub_ = n.subscribe<map_msgs::OccupancyGridUpdate>("/move_base/global_costmap/costmap_updates", 1, &MapOrganizer::incomingCostmapUpdates, this);
		pub_count_=0;
		initial_saved_egomap_ = false;
		initial_saved_sociomap_ = false;

		if (source_node.getParam("costmap_updates_wait_count", costmap_updates_wait_count_)){
			ROS_INFO("costmap_updates_wait_count (%d)", costmap_updates_wait_count_);
		}else{
			costmap_updates_wait_count_ = 5;
		}
		write_wait_count_ = costmap_updates_wait_count_;

		if (source_node.getParam("program_cycle_frequency", program_cycle_frequency_)){
			ROS_INFO("program_cycle_frequency (%lf)", program_cycle_frequency_);
		}else{
			program_cycle_frequency_ = 10.0;//(hz)
		}

		if (source_node.getParam("publish_cycle_count", publish_cycle_count_)){
			ROS_INFO("publish_cycle_count (%d)", publish_cycle_count_);
		}else{
			publish_cycle_count_ = 10;
		}

		if (source_node.getParam("cost_output_count", cost_output_count_)){
			ROS_INFO("cost_output_count (%d)", cost_output_count_);
		}else{
			cost_output_count_ = 10;
		}

		if (source_node.getParam("cost_existence_check_count", cost_existence_check_count_)){
			ROS_INFO("cost_existence_check_count (%d)", cost_existence_check_count_);
		}else{
			cost_existence_check_count_ = 2;
		}


    }

	~MapOrganizer()
	{
		delete(manage_out_cost_);
		delete(manage_exist_cost_);
	}

	void incomingEgoMap(const nav_msgs::OccupancyGridConstPtr& msg)
	{
		if ( !initial_saved_egomap_ )
		{
			current_egomap_ = *msg;

			ROS_INFO("Received a %d X %d egomap @ %.3f m/pix",
				current_egomap_.info.width,
				current_egomap_.info.height,
				current_egomap_.info.resolution);

			ROS_INFO("Egomap has been saved. \n");
			initial_saved_egomap_ = true;
		}
#if 0
		else
		{
			ROS_INFO("Egomap already saved. \n");
		}
#endif
	}

	void incomingSocioMap(const nav_msgs::OccupancyGridConstPtr& msg)
	{
		if ( initial_saved_egomap_)
		{
			if ( !initial_saved_sociomap_ )
			{
				current_sociomap_ = *msg;

				ROS_INFO("Received a %d X %d sociomap @ %.3f m/pix",
					current_sociomap_.info.width,
					current_sociomap_.info.height,
					current_sociomap_.info.resolution);

				manage_out_cost_ = new unsigned short[current_sociomap_.info.width * current_sociomap_.info.height];
				manage_exist_cost_ = new unsigned short[current_sociomap_.info.width * current_sociomap_.info.height];

				for(unsigned int i = 0; i < (current_sociomap_.info.width * current_sociomap_.info.height); i++) {
						manage_out_cost_[i] = cost_output_count_;
						manage_exist_cost_[i] = cost_existence_check_count_;
				}

				for(unsigned int i = 0; i < (current_sociomap_.info.width * current_sociomap_.info.height); i++) {
					if (current_egomap_.data[i] == 0 && current_sociomap_.data[i] == OBSTACLE_COST) {
						current_sociomap_.data[i] = ADDITIONAL_AREA_COST;
					}
				}
#if 0
				ROS_INFO("Sociomap has been saved. \n");
#endif
				initial_saved_sociomap_ = true;
				write_wait_count_ = costmap_updates_wait_count_;
			}
			else
			{
			ROS_INFO("SocioMap already saved. \n");
			write_wait_count_ = costmap_updates_wait_count_;
			}
		}
		else
		{
			ROS_INFO("Waiting for egomap reception. \n");
		}
	}

    void incomingCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
    {
		current_costmap_ = *msg;
#if 0
		ROS_INFO("Received a %d X %d costmap @ %.3f m/pix",
			current_costmap_.info.width,
			current_costmap_.info.height,
			current_costmap_.info.resolution);

		ROS_INFO("Costmap has been saved. \n");
#endif
		pub_count_++;
	}

    void incomingCostmapUpdates(const map_msgs::OccupancyGridUpdateConstPtr& update)
    {
		if ( write_wait_count_ != 0 )
		{
			ROS_INFO("Write_wait_count is %d", write_wait_count_);
			write_wait_count_--;
			pub_count_++;
		}
		else
		{
#if 0
			ROS_INFO("Write_wait_count is %d", write_wait_count_);
			ROS_INFO("Received a %d X %d costmap_updates",
				update->width,
				update->height);
#endif
			// Reject updates which have any out-of-bounds data.
			if (update->x < 0 || update->y < 0 || current_costmap_.info.width < update->x + update->width ||
				current_costmap_.info.height < update->y + update->height
				)
			{
				ROS_INFO("Update area outside of original map area.");
				return;
			}

			// Copy the incoming data into current_costmap_'s data.
			for (size_t y = 0; y < update->height; y++)
			{
				memcpy(&current_costmap_.data[(update->y + y) * current_costmap_.info.width + update->x],
					&update->data[y * update->width], update->width);
			}
#if 0
			ROS_INFO("Costmap_updates has been saved. \n");
#endif
			pub_count_++;
		}
	}

	void publishMap()
	{
		if (pub_count_ != publish_cycle_count_)
		{
			return;
		}

		ROS_INFO("Publish a %d X %d costmap @ %.3f m/pix",
			current_sociomap_.info.width,
			current_sociomap_.info.height,
			current_sociomap_.info.resolution);

		for(unsigned int y = 0; y < current_sociomap_.info.height; y++) {
			for(unsigned int x = 0; x < current_sociomap_.info.width; x++) {
				unsigned int i = x + (current_sociomap_.info.height - y - 1) * current_sociomap_.info.width;
				// 最新のコスト値が0〜99（コスト0、内接コスト、外接コスト）
				if (current_costmap_.data[i] >= 0 && current_costmap_.data[i] <= threshold_free_) {
					// 対象セルの直前コストがUnknownコストでない場合
					if (current_sociomap_.data[i] != -1)
					{
						// コストをクリアする
						current_sociomap_.data[i] = 0;
						manage_out_cost_[i] = cost_output_count_;
						manage_exist_cost_[i] = cost_existence_check_count_;
					}
				// 最新のコスト値が100以上（準障害物コスト、侵入禁止コスト、固定障害物コスト）
				} else if (current_costmap_.data[i] >= threshold_occupied_) {
					// 対象セルの直前コストがUnknownコストでない場合
					if (current_sociomap_.data[i] != -1)
					{
						// 対象セルの直前コストが侵入禁止コストでない場合
						if (current_sociomap_.data[i] != ADDITIONAL_AREA_COST )
						{
							// 対象セルの直前コストが障害物あるいは準障害物コストでない場合
							if ( current_sociomap_.data[i] < OBSTACLE_COST )
							{
								// 対象セルの存在確認カウントが「0」の場合
								if (manage_exist_cost_[i] == 0 ){
									// 対象セルに準障害物コストを設定し、出力カウントをMAXに設定する
									// （sociomapに反映する）
									current_sociomap_.data[i] = MOVING_BODY_COST;
								}
								// 対象セルの存在確認カウントが「0」より大きい場合
								else
								{
									// 対象セルの存在確認カウントを「1」減少させる
									// （sociomapには反映しない）
									manage_exist_cost_[i] -=1;
								}
							}
							// 対象セルの直前コストが障害物あるいは準障害物コストである場合
							else
							{
								// 対象セルの直前コストが準障害物コストである場合
								if ( current_sociomap_.data[i] == MOVING_BODY_COST )
								{
									// 対象セルの出力カウントが「0」の場合
									if ( manage_out_cost_[i] == 0 )
									{
										// コストをクリアする
										// （sociomapから削除する）
										current_sociomap_.data[i] = 0;
										manage_out_cost_[i] = cost_output_count_;
										manage_exist_cost_[i] = cost_existence_check_count_;
#if 0
										ROS_INFO(" cost cleared.");
#endif
									}
									// 対象セルの出力カウントが「0」より大きい場合
									else
									{
										// 出力カウントを「1」減少させる
										// （sociomapに反映し続ける）
										manage_out_cost_[i] -= 1;
#if 0
										ROS_INFO(" cost decreased.");
#endif
									}
								}
								// 対象セルの直前コストが障害物コストである場合
								else
								{
									// 障害物コストのセルに対しては何もしない
									;
								}
							}
						}
						// 対象セルの直前コストが侵入禁止コストの場合
						else
						{
							// 侵入禁止コストのセルに対しては何もしない
							;
						}
					}
				// 最新のコスト値が-1以下（Unknownコスト）
				} else {
					// 対象セルの直前コストがUnknownコストでない場合
					if (current_sociomap_.data[i] != -1)
					{
						// コストをUnknownコストとする
						current_sociomap_.data[i] = -1;
						manage_out_cost_[i] = cost_output_count_;
						manage_exist_cost_[i] = cost_existence_check_count_;
					}
				}
			}
		}

		ros::Time::waitForValid();
		current_sociomap_.info.map_load_time = ros::Time::now();
		current_sociomap_.header.frame_id = "map";
		current_sociomap_.header.stamp = ros::Time::now();

		meta_data_message_ = current_sociomap_.info;

		// Latched publisher for metadata
		metadata_pub= n.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);
		metadata_pub.publish( meta_data_message_ );

		// Latched publisher for data
		map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_movebase", 1, true);
		map_pub.publish( current_sociomap_ );

		ROS_INFO("Map_original has been published. \n");
//              ros::Rate rate(0.5);
//              rate.sleep();
		saved_map_ = false;
		pub_count_ = 0;

	}

};

#define USAGE "Usage: \n" \
              "  map_organizer -h\n"\
			  "  map_organizer [--occ <threshold_occupied>] [--free <threshold_free>] [ROS remapping args]"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_organizer");
	std::string mapname = "map";
	int threshold_occupied = 65;
	int threshold_free = 25;

	for(int i=1; i<argc; i++)
	{
		if(!strcmp(argv[i], "-h"))
		{
			puts(USAGE);
			return 0;
		}
		else if (!strcmp(argv[i], "--occ"))
		{
			if (++i < argc)
			{
				threshold_occupied = std::atoi(argv[i]);
				if (threshold_occupied < 1 || threshold_occupied > 100)
				{
					ROS_ERROR("threshold_occupied must be between 1 and 100");
					return 1;
				}
			}
			else
			{
				puts(USAGE);
				return 1;
			}
		}
		else if (!strcmp(argv[i], "--free"))
		{
			if (++i < argc)
			{
				threshold_free = std::atoi(argv[i]);
				if (threshold_free < 0 || threshold_free > 100)
				{
					ROS_ERROR("threshold_free must be between 0 and 100");
					return 1;
				}
			}
			else
			{
				puts(USAGE);
				return 1;
			}
		}
		else
		{
			puts(USAGE);
			return 1;
		}
	}

	if (threshold_occupied <= threshold_free)
	{
		ROS_ERROR("threshold_free must be smaller than threshold_occupied");
		return 1;
	}

	MapOrganizer mg(mapname, threshold_occupied, threshold_free);

	ros::Rate rate(mg.program_cycle_frequency_);
	while(!mg.saved_map_ && ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		mg.publishMap();
	}
	return 0;
}
