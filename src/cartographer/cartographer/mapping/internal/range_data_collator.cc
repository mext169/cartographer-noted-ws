/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

/**
 * @brief 多个雷达数据的时间同步
 * 
 * @param[in] sensor_id 雷达数据的话题
 * @param[in] timed_point_cloud_data 雷达数据
 * @return sensor::TimedPointCloudOriginData 根据时间处理之后的数据
 */
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) { // 第一次拷贝
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);

  // 从sensor_bridge传过来的数据的intensities为空
  timed_point_cloud_data.intensities.resize(timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);

  // TODO(gaschler): These two cases can probably be one.
  // 待处理的数据中有当前同话题的点云，就把待处理的先处理掉，然后将当前的点云保存
  // current_end_ 一开始为上一次时间同步的结束时间，会更新成本次同步的结束时间
  // current_start_为本次时间同步的开始时间
  // current_start_ 和 current_end_ 保存的是一帧点云中最后一个点的绝对时间
  if (id_to_pending_data_.count(sensor_id) != 0) {

    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    // 本次时间同步的结束时间为这帧点云数据的结束时间（一组点云的最后一个点的绝对时间）
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    return result;
  }

  // 先将当前点云添加到 等待时间同步的map中
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
  // expected_sensor_ids_中所有话题都有数据了才进行同步
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }

  // 
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  // 找到所有传感器数据中最早的时间戳(点云最后一个点的时间)
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  // current_end_是本次时间同步的结束时间
  // 是待时间同步map中的 所有点云中最早的时间戳
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

// 对时间段内的数据进行截取与合并, 返回时间同步后的点云
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {

  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  // 遍历所有的传感器话题
  for (auto it = id_to_pending_data_.begin(); it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second; // data.time为点云中最后一个点的绝对时间
    const sensor::TimedPointCloud& ranges = it->second.ranges; // 里面存了一堆点 ranges[i].time为每一个点相对于最后用个点的时间，为负的
    const std::vector<float>& intensities = it->second.intensities;

    // 找到点云中 第一个时间戳大于等于current_start_的点的索引
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() && data.time + common::FromSeconds((*overlap_begin).time) < current_start_) {
      ++overlap_begin;
    }
    // 找到点云中 最后一个时间戳小于等于current_end_的点的索引
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() && data.time + common::FromSeconds((*overlap_end).time) <= current_end_) {
      ++overlap_end;
    }

    // 丢弃点云中时间比起始时间早的点, 每执行一下CropAndMerge()打印一次log
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin) << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      // 获取下个点云的index, 即当前vector的个数
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);  // 插入原点坐标

      // 获取此传感器时间与集合时间戳的误差, 
      const float time_correction = static_cast<float>(common::ToSeconds(data.time - current_end_));

      auto intensities_overlap_it = intensities.begin() + (overlap_begin - ranges.begin());
      // reserve() 在预留空间改变时, 会将之前的数据拷贝到新的内存中
      result.ranges.reserve(result.ranges.size() + std::distance(overlap_begin, overlap_end));
      
      // 填充数据
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end; ++overlap_it, ++intensities_overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        // 针对每个点时间戳进行修正, 让最后一个点的时间为0
        point.point_time.time += time_correction;  
        result.ranges.push_back(point);
      } // end for
    } // end if

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) { // 如果点云每个点都用了, 则可将这个数据进行删除
      it = id_to_pending_data_.erase(it);
    } 
    else if (overlap_end == ranges.begin()) { // 如果一个点都没用, 就先放这, 看下一个数据
      ++it;
    } 
    else { // 用了一部分的点
      const auto intensities_overlap_end = intensities.begin() + (overlap_end - ranges.begin());
      // 将用了的点删除, 这里的赋值是拷贝
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),/*截取后面一段*/
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  } // end for

  // 对各传感器的点云 按照每个点的时间从小到大进行排序
  // 这里按照时间排序使用的是相对最后一个点的时间 因此 前面在构造result.ranges是最后一个点的时间是按照current_end_来的
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a, const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
