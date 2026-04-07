#pragma once

/**
 * @file types.hpp
 * @brief 配送系统公共类型定义。
 *
 * 将 Pose2D、Station、StationMap 统一定义在此，
 * 避免 delivery_manager 和 BT 节点各自重复定义导致类型系统不互通。
 */

#include <cstdint>
#include <string>
#include <unordered_map>

namespace delivery_core
{

/// 2D 位姿，用于仓库地图里的站点坐标存储。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 站点描述，包含站点 ID、位姿和站点类型。
struct Station
{
  std::string id;
  Pose2D pose;
  uint8_t type{0};   ///< 0=取货点, 1=送货点, 2=充电桩
};

/// 站点映射表类型别名，便于从黑板按 ID 快速查找站点。
using StationMap = std::unordered_map<std::string, Station>;

}  // namespace delivery_core
