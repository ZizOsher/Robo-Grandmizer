// jsonOperations.hpp

#pragma once

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "pathfinding.hpp"
#include <map>

using nlohmann::json;
using RoomData = json;
using namespace Pathfinding;
namespace pf = Pathfinding;

// using RoomDoorMapping = std::map<std::tuple<pf::Point, pf::Point>, std::string>;

RoomData readJsonFile(const std::string& filepath);
std::map<std::tuple<pf::Point, pf::Point>, std::string> mapDoorsToRoomNames(const RoomData& data);
std::map<std::string, std::tuple<pf::Point, pf::Point>> mapRoomNameToInOut(const RoomData& data);
