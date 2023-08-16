// jsonOperations.hpp

#pragma once

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "pathfinding.hpp"
#include <map>

using nlohmann::json;
using RoomData = json;

RoomData readJsonFile(const std::string& filepath);
RoomDoorMapping mapDoorsToRooms(const RoomData& data);
std::map<std::string, std::tuple<pf::Point, pf::Point>> mapRoomsInOut(const RoomData& data);
