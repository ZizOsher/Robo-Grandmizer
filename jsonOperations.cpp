// jsonOperations.cpp

#include "jsonOperations.hpp"

RoomData readJsonFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open the file!");
    }

    RoomData jsonData;
    file >> jsonData;
    return jsonData;
}

RoomDoorMapping mapDoorsToRooms(const RoomData& data) {
    RoomDoorMapping mapping;
    for (const auto& item : data.items()) {
        const auto& roomName = item.key();
        const auto& doorData = item.value()["door"];

        pf::Point startPoint = {doorData["start"]["x"], doorData["start"]["y"]};
        pf::Point endPoint = {doorData["end"]["x"], doorData["end"]["y"]};

        std::tuple<pf::Point, pf::Point> door = std::make_tuple(startPoint, endPoint);
        mapping[door] = roomName;
    }
    return mapping;
}

std::map<std::string, std::tuple<pf::Point, pf::Point>> mapRoomsInOut(const RoomData& data) {
    std::map<std::string, std::tuple<pf::Point, pf::Point>> roomPointsMap;

    for (const auto& item : data.items()) {
        const auto& roomName = item.key();
        const auto& inpointData = item.value()["inpoint"];
        const auto& outpointData = item.value()["outpoint"];

        pf::Point inpoint = {inpointData["x"], inpointData["y"]};
        pf::Point outpoint = {outpointData["x"], outpointData["y"]};

        roomPointsMap[roomName] = std::make_tuple(inpoint, outpoint);
    }

    return roomPointsMap;
}
