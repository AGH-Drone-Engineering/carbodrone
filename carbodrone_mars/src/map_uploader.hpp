#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>

class MapUploader
{
public:
    struct Object
    {
        Object(std::string name, double latitude, double longitude)
            : name(std::move(name)), latitude(latitude), longitude(longitude)
        {
        }

        const std::string name;
        const double latitude;
        const double longitude;
    };

    void upload_map(const std::vector<Object> &objects)
    {
        std::string json_string = "[";
        for (const auto& object : objects)
        {
            json_string += "{\"name\":\"" + object.name + "\",\"lat\":" + std::to_string(object.latitude) + ",\"lon\":" + std::to_string(object.longitude) + "},";
        }
        if (!objects.empty())
        {
            json_string.pop_back();
        }
        json_string += "]";

        std::string command = "python3 " + std::string(MAP_UPLOADER_PROGRAM_PATH) + " \"" + json_string + "\"";
        // std::cout << "Executing command: " << command << std::endl;
        system(command.c_str());
    }
};
