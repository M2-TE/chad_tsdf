#pragma once

namespace chad::detail {
    struct Pose {
        Pose() = default;
        Pose(glm::vec3 position, glm::quat rotation): _position(position), _rotation(rotation) {}
        // allow construction from std::arrays
        Pose(std::array<float, 3> position, std::array<float, 4> rotation): 
            _position(position[0], position[1], position[2]), 
            _rotation(rotation[0], rotation[1], rotation[2], rotation[3]) {}
        // allow construction from std::pair of std::arrays
        Pose(std::pair<std::array<float, 3>, std::array<float, 4>> pose): 
            _position(pose.first[0], pose.first[1], pose.first[2]), 
            _rotation(pose.second[0], pose.second[1], pose.second[2], pose.second[3]) {}
        // allow conversion to std::pair of std::arrays
        operator std::pair<std::array<float, 3>, std::array<float, 4>>() const {
            return {
                { _position.x, _position.y, _position.z },
                { _rotation.x, _rotation.y, _rotation.z, _rotation.w }
            };
        }

        glm::vec3 _position;
        glm::quat _rotation;
    };
}