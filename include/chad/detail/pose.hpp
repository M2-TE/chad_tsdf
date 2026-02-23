#pragma once

namespace chad::detail {
    struct Pose {
        // default constructor with identity rotation
        Pose(): _position(0, 0, 0), _rotation(glm::identity<decltype(_rotation)>()){}
        // standard constructor
        Pose(glm::dvec3 position, glm::dquat rotation): _position(position), _rotation(rotation) {}
        // standard constructor with euler rotation
        Pose(glm::dvec3 position, glm::dvec3 rotation_euler): _position(position), _rotation(rotation_euler) {}
        
        // constructor from std::arrays
        Pose(std::array<float, 3> position, std::array<float, 4> rotation): 
            _position(position[0], position[1], position[2]), 
            _rotation(rotation[0], rotation[1], rotation[2], rotation[3]) {}
        // constructor from std::arrays
        Pose(std::array<double, 3> position, std::array<double, 4> rotation): 
            _position(position[0], position[1], position[2]), 
            _rotation(rotation[0], rotation[1], rotation[2], rotation[3]) {}

        // the chad::detail::Pose struct is equivalent to this std::pair
        using PairType = std::pair<std::array<double, 3>, std::array<double, 4>>;
        // constructor from std::pair of std::arrays
        Pose(PairType pose): 
            _position(pose.first[0], pose.first[1], pose.first[2]), 
            _rotation(pose.second[0], pose.second[1], pose.second[2], pose.second[3]) {}
        // allow conversion to std::pair of std::arrays
        operator PairType() const {
            return {
                { _position.x, _position.y, _position.z },
                { _rotation.x, _rotation.y, _rotation.z, _rotation.w }
            };
        }

        glm::dvec3 _position;
        glm::dquat _rotation;
    };
}