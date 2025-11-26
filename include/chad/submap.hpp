#pragma once
#include <array>
#include <vector>
#include <cstdint>

namespace chad {
    // Pose representing a position and rotation
    struct Pose {
        template<typename T>
        auto inline get_position() const -> T {
            return { _position[0], _position[1], _position[2] };
        }
        template<typename T>
        auto inline get_rotation() const -> T {
            return { _rotation[0], _rotation[1], _rotation[2], _rotation[3] };
        }

        std::array<float, 3> _position{ 0, 0, 0 }; // 3D xyz
        std::array<float, 4> _rotation{ 0, 0, 0, 1 }; // quaternion
    };

    // Single submap
    struct Submap {
        using Handle = uint32_t; // handle to a specific submap stored in map
        using RootAddr = uint32_t; // address to root node of hashed tree
        struct Roots {
            RootAddr _tsdfs = 0;
            RootAddr _weights = 0;
        };
        Submap(): _roots({ 0, 0 }) {
        }

        void clear() {
            _roots = {};
            _pose_avg = {};
            _poses.clear();
        }

        void update_pose() {
            // calc average position of submap
            std::array<double, 3> position;
            for (const auto& pose: _poses) {
                position[0] += double(pose._position[0]);
                position[1] += double(pose._position[1]);
                position[2] += double(pose._position[2]);
            }
            position[0] /= double(_poses.size());
            position[1] /= double(_poses.size());
            position[2] /= double(_poses.size());
            _pose_avg._position[0] = float(position[0]);
            _pose_avg._position[1] = float(position[1]);
            _pose_avg._position[2] = float(position[2]);

            // // TODO: rotation
            // glm::quat rotation = glm::identity<glm::quat>();
            // _pose_avg._rotation[0] = rotation.x;
            // _pose_avg._rotation[1] = rotation.y;
            // _pose_avg._rotation[2] = rotation.z;
            // _pose_avg._rotation[3] = rotation.w;
        }

        Roots _roots;
        Pose _pose_avg;
        Pose _pose_err;
        std::vector<Pose> _poses;
    };
}
