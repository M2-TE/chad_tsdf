#pragma once
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/map/octree2.hpp"

namespace chad::detail::funcs {
    // Match points to tsdf data. "tsdf_data" is the new submap with an error that needs to be approximated. Sampled points are from older DAG trees
    auto inline match_points_to_tsdf(
            const map::Octree2<17, 2>& tsdf_data, Pose tsdf_err_guess, double sdf_res,
            const std::vector<glm::aligned_vec3>& points_data, Pose points_pose, Pose points_err) -> Pose{
        // prepare matrix for error transform (taking into account both points_data AND tsdf_data error)
        glm::aligned_dmat4x4 point_err_transform = glm::mat4_cast(points_err._rotation * tsdf_err_guess._rotation);
        point_err_transform = glm::translate(point_err_transform, - points_err._position);
        double sdf_res_reciprocal = 1.0 / sdf_res;

        // set up H and g for jacobian later
        std::array<double, 6> g{};
        std::array<std::array<double, 6>, 6> H{};

        // get gradients around every sampled point
        for (const auto& point_raw: points_data) {
            // make sure points are centered around [0|0|0]
            glm::aligned_dvec3 point_centered = static_cast<glm::aligned_dvec3>(point_raw) - points_pose._position;
            // apply error transform
            glm::aligned_dvec3 point = static_cast<glm::aligned_dvec3>(point_err_transform * glm::aligned_dvec4{ point_centered, 1.0 });
            // translate back
            point += points_pose._position;
            // consider initial guess for translation error
            point -= tsdf_err_guess._position;

            // get tsdf voxel at current point
            glm::aligned_ivec3 point_voxel{ glm::floor(point * sdf_res_reciprocal) };
            auto tsdf_opt = tsdf_data.find(MortonCode{ point_voxel });
            if (!tsdf_opt.has_value()) continue;
            float tsdf = tsdf_opt->_signed_distance * sdf_res_reciprocal;

            // build gradients along each axis
            bool gradient_valid = true;
            glm::aligned_dvec3 gradient{ 0, 0, 0 };
            for (std::uint8_t axis_i = 0; axis_i < 3; axis_i++) {
                glm::aligned_ivec3 neigh_pos_a = point_voxel;
                glm::aligned_ivec3 neigh_pos_b = point_voxel;
                neigh_pos_a[axis_i]--;
                neigh_pos_b[axis_i]++;

                // get gradient neighbours
                auto tsdf_opt_a = tsdf_data.find(MortonCode{ neigh_pos_a });
                auto tsdf_opt_b = tsdf_data.find(MortonCode{ neigh_pos_b });

                // reject when one of the tsdf voxels is missing
                if (!tsdf_opt_a.has_value() || !tsdf_opt_b.has_value()) {
                    gradient_valid = false;
                    break;
                } // TODO: dont break, instead just skip this axis?

                // grab the actual tsdf values
                double tsdf_a = static_cast<double>(tsdf_opt_a->_signed_distance) * sdf_res_reciprocal;
                double tsdf_b = static_cast<double>(tsdf_opt_b->_signed_distance) * sdf_res_reciprocal;

                // if (tsdf_a != 0.0 && tsdf_b != 0.0 && (tsdf_a > 0.0f) == (tsdf_b > 0.0f)) {
                //     // gradient[axis_i] = (tsdf_b - tsdf_a) / (2.0 * _sdf_res);
                //     gradient[axis_i] = (tsdf_b - tsdf_a) / 2.0;
                // }
                gradient[axis_i] = (tsdf_b - tsdf_a) / (2.0);
            }
            if (!gradient_valid) continue;

            // cross product point x gradient
            std::array<double, 6> jacobian;
            jacobian[0] = point_centered[1] * gradient[2] - point_centered[2] * gradient[1];
            jacobian[1] = point_centered[2] * gradient[0] - point_centered[0] * gradient[2];
            jacobian[2] = point_centered[0] * gradient[1] - point_centered[1] * gradient[0];
            jacobian[3] = gradient[0];
            jacobian[4] = gradient[1];
            jacobian[5] = gradient[2];

            // add multiplication result to H
            for (std::uint8_t row = 0; row < 6; row++) {
                for (std::uint8_t col = 0; col < 6; col++) {
                    // H += jacobian * jacobian.transpose()
                    H[row][col] += jacobian[row] * jacobian[col];
                }
                g[row] += jacobian[row] * tsdf;
            }
        }

        // lu_decomposition
        for (std::uint8_t i = 0; i < 6 - 1; i++) {
            for (std::uint8_t k = i + 1; k < 6; k++) {
                H[k][i] /= H[i][i];
                for (std::uint8_t j = i + 1; j < 6; j++) {
                    H[k][j] -= H[k][i] * H[i][j];
                }
            }
        }
        // lu_solve
        std::array<double, 6> xi;
        for (int i = 0; i < 6; i++) {
            xi[i] = g[i];
            for (int k = 0; k < i; k++) {
                xi[i] -= H[i][k] * xi[k];
            }
        }
        for (int i = 6 - 1; i >= 0; i--) {
            for (int k = i + 1; k < 6; k++) {
                xi[i] -= H[i][k] * xi[k];
            }
            xi[i] /= H[i][i];
        }

        // xi contains both translation and rotation (euler)
        glm::aligned_dvec3 rot{ xi[0], xi[1], xi[2] };
        glm::aligned_dvec3 pos{ xi[3], xi[4], xi[5] };
        return Pose{ pos * sdf_res, rot };
    }
}
