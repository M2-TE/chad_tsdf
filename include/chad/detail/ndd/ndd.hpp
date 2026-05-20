#pragma once

namespace ndd {
    struct Descriptor {
        static constexpr uint32_t N_RINGS = 20;
        static constexpr uint32_t N_SECTORS = 60;
        static constexpr float MAX_RADIUS = 80.0f;
        static constexpr uint32_t N_POINTS_THRESHHOLD = 5; // min points per cell
        static constexpr double SEARCH_RATIO = 0.1; // 10% of alignment key entries as search range (-5%, +5%)
        using Ring = std::array<float, N_SECTORS>;
        using Sector = std::array<float, N_RINGS>;
        using CellMatrix = std::array<Ring, N_RINGS>;
        using LookupKey = Sector;
        using AlignmentKey = Ring;

        // TODO: prevent CV from being non-inversible (see original paper)
        Descriptor(const std::vector<glm::aligned_vec3>& points, const glm::aligned_vec3& position) {
            // reset all cells to 0
            for (auto& ring: _cells) ring.fill(0.0f);

            // gather all points belonging to a given cell
            std::array<std::array<std::vector<glm::aligned_vec3>, N_SECTORS>, N_RINGS> cell_points;

            // add points to their respective cells
            for (const auto& point_ref: points) {
                glm::aligned_vec3 point = point_ref - position; // make sure points are centered to position
                // calc position of point on circular 2D plane in format of azimuth angle/distance from center
                const float azim_range = std::sqrt(point.x * point.x + point.y * point.y); // TODO: wouldnt squared range be sufficient?
                if (azim_range > MAX_RADIUS) continue; // TODO: this could easily handle axim_range being squared

                float azim_angle = 0.0f;
                if      (point.x >= 0.0f && point.y >= 0.0f) azim_angle =   0.0f + 180.0f / std::numbers::pi * std::atan(+point.y / +point.x);
                else if (point.x <  0.0f && point.y >= 0.0f) azim_angle = 180.0f - 180.0f / std::numbers::pi * std::atan(+point.y / -point.x);
                else if (point.x <  0.0f && point.y <  0.0f) azim_angle = 180.0f + 180.0f / std::numbers::pi * std::atan(+point.y / +point.x);
                else if (point.x >= 0.0f && point.y <  0.0f) azim_angle = 360.0f - 180.0f / std::numbers::pi * std::atan(-point.y / +point.x);

                // convert azimuth angle/range into ring/sector indices
                uint32_t ring_i = std::ceil(azim_range / MAX_RADIUS * float(N_RINGS)); // TODO: this could handle squared range
                uint32_t sector_i = std::ceil(azim_angle / 360.0f * float(N_SECTORS));

                // TODO: are the clamps even necessary?
                ring_i = std::clamp<uint32_t>(ring_i, 1, N_RINGS); // TODO: possible to just adjust clamps instead of -1?
                sector_i = std::clamp<uint32_t>(sector_i, 1, N_SECTORS);

                // add point to the corresponding cell
                cell_points[ring_i - 1][sector_i - 1].push_back(point);
            }

            for (uint32_t ring_i = 0; ring_i < N_RINGS; ring_i++) {
                for (uint32_t sector_i = 0; sector_i < N_SECTORS; sector_i++) {
                    // need at least 5 points
                    if (cell_points[ring_i][sector_i].size() < N_POINTS_THRESHHOLD) continue;

                    // retrieve cell containing all the points
                    std::vector<glm::aligned_vec3>& cell = cell_points[ring_i][sector_i];

                    // mean point μ (calc in double precision as points count can be high with large individual positions)
                    glm::aligned_dvec3 pointd_mean{ 0, 0, 0 };
                    for (const auto& point: cell) {
                        pointd_mean += glm::aligned_dvec3{ point };
                    }
                    pointd_mean /= double(cell.size());
                    const glm::aligned_vec3 point_mean = glm::aligned_vec3{ pointd_mean };

                    // store centered points back into cell points
                    for (auto& point: cell) {
                        point -= point_mean;
                    }

                    // calculate covariance matrix cv via "cell * transpose(cell) / (cell.size() - 1)" with cell being treated as a 3xN matrix
                    glm::aligned_mat3x3 cv = glm::zero<glm::aligned_mat3x3>();
                    for (uint32_t row = 0; row < cell.size(); row++) {
                        // diagonal
                        cv[0][0] += double(cell[row].x * cell[row].x);
                        cv[1][1] += double(cell[row].y * cell[row].y);
                        cv[2][2] += double(cell[row].z * cell[row].z);
                        cv[0][1] += double(cell[row].x * cell[row].y);
                        cv[0][2] += double(cell[row].x * cell[row].z);
                        cv[1][2] += double(cell[row].y * cell[row].z);
                    }
                    cv *= 1.0 / double(cell.size() - 1);
                    cv[1][0] = cv[0][1];
                    cv[2][0] = cv[0][2];
                    cv[2][1] = cv[1][2];

                    cv = glm::inverse(cv);

                    // gauss = exp( cell * cv * transpose(cell) * -0.5 )
                    double gaussian_trace = 0.0;
                    for (uint32_t diag = 0; diag < cell.size(); diag++) {
                        float val = 0.0f;
                        glm::aligned_vec3 cell_cv;
                        cell_cv.x = cell[diag].x * cv[0].x + cell[diag].y * cv[1].x + cell[diag].z * cv[2].x;
                        cell_cv.y = cell[diag].x * cv[0].y + cell[diag].y * cv[1].y + cell[diag].z * cv[2].y;
                        cell_cv.z = cell[diag].x * cv[0].z + cell[diag].y * cv[1].z + cell[diag].z * cv[2].z;

                        val += cell[diag].x * cell_cv.x;
                        val += cell[diag].y * cell_cv.y;
                        val += cell[diag].z * cell_cv.z;
                        val *= -0.5;
                        gaussian_trace += std::exp(double(val));
                    }
                    // write final pd sum to ring/sector cell
                    _cells[ring_i][sector_i] = gaussian_trace;
                }
            }

            // create alignment key
            for (uint32_t sector_i = 0; sector_i < Descriptor::N_SECTORS; sector_i++) {
                // sum of single sector (column)
                float sum = 0.0f;
                for (const auto& ring: _cells) {
                    sum += ring[sector_i];
                }
                _alignment_key[sector_i] = sum;
            }
        }
        Descriptor() = default;
        ~Descriptor() = default;
        // construct the lookup key for this descriptor
        auto get_lookup_key() const -> LookupKey {
            LookupKey key;
            for (uint32_t ring_i = 0; ring_i < Descriptor::N_RINGS; ring_i++) {
                // sum of single ring (row)
                float sum = 0.0f;
                for (const auto& sector: _cells[ring_i]) {
                    sum += sector;
                }
                key[ring_i] = sum;
            }
            return key;
        }
        // estimate correlation with another descriptor, returning the optimal correlation and column-shift
        auto estimate_correlation(const Descriptor& other) const -> std::pair<double, uint32_t> {
            // get the best shift of desc_b for alignment and make search indices around it
            uint32_t shift_best = get_alignment(other._alignment_key);
            const static uint32_t search_distance = std::round(0.5 * SEARCH_RATIO * double(N_SECTORS)); // TODO: make constexpr
            std::vector<uint32_t> shift_indices{ shift_best };
            // get indices around "best shift" as the search radius
            for (uint32_t i = 1; i <= search_distance; i++) {
                shift_indices.push_back((shift_best + N_SECTORS + i) % N_SECTORS);
                shift_indices.push_back((shift_best + N_SECTORS - i) % N_SECTORS);
            }

            // use the shift indices to compare the actual descriptors
            uint32_t max_shift = 0;
            double max_correlation = 0.0;
            // precalc: mean_a and mean_b
            const double mean_a = get_mean();
            const double mean_b = other.get_mean();
            // precalc: dot(_cells_a[...][...] - mean_a, _cells_b[...][...] - mean_b)
            const double dot_a = get_centered_dot(mean_a);
            const double dot_b = other.get_centered_dot(mean_b);
            const double dot_a_b_recip = 1.0 / std::sqrt(dot_a * dot_b);

            // calc coefficient correlation between desc_a and circular-shifted desc_b
            for (auto& shift_i: shift_indices) {
                // sum of dot product between centered descriptors (shifted desc_b)
                double dot_ab = 0.0;
                for (uint32_t ring_i = 0; ring_i < Descriptor::N_RINGS; ring_i++) {
                    const auto& ring_a = _cells[ring_i];
                    const auto& ring_b = other._cells[ring_i];

                    uint32_t ring_a_sector = 0;
                    // instead of creating shifted copy of ring_b, just offset the index for it
                    for (uint32_t ring_b_sector = shift_i; ring_b_sector < ring_b.size(); ring_b_sector++) {
                        float cell_a_cen = ring_a[ring_a_sector++] - mean_a;
                        float cell_b_cen = ring_b[ring_b_sector] - mean_b;
                        dot_ab += double(cell_a_cen * cell_b_cen);
                    }
                    if (shift_i == 0) continue;
                    for (uint32_t ring_b_sector = 0; ring_b_sector < shift_i; ring_b_sector++) {
                        float cell_a_cen = ring_a[ring_a_sector++] - mean_a;
                        float cell_b_cen = ring_b[ring_b_sector] - mean_b;
                        dot_ab += double(cell_a_cen * cell_b_cen);
                    }
                }

                // calculate the correlation between the two descriptors
                const double correlation = std::abs(dot_ab * dot_a_b_recip);
                if (correlation > max_correlation) {
                    max_correlation = correlation;
                    max_shift = shift_i;
                }
            }

            return std::make_pair(max_correlation, max_shift);
        }

    private:
        // get mean of all cells
        auto get_mean() const -> double {
            // use double precision for calc, as values can get quite large
            double mean = 0.0;
            for (const auto& ring: _cells) {
                for (const auto& cell: ring) {
                    mean += double(cell);
                }
            }
            mean *= 1.0 / double(N_RINGS * N_SECTORS);
            return mean;
        }
        // get shift with best match between the two alignment keys
        auto get_alignment(const AlignmentKey& other) const -> uint32_t {
            uint32_t shift_min = 0;
            double norm_min = std::numeric_limits<double>::max();

            // perform circular shift on second key
            for (uint32_t shift_i = 0; shift_i < uint32_t(_alignment_key.size()); shift_i++) {
                double norm_sqr = 0.0; // calc the dot product of the "difference between the keys" with itself
                uint32_t alignment_key_i = 0; // keep separate index for key_a

                // instead of creating a shifted copy of key_b, just shift its index
                for (uint32_t other_i = shift_i; other_i < uint32_t(_alignment_key.size()); other_i++) {
                    double diff = other[other_i] - _alignment_key[alignment_key_i++];
                    norm_sqr += diff * diff;
                }

                if (shift_i > 0) {
                    // start at beginning of key_b and go to shift_i - 1
                    for (uint32_t other_i = 0; other_i < shift_i; other_i++) {
                        double diff = other[other_i] - _alignment_key[alignment_key_i++];
                        norm_sqr += diff * diff;
                    }
                }

                // remember shift with best alignment
                if (norm_sqr < norm_min) {
                    shift_min = shift_i;
                    norm_min = norm_sqr;
                }
            }
            return shift_min;
        };
        // get dot with itself after centering each cell (cell[...][...] - mean)
        auto get_centered_dot(double mean) const -> double {
            // use double precision for calc, as values can get quite large
            double dot = 0.0;
            for (const auto& ring: _cells) {
                for (const auto& cell: ring) {
                    double cell_offset = double(cell) - mean;
                    dot += cell_offset * cell_offset;
                }
            }
            return dot;
        }

    public:
        CellMatrix _cells;
        AlignmentKey _alignment_key;
    };
};
