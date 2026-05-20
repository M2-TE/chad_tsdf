#pragma once

namespace chad::detail::funcs {
    // pilfered from HATSDF
    void lu_decomposition(std::array<std::array<double, 6>, 6>& H) {
        for (int i = 0; i < 6 - 1; i++) {
            for (int k = i + 1; k < 6; k++) {
                H[k][i] /= H[i][i];
                for (int j = i + 1; j < 6; j++) {
                    H[k][j] -= H[k][i] * H[i][j];
                }
            }
        }
    }
    // pilfered from HATSDF
    auto lu_solve(const std::array<std::array<double, 6>, 6>& H, const std::array<double, 6>& g) -> std::array<double,6> {
        std::array<double, 6> x;
        for (int i = 0; i < 6; i++) {
            x[i] = g[i];
            for (int k = 0; k < i; k++) {
                x[i] -= H[i][k] * x[k];
            }
        }
        for (int i = 6 - 1; i >= 0; i--) {
            for (int k = i + 1; k < 6; k++) {
                x[i] -= H[i][k] * x[k];
            }
            x[i] /= H[i][i];
        }
        return x;
    }
}
