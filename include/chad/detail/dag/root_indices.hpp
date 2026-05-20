#pragma once

namespace chad::detail::dag {
    using RootIndex = std::uint32_t;

    struct RootIndices {
        RootIndices() = default;
        // allow construction from std::pair of uints
        RootIndices(std::pair<RootIndex, RootIndex> indices): _tsdfs(indices.first), _weights(indices.second) {
        }
        // allow conversion to std::pair of uints
        operator std::pair<RootIndex, RootIndex>() const {
            return { _tsdfs, _weights };
        }

        RootIndex _tsdfs;
        RootIndex _weights;
    };
}
