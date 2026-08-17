#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/map/indices.hpp"

namespace chad::detail::map {
    struct SubSubmap {
        Pose                          _pose;
        DescriptorIndex               _descriptor_i;
        std::vector<ndd::Correlation> _correlations;
    };
}
