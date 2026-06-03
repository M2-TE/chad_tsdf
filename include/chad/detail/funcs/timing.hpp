#pragma once

namespace {
    void inline CHAD_MESSAGE(std::string_view message) {
        fmt::println("[CHAD] {}", message);
    }
    void inline MEASURE_TIME(std::chrono::steady_clock::time_point beg, std::string_view message) {
        double dur = std::chrono::duration<double, std::milli>{ std::chrono::steady_clock::now() - beg }.count();
        fmt::println("[CHAD] {}: {:.2f}ms", message, dur);
    }
}
