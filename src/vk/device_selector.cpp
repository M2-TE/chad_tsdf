#define VULKAN_HPP_USE_REFLECT
#include <vulkan/vulkan.hpp>
#include "vk/device_selector.hpp"

auto DeviceSelector::create_logical_device(vk::PhysicalDevice physical_device, void* additional_features_p) -> std::pair<vk::Device, std::vector<uint32_t>> {
    // set up features
    vk::PhysicalDeviceFeatures2 required_features {
        .features = _required_features,
    };
    // chain main features
    void** tail_pp = &required_features.pNext;
    if (_required_minor >= 1) {
        *tail_pp = &_required_vk11_features;
        tail_pp = &_required_vk11_features.pNext;
    }
    if (_required_minor >= 2) {
        *tail_pp = &_required_vk12_features;
        tail_pp = &_required_vk12_features.pNext;
    }
    if (_required_minor >= 3) {
        *tail_pp = &_required_vk13_features;
        tail_pp = &_required_vk13_features.pNext;
    }

    // chain additional features
    *tail_pp = additional_features_p;

    // enable optional extensions if available
    auto available_extensions = physical_device.enumerateDeviceExtensionProperties();
    for (const char* ext: _optional_extensions) {
        for (auto& available: available_extensions) {
            if (strcmp(ext, available.extensionName) == 0) {
                _required_extensions.push_back(ext);
                break;
            }
        }
    }
    
    // create device
    auto [info_queues, queue_mappings] = create_queue_infos(physical_device);
    vk::DeviceCreateInfo info_device {
        .pNext = &required_features,
        .queueCreateInfoCount = (uint32_t)info_queues.size(),
        .pQueueCreateInfos = info_queues.data(),
        .enabledExtensionCount = (uint32_t)_required_extensions.size(),
        .ppEnabledExtensionNames = _required_extensions.data(),
    };

    vk::Device device = physical_device.createDevice(info_device);
    return std::make_pair(device, queue_mappings);
}

bool DeviceSelector::check_api_ver(vk::PhysicalDeviceProperties& props) {
    bool passed = true;
    if (vk::apiVersionMajor(props.apiVersion) < _required_major) passed = false;
    if (vk::apiVersionMinor(props.apiVersion) < _required_minor) passed = false;
    if (!passed) fmt::println("\tMissing vulkan {}.{} support", _required_major, _required_minor);
    return passed;
}
bool DeviceSelector::check_extensions(std::set<std::string> required_extensions, vk::PhysicalDevice physical_device) {
    auto ext_props = physical_device.enumerateDeviceExtensionProperties();
    for (auto& extension: ext_props) {
        std::string ext_name = extension.extensionName;
        auto ext_it = required_extensions.find(ext_name);
        // erase from set if found
        if (ext_it != required_extensions.end()) {
            required_extensions.erase(ext_it);
        }
    }
    // print missing extensions, if any
    for (auto& extension: required_extensions) {
        fmt::println("\tMissing device extension: {}", extension);
    }
    // pass if all required extensions were erased
    bool passed = required_extensions.size() == 0;
    return passed;

}

template<typename T>
bool check_features(T& available, T& required) {
    // track available features via simple vector
    std::vector<vk::Bool32> available_features;
    auto fnc_add_available = [&](auto arg) {
        if constexpr (std::is_same_v<decltype(arg), vk::Bool32>) {
            available_features.push_back(arg);
        }
    };
    std::apply([&](auto&... args) {
        ((fnc_add_available(args)), ...);
    }, available.reflect());

    // track required features via simple vector
    std::vector<vk::Bool32> required_features;
    auto fnc_add_required = [&](auto arg) {
        if constexpr (std::is_same_v<decltype(arg), vk::Bool32>) {
            required_features.push_back(arg);
        }
    };
    std::apply([&](auto&... args) {
        ((fnc_add_required(args)), ...);
    }, required.reflect());

    // check if all required features are available
    bool passed = true;
    for (size_t i = 0; i < available_features.size(); i++) {
        if (required_features[i] && !available_features[i]) passed = false;
    }
    return passed;
}
bool DeviceSelector::check_core_features(vk::PhysicalDevice phys_device) {
    auto features = phys_device.getFeatures2<
        vk::PhysicalDeviceFeatures2,
        vk::PhysicalDeviceVulkan11Features,
        vk::PhysicalDeviceVulkan12Features,
        vk::PhysicalDeviceVulkan13Features>();
    bool passed = true;
    passed &= check_features(features.get<vk::PhysicalDeviceFeatures2>().features, _required_features);
    passed &= check_features(features.get<vk::PhysicalDeviceVulkan11Features>(), _required_vk11_features);
    passed &= check_features(features.get<vk::PhysicalDeviceVulkan12Features>(), _required_vk12_features);
    passed &= check_features(features.get<vk::PhysicalDeviceVulkan13Features>(), _required_vk13_features);
    return passed;
}