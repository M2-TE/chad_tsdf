#define VULKAN_HPP_USE_REFLECT
#include <bit>
#include <vulkan/vulkan.hpp>
#include "chad/vk/device/selector.hpp"

namespace dv {
auto Selector::select_physical_device(vk::Instance instance, vk::SurfaceKHR surface) -> vk::PhysicalDevice {
    // enumerate devices
    auto phys_devices = instance.enumeratePhysicalDevices();
    if (phys_devices.size() == 0) fmt::println("No device with vulkan support found");

    // create a std::set from the required extensions
    std::set<std::string> required_extensions {
        _required_extensions.cbegin(),
        _required_extensions.cend()
    };
    
    // check for matching devices (bool = is_preferred, vk::DeviceSize = memory_size)
    std::vector<std::pair<vk::PhysicalDevice, vk::DeviceSize>> matching_devices;
    fmt::println("Available devices:");
    for (vk::PhysicalDevice device: phys_devices) {
        auto props = device.getProperties();
        bool passed = true;
        passed &= check_api_ver(props);
        passed &= check_extensions(required_extensions, device);
        passed &= check_core_features(device);
        passed &= check_presentation(device, surface);

        // add device candidate if it passed tests
        if (passed) {
            vk::DeviceSize memory_size = 0;
            auto memory_props = device.getMemoryProperties();
            for (auto& heap: memory_props.memoryHeaps) {
                if (heap.flags & vk::MemoryHeapFlagBits::eDeviceLocal) {
                    memory_size += heap.size;
                }
            }
            if (props.deviceType == _preferred_device_type) memory_size += 1ull << 63ull;
            matching_devices.emplace_back(device, memory_size);
        }
        std::string pass_str = passed ? "passed" : "failed";
        fmt::println("-> {}: {}", pass_str, (const char*)props.deviceName);
    }

    // optionally bail out
    if (matching_devices.size() == 0) {
        fmt::println("None of the devices match the requirements");
        exit(0);
    }

    // sort devices by favouring certain gpu types and large local memory heaps
    typedef std::pair<vk::PhysicalDevice, vk::DeviceSize> DeviceEntry;
    auto fnc_sorter = [&](DeviceEntry& a, DeviceEntry& b) {
        return a.second > b.second;
    };
    std::sort(matching_devices.begin(), matching_devices.end(), fnc_sorter);
    vk::PhysicalDevice phys_device = std::get<0>(matching_devices.front());
    fmt::println("Picked device: {}", (const char*)phys_device.getProperties().deviceName);
    return phys_device;
}
auto Selector::create_logical_device(vk::PhysicalDevice physical_device, void* additional_features_p) -> std::pair<vk::Device, std::vector<uint32_t>> {
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
bool Selector::check_api_ver(vk::PhysicalDeviceProperties& props) {
    bool passed = true;
    if (vk::apiVersionMajor(props.apiVersion) < _required_major) passed = false;
    if (vk::apiVersionMinor(props.apiVersion) < _required_minor) passed = false;
    if (!passed) fmt::println("\tMissing vulkan {}.{} support", _required_major, _required_minor);
    return passed;
}
bool Selector::check_extensions(std::set<std::string> required_extensions, vk::PhysicalDevice physical_device) {
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
bool Selector::check_core_features(vk::PhysicalDevice phys_device) {
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
bool Selector::check_presentation(vk::PhysicalDevice physical_device, vk::SurfaceKHR surface) {
    // check if presentation capabilties are required
    if (surface == nullptr) return true;

    // create dummy queues to check presentation capabilities
    auto [queue_infos, _] = create_queue_infos(physical_device);
    vk::Bool32 passed = false;
    // check each queue family for presentation capabilities
    for (auto& info: queue_infos) {
        vk::Bool32 b = physical_device.getSurfaceSupportKHR(info.queueFamilyIndex, surface);
        passed |= b;
    }

    if (!passed) fmt::print("Missing presentation capabilities");
    return passed;
}
auto Selector::create_queue_infos(vk::PhysicalDevice physical_device) -> std::pair<std::vector<vk::DeviceQueueCreateInfo>, std::vector<uint32_t>> {
    // <queue family index, count of queue capabilities>
    typedef std::pair<uint32_t, uint32_t> QueueCount;
    std::vector<std::vector<QueueCount>> queue_family_counters;
    queue_family_counters.resize(_required_queues.size());

    // iterate through all queue families to store relevant ones
    auto queue_families = physical_device.getQueueFamilyProperties();
    for (uint32_t i = 0; i < queue_families.size(); i++) {
        vk::QueueFlags flags = queue_families[i].queueFlags;
        uint32_t capability_count = std::popcount((uint32_t)flags);
        // check if this queue family works for each requested queue
        for (size_t q = 0; q < _required_queues.size(); q++) {
            vk::QueueFlags masked = flags & _required_queues[q];
            if (masked == _required_queues[q]) {
                queue_family_counters[q].emplace_back(i, capability_count);
            }
        }
    }

    // track unique queue families
    std::set<uint32_t> unique_queue_families;
    // contains queue family indices for requested queues
    std::vector<uint32_t> queue_mappings(_required_queues.size());
    // set up queue create infos with unique family indices
    std::vector<vk::DeviceQueueCreateInfo> info_queues;
    for (size_t i = 0; i < _required_queues.size(); i++) {
        // sort queues by capability count
        auto& vec = queue_family_counters[i];
        std::sort(vec.begin(), vec.end(), [](QueueCount& a, QueueCount& b){
            return a.second < b.second;
        });
        // use the queue with the fewest capabilities possible
        uint32_t queue_family_index = vec.front().first;

        // map the requested queue
        queue_mappings[i] = queue_family_index;

        // create new queue family info if unique
        auto [unique_it, unique_b] = unique_queue_families.emplace(queue_family_index);
        if (!unique_b) continue;
        info_queues.push_back({
            .queueFamilyIndex = queue_family_index,
            .queueCount = 1,
            .pQueuePriorities = &queue_priority,
        });
    }
    return std::make_pair(info_queues, queue_mappings);
}
} // namespace dv