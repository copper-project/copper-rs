#include <cstddef>

#include <sl/Camera.hpp>

extern "C" void* sl_mat_create_alias(
    int width,
    int height,
    int type,
    int mem,
    void* data,
    std::size_t step_bytes
) {
    if (width <= 0 || height <= 0 || data == nullptr || step_bytes == 0) {
        return nullptr;
    }

    try {
        return static_cast<void*>(new sl::Mat(
            static_cast<std::size_t>(width),
            static_cast<std::size_t>(height),
            static_cast<sl::MAT_TYPE>(type),
            static_cast<sl::uchar1*>(data),
            step_bytes,
            static_cast<sl::MEM>(mem + 1)
        ));
    } catch (...) {
        return nullptr;
    }
}
