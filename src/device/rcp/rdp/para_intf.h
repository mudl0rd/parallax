#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include <stdbool.h>
    void vk_rasterize(void);
    void vk_process_commands(void);
    bool vk_init(void);
    void vk_destroy(void);
    void screen_swap(bool blank);
    int retro_width;
    int retro_height;
    int retro_pitch;
#ifdef __cplusplus
}
#endif