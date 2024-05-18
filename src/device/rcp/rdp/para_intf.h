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
    void video_render(void);
    void screen_swap(bool blank);
    void init_framebuffer(int width, int height);
#ifdef __cplusplus
}
#endif