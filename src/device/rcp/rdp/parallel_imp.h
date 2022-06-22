
#include "m64p_plugin.h"
#include "m64p_common.h"
#include "gfxstructdefs.h"

#ifdef __cplusplus
extern "C"
{
#endif

    extern int32_t vk_rescaling;
    extern bool vk_ssreadbacks;
    extern bool vk_ssdither;

    extern unsigned vk_overscan;
    extern unsigned vk_vertical_stretch;
    extern unsigned vk_downscaling_steps;
    extern bool vk_native_texture_lod;
    extern bool vk_native_tex_rect;
    extern bool vk_divot_filter, vk_gamma_dither;
    extern bool vk_vi_aa, vk_vi_scale, vk_dither_filter;
    extern bool vk_interlacing;
    extern bool vk_synchronous;
    extern bool skip_swap_clear;

extern bool libretro_swap_buffer;
extern uint32_t* prescale;
#ifdef __cplusplus
}
#endif
