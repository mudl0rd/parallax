#include "parallel_imp.h"
#include <memory>
#include <vector>
#include "rdp_device.hpp"
#include "context.hpp"
#include "device.hpp"
#include "gfxstructdefs.h"
#include "para_intf.h"
#include "Gfx #1.3.h"
#include "common.h"
#include "gfxstructdefs.h"
#include "glad.h"
#ifdef _WIN32
#include <windows.h>
#endif

unsigned rdram_size = 8 * 1024 * 1024;

using namespace Vulkan;
using namespace std;

static int cmd_cur;
static int cmd_ptr;
static uint32_t cmd_data[0x00040000 >> 2];

static unique_ptr<RDP::CommandProcessor> frontend;
static unique_ptr<Device> device;
static unique_ptr<Context> context;

int32_t vk_rescaling;
bool vk_ssreadbacks;
bool vk_ssdither;
bool running = false;
unsigned width, height;

bool skip_swap_clear;
static bool vk_initialized;

#include <immintrin.h>

inline unsigned long rotatel(unsigned long X, int C)
{
	return (X << C) | (X >> ((sizeof(long) * 8) - C));
}

static const unsigned cmd_len_lut[64] = {
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	4,
	6,
	12,
	14,
	12,
	14,
	20,
	22,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	2,
	2,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
};

struct shader_id
{
	int fsid;
	int vsid;
	unsigned int pid;
};
#define TEX_NUM 3
static GLuint vao = 0;
static GLuint buffer_pbo;
shader_id program = {0};
static GLuint texture[TEX_NUM];
static uint8_t *buffer_data;
static uint32_t buffer_size = (640 * 8) * (480 * 8) * sizeof(uint32_t);
int32_t tex_width[TEX_NUM];
int32_t tex_height[TEX_NUM];
static int rotate_buffer;

#include <libretro.h>
extern struct retro_hw_render_callback hw_render;
#define SHADER_HEADER "#version 330 core\n"
const GLchar *vert_shader =
	SHADER_HEADER
	"out vec2 uv;\n"
	"void main(void) {\n"
	"uv = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
	"gl_Position = vec4(uv * vec2(2.0, -2.0) + vec2(-1.0, 1.0), 0.0, 1.0);\n"
	"}\n";
const GLchar *frag_shader =
	SHADER_HEADER
	"in vec2 uv;\n"
	"layout(location = 0) out vec4 color;\n"
	"uniform sampler2D tex0;\n"
	"void main(void) {\n"
	"color = texture(tex0, uv);\n"
	"}\n";

shader_id initShader(const char *vsh, const char *fsh)
{
	shader_id shad = {0};
	shad.vsid = glCreateShaderProgramv(GL_VERTEX_SHADER, 1, &vsh);
	shad.fsid = glCreateShaderProgramv(GL_FRAGMENT_SHADER, 1, &fsh);
	glGenProgramPipelines(1, &shad.pid);
	glBindProgramPipeline(shad.pid);
	glUseProgramStages(shad.pid, GL_VERTEX_SHADER_BIT, shad.vsid);
	glUseProgramStages(shad.pid, GL_FRAGMENT_SHADER_BIT, shad.fsid);
	glBindProgramPipeline(0);
	return shad;
}

void init_framebuffer(int width, int height)
{
	if (program.pid)
		glDeleteProgramPipelines(1, &program.pid);
	program = initShader(vert_shader, frag_shader);
	glBindProgramPipeline(program.pid);
	glGenVertexArrays(1, &vao);
	glGenTextures(TEX_NUM, &texture[0]);
	for (int i = 0; i < TEX_NUM; ++i)
	{
		glBindTexture(GL_TEXTURE_2D, texture[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	glGenBuffers(1, &buffer_pbo);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffer_pbo);
	glBufferStorage(GL_PIXEL_UNPACK_BUFFER, buffer_size * TEX_NUM, 0, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT);
	buffer_data = glMapBufferRange(GL_PIXEL_UNPACK_BUFFER, 0, buffer_size * TEX_NUM, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT);
}

static inline unsigned get_alignment(unsigned pitch)
{
	if (pitch & 1)
		return 1;
	if (pitch & 2)
		return 2;
	if (pitch & 4)
		return 4;
	return 8;
}

uint8_t *screen_get_texture_data()
{
	return buffer_data + (rotate_buffer * buffer_size);
}

void screen_write(int width, int height)
{
	bool buffer_size_changed = tex_width[rotate_buffer] != width || tex_height[rotate_buffer] != height;
	char *offset = (char *)(rotate_buffer * buffer_size);
	glBindTexture(GL_TEXTURE_2D, texture[rotate_buffer]);
	// check if the framebuffer size has changed
	if (buffer_size_changed)
	{
		int retro_pitch = width * sizeof(uint32_t);
		tex_width[rotate_buffer] = width;
		tex_height[rotate_buffer] = height;
		// set pitch for all unpacking operations
		glPixelStorei(GL_UNPACK_ALIGNMENT, get_alignment(retro_pitch));
		glPixelStorei(GL_UNPACK_ROW_LENGTH, retro_pitch);
		// reallocate texture buffer on GPU
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, tex_width[rotate_buffer],
					 tex_height[rotate_buffer], 0, GL_RGBA, GL_UNSIGNED_BYTE, offset);
	}
	else
	{
		// copy local buffer to GPU texture buffer
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tex_width[rotate_buffer], tex_height[rotate_buffer],
						GL_RGBA, GL_UNSIGNED_BYTE, offset);
	}
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, hw_render.get_current_framebuffer());
	glViewport(0,0,640,480);
	glScissor(0,0,640,480);
	glBindProgramPipeline(program.pid);
	glActiveTexture(GL_TEXTURE0);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	glBindVertexArray(0);
	glBindProgramPipeline(0);
	glBindFramebuffer(GL_FRAMEBUFFER, hw_render.get_current_framebuffer());
	rotate_buffer = (rotate_buffer + 1) % TEX_NUM;
}

void vk_blit(unsigned &width, unsigned &height)
{
	if (running)
	{

		RDP::ScanoutOptions opts = {};
		opts.persist_frame_on_invalid_input = true;
		opts.crop_rect.top = true;
		opts.crop_rect.bottom = true;
		opts.crop_rect.enable = true;
		opts.vi.aa = true;
		opts.downscale_steps = 2;

		RDP::VIScanoutBuffer scanout;
		frontend->scanout_async_buffer(scanout, opts);

		if (!scanout.width || !scanout.height)
		{
			width = 0;
			height = 0;
			return;
		}
		width = scanout.width;
		height = scanout.height;
		scanout.fence->wait();
		uint8_t *color_data = screen_get_texture_data();
		memcpy(color_data, device->map_host_buffer(*scanout.buffer, Vulkan::MEMORY_ACCESS_READ_BIT),
			   width * height * sizeof(uint32_t));
		device->unmap_host_buffer(*scanout.buffer, Vulkan::MEMORY_ACCESS_READ_BIT);

		screen_write(width, height);
	}
}

void vk_rasterize()
{

	if (!frontend)
	{
		device->next_frame_context();
		return;
	}

	if (frontend && running)
	{
		frontend->set_vi_register(RDP::VIRegister::Control, *GET_GFX_INFO(VI_STATUS_REG));
		frontend->set_vi_register(RDP::VIRegister::Origin, *GET_GFX_INFO(VI_ORIGIN_REG));
		frontend->set_vi_register(RDP::VIRegister::Width, *GET_GFX_INFO(VI_WIDTH_REG));
		frontend->set_vi_register(RDP::VIRegister::Intr, *GET_GFX_INFO(VI_INTR_REG));
		frontend->set_vi_register(RDP::VIRegister::VCurrentLine, *GET_GFX_INFO(VI_V_CURRENT_LINE_REG));
		frontend->set_vi_register(RDP::VIRegister::Timing, *GET_GFX_INFO(VI_V_BURST_REG));
		frontend->set_vi_register(RDP::VIRegister::VSync, *GET_GFX_INFO(VI_V_SYNC_REG));
		frontend->set_vi_register(RDP::VIRegister::HSync, *GET_GFX_INFO(VI_H_SYNC_REG));
		frontend->set_vi_register(RDP::VIRegister::Leap, *GET_GFX_INFO(VI_LEAP_REG));
		frontend->set_vi_register(RDP::VIRegister::HStart, *GET_GFX_INFO(VI_H_START_REG));
		frontend->set_vi_register(RDP::VIRegister::VStart, *GET_GFX_INFO(VI_V_START_REG));
		frontend->set_vi_register(RDP::VIRegister::VBurst, *GET_GFX_INFO(VI_V_BURST_REG));
		frontend->set_vi_register(RDP::VIRegister::XScale, *GET_GFX_INFO(VI_X_SCALE_REG));
		frontend->set_vi_register(RDP::VIRegister::YScale, *GET_GFX_INFO(VI_Y_SCALE_REG));

		RDP::Quirks quirks;
		quirks.set_native_texture_lod(true);
		quirks.set_native_resolution_tex_rect(true);
		frontend->set_quirks(quirks);
		frontend->begin_frame_context();
		unsigned width = 0;
		unsigned height = 0;
		vk_blit(width, height);
		if (width == 0 || height == 0)
			screen_swap(true);
		else
			screen_swap(false);
	}
}

void vk_process_commands()
{
	if (running)
	{

		const uint32_t DP_CURRENT = *GET_GFX_INFO(DPC_CURRENT_REG) & 0x00FFFFF8;
		const uint32_t DP_END = *GET_GFX_INFO(DPC_END_REG) & 0x00FFFFF8;

		int length = DP_END - DP_CURRENT;
		if (length <= 0)
			return;

		length = unsigned(length) >> 3;
		if ((cmd_ptr + length) & ~(0x0003FFFF >> 3))
			return;

		uint32_t offset = DP_CURRENT;
		if (*GET_GFX_INFO(DPC_STATUS_REG) & DP_STATUS_XBUS_DMA)
		{
			do
			{
				offset &= 0xFF8;
				cmd_data[2 * cmd_ptr + 0] = *reinterpret_cast<const uint32_t *>(SP_DMEM + offset);
				cmd_data[2 * cmd_ptr + 1] = *reinterpret_cast<const uint32_t *>(SP_DMEM + offset + 4);
				offset += sizeof(uint64_t);
				cmd_ptr++;
			} while (--length > 0);
		}
		else
		{
			if (DP_END > 0x7ffffff || DP_CURRENT > 0x7ffffff)
			{
				return;
			}
			else
			{
				do
				{
					offset &= 0xFFFFF8;
					cmd_data[2 * cmd_ptr + 0] = *reinterpret_cast<const uint32_t *>(DRAM + offset);
					cmd_data[2 * cmd_ptr + 1] = *reinterpret_cast<const uint32_t *>(DRAM + offset + 4);
					offset += sizeof(uint64_t);
					cmd_ptr++;
				} while (--length > 0);
			}
		}

		while (cmd_cur - cmd_ptr < 0)
		{
			uint32_t w1 = cmd_data[2 * cmd_cur];
			uint32_t command = (w1 >> 24) & 63;
			int cmd_length = cmd_len_lut[command];

			if (cmd_ptr - cmd_cur - cmd_length < 0)
			{
				*GET_GFX_INFO(DPC_START_REG) = *GET_GFX_INFO(DPC_CURRENT_REG) = *GET_GFX_INFO(DPC_END_REG);
				return;
			}

			if (command >= 8 && frontend)
				frontend->enqueue_command(cmd_length * 2, &cmd_data[2 * cmd_cur]);

			if (RDP::Op(command) == RDP::Op::SyncFull)
			{
				// For synchronous RDP:
				*gfx_info.MI_INTR_REG |= DP_INTERRUPT;
				gfx_info.CheckInterrupts();
			}

			cmd_cur += cmd_length;
		}

		cmd_ptr = 0;
		cmd_cur = 0;
		*GET_GFX_INFO(DPC_START_REG) = *GET_GFX_INFO(DPC_CURRENT_REG) = *GET_GFX_INFO(DPC_END_REG);
	}
}

void vk_destroy()
{
	if (!vk_initialized)
		return;
	running = false;
	frontend.reset();
	device.reset();
	context.reset();
	if (program.pid)
		glDeleteProgramPipelines(1, &program.pid);
	glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
	glDeleteTextures(TEX_NUM, &texture[0]);
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &buffer_pbo);
}

bool vk_init()
{
	running = false;
	context.reset(new Context);
	device.reset(new Device);

	if (!::Vulkan::Context::init_loader(nullptr))
		return false;
	if (!context->init_instance_and_device(nullptr, 0, nullptr, 0))
		return false;
	device->set_context(*context);

	frontend.reset(new RDP::CommandProcessor(*device, reinterpret_cast<void *>(gfx_info.RDRAM),
											 0, rdram_size, rdram_size / 2, 0));
	if (!frontend->device_is_supported())
	{
		frontend.reset();
		return false;
	}
	running = true;
	vk_initialized = 1;
	RDP::Quirks quirks;
	quirks.set_native_texture_lod(true);
	quirks.set_native_resolution_tex_rect(true);
	frontend->set_quirks(quirks);
	init_framebuffer(640, 480);
	return true;
}

void screen_swap(bool blank)
{
	libretro_swap_buffer = !blank;
}
