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

GLuint tex_id = 0;

struct shader_id
{
	int fsid;
	int vsid;
	unsigned int pid;
};

static GLuint vao = 0;
shader_id program = {0};
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
	if (tex_id)
		glDeleteTextures(1, &tex_id);
	program = initShader(vert_shader, frag_shader);
	glBindProgramPipeline(program.pid);
	glGenVertexArrays(1, &vao);

	glCreateTextures(GL_TEXTURE_2D, 1, &tex_id);
	glTextureParameteri(tex_id, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(tex_id, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
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

void vk_blit(unsigned &width, unsigned &height)
{
	if (running)
	{

		RDP::ScanoutOptions opts = {};
		opts.persist_frame_on_invalid_input = true;
		opts.crop_rect.top = true;
		opts.crop_rect.bottom = true;
		opts.crop_rect.enable = true;

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
		retro_width = width;
		retro_height = height;
		retro_pitch = width * sizeof(uint32_t);

		scanout.fence->wait();
		uint8_t *ptr = (uint8_t *)device->map_host_buffer(*scanout.buffer, Vulkan::MEMORY_ACCESS_READ_BIT);
		glPixelStorei(GL_UNPACK_ALIGNMENT, get_alignment(retro_pitch));
		glPixelStorei(GL_UNPACK_ROW_LENGTH, retro_pitch / sizeof(uint32_t));
		glTextureStorage2D(tex_id, 1, GL_RGBA8, retro_width, retro_height);
		glTextureSubImage2D(tex_id, 0, 0, 0, retro_width, retro_height, GL_RGBA, GL_UNSIGNED_BYTE, ptr);
		device->unmap_host_buffer(*scanout.buffer, Vulkan::MEMORY_ACCESS_READ_BIT);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, hw_render.get_current_framebuffer());
		glBindProgramPipeline(program.pid);
		glBindTextureUnit(0, tex_id);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindVertexArray(vao);
		glDrawArrays(GL_TRIANGLES, 0, 3);
		glBindVertexArray(0);
		glBindProgramPipeline(0);
		glBindFramebuffer(GL_FRAMEBUFFER, hw_render.get_current_framebuffer());
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
		frontend->set_quirks(quirks);
		unsigned width = 0;
		unsigned height = 0;
		vk_blit(width, height);
		if (width == 0 || height == 0)
			screen_swap(true);
		else
			screen_swap(false);
		frontend->begin_frame_context();
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
	if (tex_id)
		glDeleteTextures(1, &tex_id);
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
	RDP::Quirks quirks;
	frontend->set_quirks(quirks);
	running = true;
	vk_initialized = 1;
	init_framebuffer(640, 480);
	return true;
}

void screen_swap(bool blank)
{
	libretro_swap_buffer = !blank;
}
