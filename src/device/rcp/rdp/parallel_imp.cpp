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

#include <libretro.h>
extern struct retro_hw_render_callback hw_render;

static void import_semaphore(GLuint &glsem, const Vulkan::ExternalHandle &handle)
{
	glGenSemaphoresEXT(1, &glsem);
#ifdef _WIN32
	glImportSemaphoreWin32HandleEXT(glsem, GL_HANDLE_TYPE_OPAQUE_WIN32_EXT, handle.handle);
	CloseHandle(handle.handle);
#else
	// Importing an FD takes ownership of it.
	glImportSemaphoreFdEXT(glsem, GL_HANDLE_TYPE_OPAQUE_FD_EXT, handle.handle);
#endif
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
		opts.export_scanout = true;
		opts.export_handle_type = Vulkan::ExternalHandle::get_opaque_memory_handle_type();
		Vulkan::ImageHandle image = frontend->scanout(opts);
		if (!image)
		{
			width = 0;
			height = 0;
			return;
		}

		if (!image->get_width() || !image->get_height())
		{
			width = 0;
			height = 0;
			return;
		}
		auto exported_image = image->export_handle();
		width = 640;
		height = 480;

		GLuint gltex;
		GLuint glmem;
		GLuint glfbo;

		glCreateTextures(GL_TEXTURE_2D, 1, &gltex);
		glCreateMemoryObjectsEXT(1, &glmem);
		glCreateFramebuffers(1, &glfbo);
		GLint gltrue = GL_TRUE;
		glMemoryObjectParameterivEXT(glmem, GL_DEDICATED_MEMORY_OBJECT_EXT, &gltrue);
		glImportMemoryWin32HandleEXT(glmem, image->get_allocation().get_size(),
									 GL_HANDLE_TYPE_OPAQUE_WIN32_EXT, exported_image.handle);
		glTextureStorageMem2DEXT(gltex, 1, GL_RGBA8,
								 GLsizei(image->get_width()),
								 GLsizei(image->get_height()),
								 glmem, 0);
		glNamedFramebufferTexture(glfbo, GL_COLOR_ATTACHMENT0, gltex, 0);
		{
			// Exportable timeline is not widely supported sadly.
			auto signal_semaphore = device->request_semaphore_external(
				VK_SEMAPHORE_TYPE_BINARY_KHR,
				Vulkan::ExternalHandle::get_opaque_semaphore_handle_type());

			// scanout() already performed the barrier to EXTERNAL queue,
			// so just submit a signal to the external semaphore.
			device->submit_empty(Vulkan::CommandBuffer::Type::Generic, nullptr, signal_semaphore.get());
			auto exported_signal = signal_semaphore->export_to_handle();

			GLuint glsem;
			import_semaphore(glsem, exported_signal);

			// Wait. The layout matches whatever we used when releasing the image.
			GLenum gllayout = GL_LAYOUT_SHADER_READ_ONLY_EXT;
			glWaitSemaphoreEXT(glsem, 0, nullptr, 1, &gltex, &gllayout);
			glDeleteSemaphoresEXT(1, &glsem);
			glBlitNamedFramebuffer(glfbo, hw_render.get_current_framebuffer(),
								   0, GLint(image->get_height()), GLint(image->get_width()), 0,
								   0, 0, 640, 480, GL_COLOR_BUFFER_BIT, GL_LINEAR);

			{
				auto wait_semaphore = device->request_semaphore_external(
					VK_SEMAPHORE_TYPE_BINARY_KHR, Vulkan::ExternalHandle::get_opaque_semaphore_handle_type());
				// Have to mark the semaphore is signalled since we assert on that being the case when exporting a semaphore.
				wait_semaphore->signal_external();
				auto exported_semaphore = wait_semaphore->export_to_handle();

				GLuint glsem;
				GLenum gllayout = GL_LAYOUT_SHADER_READ_ONLY_EXT;
				import_semaphore(glsem, exported_semaphore);
				glSignalSemaphoreEXT(glsem, 0, nullptr, 1, &gltex, &gllayout);

				// Add the write-after-read barrier.
				device->add_wait_semaphore(Vulkan::CommandBuffer::Type::Generic, std::move(wait_semaphore),
										   VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, true);

				glDeleteSemaphoresEXT(1, &glsem);
			}
			glDeleteFramebuffers(1, &glfbo);
			glDeleteTextures(1, &gltex);
			glDeleteMemoryObjectsEXT(1, &glmem);
			frontend->begin_frame_context();
		}
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
}

bool vk_init()
{
	running = false;
	Vulkan::Context::SystemHandles handles = {};
	context.reset(new Context);
	device.reset(new Device);

	if (!::Vulkan::Context::init_loader(nullptr))
		return false;
	context->set_system_handles(handles);
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
	return true;
}

void screen_swap(bool blank)
{
	libretro_swap_buffer = !blank;
}
