/*************************************************************************/
/*  rasterizer_vulkan.h                                                  */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                    http://www.godotengine.org                         */
/*************************************************************************/
/* Copyright (c) 2007-2018 Cengiz Terzibas (Yaakuro)                     */
/* Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef RASTERIZER_VULKAN_H
#define RASTERIZER_VULKAN_H

#include "rasterizer_canvas_vulkan.h"
#include "rasterizer_scene_vulkan.h"
#include "rasterizer_storage_vulkan.h"
#include "servers/visual/rasterizer.h"

#include <vulkan/vulkan.h>

class RasterizerVulkan : public Rasterizer {

		static Rasterizer *_create_current();

		RasterizerStorageVulkan *storage;
		RasterizerCanvasVulkan *canvas;
		RasterizerSceneVulkan *scene;

	public:

		RasterizerVulkan();
		virtual ~RasterizerVulkan();

		virtual RasterizerStorage *get_storage();
		virtual RasterizerCanvas *get_canvas();
		virtual RasterizerScene *get_scene();

		virtual void set_boot_image(const Ref<Image> &p_image, const Color &p_color, bool p_scale);

		virtual void initialize();
		virtual void begin_frame(double frame_step);
		virtual void set_current_render_target(RID p_render_target);
		virtual void restore_render_target();
		virtual void clear_render_target(const Color &p_color);
		virtual void blit_render_target_to_screen(RID p_render_target, const Rect2 &p_screen_rect, int p_screen = 0);
		virtual void end_frame(bool p_swap_buffers);
		virtual void finalize();

		static void make_current();
		static void register_config();
		
		static void set_vulkan_instance(vkf::Instance* instance);
		static void set_vulkan_device(vkf::Device* device);
		static void set_vulkan_swapchain(vkf::SwapChain* swapchain);
		static void set_vulkan_queue(vkf::Queue* queue);
		static void set_vulkan_command_pool(vkf::CommandPool* command_pool);
		static void set_vulkan_command_buffers(vkf::CommandBuffers* command_buffers);

		static vkf::Instance* instance;
		static vkf::Device* device;
		static vkf::SwapChain* swapchain;
		static vkf::Queue* queue;
		static vkf::CommandPool* command_pool;
		static vkf::CommandBuffers* command_buffers;

		static vkf::Instance* get_instance();
		static vkf::Device* get_device();
		static vkf::SwapChain* get_swapchain();
		static vkf::Queue* get_queue();
		static vkf::CommandPool* get_command_pool();
		static vkf::CommandBuffers* get_command_buffers();
};

#endif
