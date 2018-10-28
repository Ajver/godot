/*************************************************************************/
/*  open_simplex_noise.h                                                 */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2018 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2018 Godot Engine contributors (cf. AUTHORS.md)    */
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

#ifndef OPEN_SIMPLEX_NOISE_H
#define OPEN_SIMPLEX_NOISE_H

#include "core/image.h"
#include "core/reference.h"
#include "scene/resources/texture.h"

#include "thirdparty/misc/open-simplex-noise.h"
#include "scene/resources/noise.h"

class OpenSimplexNoise : public Noise {
	GDCLASS(OpenSimplexNoise, Noise)
	OBJ_SAVE_TYPE(OpenSimplexNoise);

	osn_context contexts[6];

	int seed;
	float persistence; // Controls details, value in [0,1]. Higher increases grain, lower increases smoothness.
	int octaves; // Number of noise layers
	int32_t period; // Distance above which we start to see similarities. The higher, the longer "hills" will be on a terrain.
	Vector<int32_t> seamless_period;
	float lacunarity; // Controls period change across octaves. 2 is usually a good value to address all detail levels.

public:
	OpenSimplexNoise();
	~OpenSimplexNoise();

	void _init_seeds();

	void set_seed(int seed);
	int get_seed() const;

	void set_period(int32_t p_period);
	int32_t get_period() const;

	void set_seamless_period(Vector<int32_t> p_seamless_period);
	Vector<int32_t> get_seamless_period() const;

	void set_octaves(int p_octaves);
	int get_octaves() const { return octaves; }

	void set_persistence(float p_persistence);
	float get_persistence() const { return persistence; }

	void set_lacunarity(float p_lacunarity);
	float get_lacunarity() const { return lacunarity; }

	Ref<Image> get_image(int p_width, int p_height);
	Vector<Ref<Image>> get_image_3d(int p_x, int p_y, int p_z);
	Ref<Image> get_seamless_image(int p_size);
	Vector<Ref<Image> > get_seamless_image_3d(int p_size);

	float get_noise_2d(float x, float y);
	float get_noise_3d(float x, float y, float z);
	float get_noise_4d(float x, float y, float z, float w);

	_FORCE_INLINE_ float _get_octave_noise_2d(int octave, float x, float y) { return open_simplex_noise2(&(contexts[octave]), x, y); }
	_FORCE_INLINE_ float _get_octave_noise_3d(int octave, float x, float y, float z) { return open_simplex_noise3_tileable(&(contexts[octave]), x, y, z); }
	_FORCE_INLINE_ float _get_octave_noise_4d(int octave, float x, float y, float z, float w) { return open_simplex_noise4(&(contexts[octave]), x, y, z, w); }

	// Convenience

	_FORCE_INLINE_ float get_noise_2dv(Vector2 v) { return get_noise_2d(v.x, v.y); }
	_FORCE_INLINE_ float get_noise_3dv(Vector3 v) { return get_noise_3d(v.x, v.y, v.z); }

protected:
	static void _bind_methods();
};

#endif // OPEN_SIMPLEX_NOISE_H
