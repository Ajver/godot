// Copyright (c) 2007-2017 Juan Linietsky, Ariel Manzur.
// Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// -- Godot Engine <https://godotengine.org>

#include "thirdparty/assimp/include/assimp/DefaultLogger.hpp"
#include "thirdparty/assimp/include/assimp/Importer.hpp"
#include "thirdparty/assimp/include/assimp/LogStream.hpp"
#include "thirdparty/assimp/include/assimp/Logger.hpp"
#include "thirdparty/assimp/include/assimp/SceneCombiner.h"
#include "thirdparty/assimp/include/assimp/cexport.h"
#include "thirdparty/assimp/include/assimp/cimport.h"
#include "thirdparty/assimp/include/assimp/matrix4x4.h"
#include "thirdparty/assimp/include/assimp/pbrmaterial.h"
#include "thirdparty/assimp/include/assimp/postprocess.h"
#include "thirdparty/assimp/include/assimp/scene.h"

#include "core/bind/core_bind.h"
#include "core/io/image_loader.h"
#include "editor/editor_file_system.h"
#include "editor/import/resource_importer_scene.h"
#include "editor_scene_importer_asset_import.h"
#include "editor_settings.h"
#include "scene/3d/camera.h"
#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/animation/animation_player.h"
#include "scene/main/node.h"
#include "scene/resources/material.h"
#include "scene/resources/surface_tool.h"
#include "zutil.h"
#include <string>

void EditorSceneImporterAssetImport::get_extensions(List<String> *r_extensions) const {

	const String import_setting_string = "filesystem/import/open_asset_import/";

	Map<String, ImportFormat> import_format;
	{
		Vector<String> exts;
		exts.push_back("gltf");
		exts.push_back("glb");
		ImportFormat import = { exts, false };
		import_format.insert("gltf2", import);
	}
	{
		Vector<String> exts;
		exts.push_back("dae");
		ImportFormat import = { exts, false };
		import_format.insert("dae", import);
	}
	{
		Vector<String> exts;
		exts.push_back("fbx");
		ImportFormat import = { exts, true };
		import_format.insert("fbx", import);
	}
	{
		Vector<String> exts;
		exts.push_back("abc");
		ImportFormat import = { exts, false };
		import_format.insert("abc", import);
	}
	{
		Vector<String> exts;
		exts.push_back("blend");
		ImportFormat import = { exts, false };
		import_format.insert("blend", import);
	}
	{
		Vector<String> exts;
		exts.push_back("q3d");
		ImportFormat import = { exts, false };
		import_format.insert("q3d", import);
	}
	{
		Vector<String> exts;
		exts.push_back("sib");
		ImportFormat import = { exts, false };
		import_format.insert("sib", import);
	}
	{
		Vector<String> exts;
		exts.push_back("smd");
		ImportFormat import = { exts, false };
		import_format.insert("smd", import);
	}
	{
		Vector<String> exts;
		exts.push_back("stl");
		ImportFormat import = { exts, false };
		import_format.insert("stl", import);
	}
	{
		Vector<String> exts;
		exts.push_back("smd");
		ImportFormat import = { exts, false };
		import_format.insert("smd", import);
	}
	{
		Vector<String> exts;
		exts.push_back("stl");
		ImportFormat import = { exts, false };
		import_format.insert("stl", import);
	}
	{
		Vector<String> exts;
		exts.push_back("terragen");
		ImportFormat import = { exts, false };
		import_format.insert("terragen", import);
	}
	{
		Vector<String> exts;
		exts.push_back("3d");
		ImportFormat import = { exts, false };
		import_format.insert("3d", import);
	}
	{
		Vector<String> exts;
		exts.push_back("x");
		ImportFormat import = { exts, false };
		import_format.insert("x", import);
	}
	{
		Vector<String> exts;
		exts.push_back("x3d");
		ImportFormat import = { exts, false };
		import_format.insert("x3d", import);
	}
	{
		Vector<String> exts;
		exts.push_back("3mf");
		ImportFormat import = { exts, false };
		import_format.insert("3mf", import);
	}
	{
		Vector<String> exts;
		exts.push_back("pmx");
		ImportFormat import = { exts, false };
		import_format.insert("mmd", import);
	}
	{
		Vector<String> exts;
		exts.push_back("amf");
		ImportFormat import = { exts, false };
		import_format.insert("amf", import);
	}
	{
		Vector<String> exts;
		exts.push_back("3ds");
		ImportFormat import = { exts, false };
		import_format.insert("3ds", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ac");
		ImportFormat import = { exts, false };
		import_format.insert("ac", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ase");
		ImportFormat import = { exts, false };
		import_format.insert("ase", import);
	}
	{
		Vector<String> exts;
		exts.push_back("assbin");
		exts.push_back("assxml");
		ImportFormat import = { exts, false };
		import_format.insert("ass", import);
	}
	{
		Vector<String> exts;
		exts.push_back("b3d");
		ImportFormat import = { exts, false };
		import_format.insert("b3d", import);
	}
	{
		Vector<String> exts;
		exts.push_back("bvh");
		ImportFormat import = { exts, false };
		import_format.insert("bvh", import);
	}
	{
		Vector<String> exts;
		exts.push_back("dxf");
		ImportFormat import = { exts, false };
		import_format.insert("dxf", import);
	}
	{
		Vector<String> exts;
		exts.push_back("csm");
		ImportFormat import = { exts, false };
		import_format.insert("csm", import);
	}
	{
		Vector<String> exts;
		exts.push_back("hmp");
		ImportFormat import = { exts, false };
		import_format.insert("hmp", import);
	}
	{
		Vector<String> exts;
		exts.push_back("lwo");
		ImportFormat import = { exts, false };
		import_format.insert("lwo", import);
	}
	{
		Vector<String> exts;
		exts.push_back("lws");
		ImportFormat import = { exts, false };
		import_format.insert("lws", import);
	}
	{
		Vector<String> exts;
		exts.push_back("md2");
		ImportFormat import = { exts, false };
		import_format.insert("md2", import);
	}
	{
		Vector<String> exts;
		exts.push_back("md3");
		ImportFormat import = { exts, false };
		import_format.insert("md3", import);
	}
	{
		Vector<String> exts;
		exts.push_back("md5mesh");
		ImportFormat import = { exts, false };
		import_format.insert("md5", import);
	}
	{
		Vector<String> exts;
		exts.push_back("mdc");
		ImportFormat import = { exts, false };
		import_format.insert("mdc", import);
	}
	{
		Vector<String> exts;
		exts.push_back("mdl");
		ImportFormat import = { exts, false };
		import_format.insert("mdl", import);
	}
	{
		Vector<String> exts;
		exts.push_back("nff");
		ImportFormat import = { exts, false };
		import_format.insert("nff", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ndo");
		ImportFormat import = { exts, false };
		import_format.insert("ndo", import);
	}
	{
		Vector<String> exts;
		exts.push_back("off");
		ImportFormat import = { exts, false };
		import_format.insert("off", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ogex");
		ImportFormat import = { exts, false };
		import_format.insert("ogex", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ply");
		ImportFormat import = { exts, false };
		import_format.insert("ply", import);
	}
	{
		Vector<String> exts;
		exts.push_back("ms3d");
		ImportFormat import = { exts, false };
		import_format.insert("ms3d", import);
	}
	{
		Vector<String> exts;
		exts.push_back("cob");
		ImportFormat import = { exts, false };
		import_format.insert("cob", import);
	}
	{
		Vector<String> exts;
		exts.push_back("xgl");
		ImportFormat import = { exts, false };
		import_format.insert("xgl", import);
	}
	for (Map<String, ImportFormat>::Element *E = import_format.front(); E; E = E->next()) {
		_register_project_setting_import(E->key(), import_setting_string, E->get().extensions, r_extensions, E->get().is_default);
	}
}

void EditorSceneImporterAssetImport::_register_project_setting_import(const String generic, const String import_setting_string, const Vector<String> &exts, List<String> *r_extensions, const bool p_enabled) const {
	const String use_generic = "use_" + generic;
	_GLOBAL_DEF(import_setting_string + use_generic, p_enabled, true);
	if (ProjectSettings::get_singleton()->get(import_setting_string + use_generic)) {
		for (size_t i = 0; i < exts.size(); i++) {
			r_extensions->push_back(exts[i]);
		}
	}
}

uint32_t EditorSceneImporterAssetImport::get_import_flags() const {
	return IMPORT_SCENE;
}

AssimpStream::AssimpStream() {
	// empty
}

AssimpStream::~AssimpStream() {
	// empty
}

void AssimpStream::write(const char *message) {
	print_verbose(String("Open Asset Importer: ") + String(message).strip_edges());
}

void EditorSceneImporterAssetImport::_bind_methods() {
}

Node *EditorSceneImporterAssetImport::import_scene(const String &p_path, uint32_t p_flags, int p_bake_fps, List<String> *r_missing_deps, Error *r_err) {
	Assimp::Importer importer;
	std::wstring w_path = ProjectSettings::get_singleton()->globalize_path(p_path).c_str();
	std::string s_path(w_path.begin(), w_path.end());
	//importer.SetPropertyBool(AI_CONFIG_PP_FD_REMOVE, true);
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	//importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
	//importer.SetPropertyFloat(AI_CONFIG_PP_DB_THRESHOLD, 1.0f);
	int32_t post_process_Steps = aiProcess_CalcTangentSpace |
								 //aiProcess_FlipUVs |
								 //aiProcess_FlipWindingOrder |
								 //aiProcess_DropNormals |
								 aiProcess_GenSmoothNormals |
								 aiProcess_JoinIdenticalVertices |
								 aiProcess_ImproveCacheLocality |
								 aiProcess_LimitBoneWeights |
								 //aiProcess_RemoveRedundantMaterials |
								 aiProcess_SplitLargeMeshes |
								 aiProcess_Triangulate |
								 aiProcess_GenUVCoords |
								 //aiProcess_FindDegenerates |
								 //aiProcess_SortByPType |
								 //aiProcess_FindInvalidData |
								 aiProcess_TransformUVCoords |
								 aiProcess_FindInstances |
								 //aiProcess_FixInfacingNormals |
								 //aiProcess_ValidateDataStructure |
								 aiProcess_OptimizeMeshes |
								 //aiProcess_OptimizeGraph |
								 //aiProcess_Debone |
								 aiProcess_EmbedTextures |
								 aiProcess_SplitByBoneCount |
								 0;
	const aiScene *scene = importer.ReadFile(s_path.c_str(),
			post_process_Steps);
	ERR_EXPLAIN(String("Open Asset Importer failed to open: ") + String(importer.GetErrorString()));
	ERR_FAIL_COND_V(scene == NULL, NULL);
	return _generate_scene(p_path, scene, p_flags, p_bake_fps);
}

template <class T>
struct EditorSceneImporterAssetImportInterpolate {

	T lerp(const T &a, const T &b, float c) const {

		return a + (b - a) * c;
	}

	T catmull_rom(const T &p0, const T &p1, const T &p2, const T &p3, float t) {

		float t2 = t * t;
		float t3 = t2 * t;

		return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4 * p2 - p3) * t2 + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
	}

	T bezier(T start, T control_1, T control_2, T end, float t) {
		/* Formula from Wikipedia article on Bezier curves. */
		real_t omt = (1.0 - t);
		real_t omt2 = omt * omt;
		real_t omt3 = omt2 * omt;
		real_t t2 = t * t;
		real_t t3 = t2 * t;

		return start * omt3 + control_1 * omt2 * t * 3.0 + control_2 * omt * t2 * 3.0 + end * t3;
	}
};

//thank you for existing, partial specialization
template <>
struct EditorSceneImporterAssetImportInterpolate<Quat> {

	Quat lerp(const Quat &a, const Quat &b, float c) const {
		ERR_FAIL_COND_V(!a.is_normalized(), Quat());
		ERR_FAIL_COND_V(!b.is_normalized(), Quat());

		return a.slerp(b, c).normalized();
	}

	Quat catmull_rom(const Quat &p0, const Quat &p1, const Quat &p2, const Quat &p3, float c) {
		ERR_FAIL_COND_V(!p1.is_normalized(), Quat());
		ERR_FAIL_COND_V(!p2.is_normalized(), Quat());

		return p1.slerp(p2, c).normalized();
	}

	Quat bezier(Quat start, Quat control_1, Quat control_2, Quat end, float t) {
		ERR_FAIL_COND_V(!start.is_normalized(), Quat());
		ERR_FAIL_COND_V(!end.is_normalized(), Quat());

		return start.slerp(end, t).normalized();
	}
};

template <class T>
T EditorSceneImporterAssetImport::_interpolate_track(const Vector<float> &p_times, const Vector<T> &p_values, float p_time, AssetImportAnimation::Interpolation p_interp) {
	//could use binary search, worth it?
	int idx = -1;
	for (int i = 0; i < p_times.size(); i++) {
		if (p_times[i] > p_time)
			break;
		idx++;
	}

	EditorSceneImporterAssetImportInterpolate<T> interp;

	switch (p_interp) {
		case AssetImportAnimation::INTERP_LINEAR: {

			if (idx == -1) {
				return p_values[0];
			} else if (idx >= p_times.size() - 1) {
				return p_values[p_times.size() - 1];
			}

			float c = (p_time - p_times[idx]) / (p_times[idx + 1] - p_times[idx]);

			return interp.lerp(p_values[idx], p_values[idx + 1], c);

		} break;
		case AssetImportAnimation::INTERP_STEP: {

			if (idx == -1) {
				return p_values[0];
			} else if (idx >= p_times.size() - 1) {
				return p_values[p_times.size() - 1];
			}

			return p_values[idx];

		} break;
		case AssetImportAnimation::INTERP_CATMULLROMSPLINE: {

			if (idx == -1) {
				return p_values[1];
			} else if (idx >= p_times.size() - 1) {
				return p_values[1 + p_times.size() - 1];
			}

			float c = (p_time - p_times[idx]) / (p_times[idx + 1] - p_times[idx]);

			return interp.catmull_rom(p_values[idx - 1], p_values[idx], p_values[idx + 1], p_values[idx + 3], c);

		} break;
		case AssetImportAnimation::INTERP_CUBIC_SPLINE: {

			if (idx == -1) {
				return p_values[1];
			} else if (idx >= p_times.size() - 1) {
				return p_values[(p_times.size() - 1) * 3 + 1];
			}

			float c = (p_time - p_times[idx]) / (p_times[idx + 1] - p_times[idx]);

			T from = p_values[idx * 3 + 1];
			T c1 = from + p_values[idx * 3 + 2];
			T to = p_values[idx * 3 + 4];
			T c2 = to + p_values[idx * 3 + 3];

			return interp.bezier(from, c1, c2, to, c);

		} break;
	}

	ERR_FAIL_V(p_values[0]);
}

Spatial *EditorSceneImporterAssetImport::_generate_scene(const String &p_path, const aiScene *scene, const uint32_t p_flags, int p_bake_fps) {
	ERR_FAIL_COND_V(scene == NULL, NULL);
	Spatial *root = memnew(Spatial);
	AnimationPlayer *ap = NULL;
	if (p_flags & IMPORT_ANIMATION) {
		ap = memnew(AnimationPlayer);
		root->add_child(ap);
		ap->set_owner(root);
		ap->set_name(TTR("AnimationPlayer"));
	}
	Set<String> bone_names;
	Set<String> light_names;
	Set<String> camera_names;
	for (size_t l = 0; l < scene->mNumLights; l++) {
		Light *light = NULL;
		aiLight *ai_light = scene->mLights[l];
		ERR_CONTINUE(ai_light == NULL);
		if (ai_light->mType == aiLightSource_DIRECTIONAL) {
			light = memnew(DirectionalLight);
			Vector3 dir = Vector3(ai_light->mDirection.x, ai_light->mDirection.y, ai_light->mDirection.z);
			dir.normalize();
			Transform xform;
			Quat quat;
			quat.set_euler(dir);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			xform.origin = pos;
			light->set_transform(xform);
		} else if (ai_light->mType == aiLightSource_POINT) {
			light = memnew(OmniLight);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			Transform xform;
			xform.origin = pos;
			light->set_transform(xform);
			// No idea for energy
			light->set_param(Light::PARAM_ATTENUATION, 0.0f);
		} else if (ai_light->mType == aiLightSource_SPOT) {
			light = memnew(SpotLight);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			Transform xform;
			xform.origin = pos;
			Vector3 dir = Vector3(ai_light->mDirection.x, ai_light->mDirection.y, ai_light->mDirection.z);
			dir.normalize();
			Quat quat;
			quat.set_euler(dir);
			xform.basis = quat;
			light->set_transform(xform);
			// No idea for energy
			light->set_param(Light::PARAM_ATTENUATION, 0.0f);
		}
		ERR_CONTINUE(light == NULL);
		light->set_color(Color(ai_light->mColorDiffuse.r, ai_light->mColorDiffuse.g, ai_light->mColorDiffuse.b));
		root->add_child(light);
		light->set_name(_ai_string_to_string(ai_light->mName));
		light->set_owner(root);
		light_names.insert(_ai_string_to_string(scene->mLights[l]->mName));
	}
	for (size_t c = 0; c < scene->mNumCameras; c++) {
		aiCamera *ai_camera = scene->mCameras[c];
		Camera *camera = memnew(Camera);
		float near = ai_camera->mClipPlaneNear;
		if (Math::is_equal_approx(near, 0.0f)) {
			near = 0.1f;
		}
		camera->set_perspective(Math::rad2deg(ai_camera->mHorizontalFOV) * 2.0f, near, ai_camera->mClipPlaneFar);
		Vector3 pos = Vector3(ai_camera->mPosition.x, ai_camera->mPosition.y, ai_camera->mPosition.z);

		Vector3 look_at = Vector3(ai_camera->mLookAt.x, ai_camera->mLookAt.y, ai_camera->mLookAt.z).normalized();
		Quat quat;
		quat.set_euler(look_at);
		Transform xform;
		xform.basis = quat;
		xform.set_origin(pos);
		root->add_child(camera);
		camera->set_transform(xform);
		camera->set_name(_ai_string_to_string(ai_camera->mName));
		camera->set_owner(root);
		camera_names.insert(_ai_string_to_string(scene->mCameras[c]->mName));
	}
	Map<Skeleton *, MeshInstance *> skeletons;
	Map<String, Transform> bone_rests;
	Map<MeshInstance *, String> meshes;
	Map<MeshInstance *, String> orig_meshes;
	int32_t mesh_count = 0;
	_generate_node(p_path, scene, scene->mRootNode, root, root, bone_names, light_names, camera_names, skeletons, bone_rests, meshes, orig_meshes, mesh_count);
	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		_set_bone_parent(E->key(), root);
	}
	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		E->key()->localize_rests();
	}
	Set<Node *> keep_nodes;
	_keep_node(p_path, root, root, keep_nodes);
	_fill_kept_node(keep_nodes);
	Set<String> removed_nodes;
	_filter_node(p_path, root, root, keep_nodes, removed_nodes);

	bool has_pivot_inverse = false;
	for (Set<Node *>::Element *E = keep_nodes.front(); E; E = E->next()) {
		String name = E->get()->get_name();
		if (name.ends_with("_$AssimpFbx$_RotationPivotInverse") ||
				name.ends_with("_$AssimpFbx$_ScalingPivotInverse")) {
			has_pivot_inverse = has_pivot_inverse || true;
			break;
		}
	}
	if (p_flags & IMPORT_ANIMATION) {
		//String skeleton_bone_name = _find_skeleton_bone_root(skeletons, meshes, root);
		//String orig_skeleton_bone_name = _find_skeleton_bone_root(skeletons, orig_meshes, root);
		for (int i = 0; i < scene->mNumAnimations; i++) {
			_import_animation(p_path, meshes, orig_meshes, scene, ap, i, p_bake_fps, skeletons, removed_nodes, has_pivot_inverse);
		}
		List<StringName> animation_names;
		ap->get_animation_list(&animation_names);
		if (animation_names.empty()) {
			root->remove_child(ap);
			memdelete(ap);
		}
	}
	return root;
}

void EditorSceneImporterAssetImport::_import_animation_task(const aiScene *scene, const String &p_path, AnimationPlayer *ap, int p_bake_fps, Map<Skeleton *, MeshInstance *> skeletons, const String skeleton_root_name, const String orig_skeleton_bone_name, Spatial *root, const Set<String> p_removed_nodes, bool p_has_pivot_inverse) {
}

void EditorSceneImporterAssetImport::_fill_kept_node(Set<Node *> &keep_nodes) {
	for (Set<Node *>::Element *E = keep_nodes.front(); E; E = E->next()) {
		Node *node = E->get();
		while (node != NULL) {
			if (keep_nodes.has(node) == false) {
				keep_nodes.insert(node);
			}
			node = node->get_parent();
		}
	}
}

String EditorSceneImporterAssetImport::_find_skeleton_bone_root(Map<Skeleton *, MeshInstance *> &skeletons, Map<MeshInstance *, String> &meshes, Spatial *root) {
	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		if (meshes.has(E->get())) {
			String name = meshes[E->get()];
			if (name != "") {
				return name;
			}
		}
	}
	return "";
}

void EditorSceneImporterAssetImport::_set_bone_parent(Skeleton *s, Node *p_owner) {
	for (size_t j = 0; j < s->get_bone_count(); j++) {
		String bone_name = s->get_bone_name(j);
		int32_t node_parent_index = -1;
		const Node *bone_node = p_owner->find_node(bone_name);
		if (bone_node == NULL) {
			continue;
		}
		while (bone_node != NULL && bone_node->get_parent() != NULL) {
			String parent_bone_name = bone_node->get_parent()->get_name();
			node_parent_index = s->find_bone(parent_bone_name);
			if (node_parent_index != -1) {
				break;
			}
			bone_node = bone_node->get_parent();
		}
		//ERR_EXPLAIN(String("Can't find parent bone for ") + bone_node->get_name())
		//ERR_CONTINUE(node_parent_index == -1);
		s->set_bone_parent(j, node_parent_index);
	}
}

void EditorSceneImporterAssetImport::_insert_animation_track(const aiScene *p_scene, const String p_path, int p_bake_fps, Ref<Animation> animation, float ticks_per_second, float length, const Skeleton *sk, size_t i, const aiNodeAnim *track, String node_name, String p_skeleton_root, const String p_orig_skeleton_root, NodePath node_path, bool p_has_pivot_inverse) {

	if (track->mNumRotationKeys || track->mNumPositionKeys || track->mNumScalingKeys) {
		//make transform track
		int track_idx = animation->get_track_count();
		animation->add_track(Animation::TYPE_TRANSFORM);
		animation->track_set_path(track_idx, node_path);
		//first determine animation length

		for (int i = 0; i < track->mNumRotationKeys; i++) {
			length = MAX(length, track->mRotationKeys[i].mTime / ticks_per_second);
		}
		for (int i = 0; i < track->mNumPositionKeys; i++) {
			length = MAX(length, track->mPositionKeys[i].mTime / ticks_per_second);
		}
		for (int i = 0; i < track->mNumScalingKeys; i++) {
			length = MAX(length, track->mScalingKeys[i].mTime / ticks_per_second);
		}

		float increment = 1.0 / float(p_bake_fps);
		float time = 0.0;

		Vector3 base_pos;
		Quat base_rot;
		Vector3 base_scale = Vector3(1, 1, 1);

		if (track->mNumRotationKeys != 0) {
			aiQuatKey key = track->mRotationKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			real_t w = key.mValue.w;
			Quat q(x, y, z, w);
			q = q.normalized();
			base_rot = q;
		}

		if (track->mNumPositionKeys != 0) {
			aiVectorKey key = track->mPositionKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_pos = Vector3(x, y, z);
		}

		if (track->mNumScalingKeys != 0) {
			aiVectorKey key = track->mScalingKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_scale = Vector3(x, y, z);
		}

		bool last = false;

		Vector<Vector3> pos_values;
		Vector<float> pos_times;
		Vector<Vector3> scale_values;
		Vector<float> scale_times;
		Vector<Quat> rot_values;
		Vector<float> rot_times;

		for (size_t p = 0; p < track->mNumPositionKeys; p++) {
			aiVector3D pos = track->mPositionKeys[p].mValue;
			pos_values.push_back(Vector3(pos.x, pos.y, pos.z));
			pos_times.push_back(track->mPositionKeys[p].mTime / ticks_per_second);
		}

		for (size_t r = 0; r < track->mNumRotationKeys; r++) {
			aiQuaternion quat = track->mRotationKeys[r].mValue;
			rot_values.push_back(Quat(quat.x, quat.y, quat.z, quat.w).normalized());
			rot_times.push_back(track->mRotationKeys[r].mTime / ticks_per_second);
		}

		for (size_t sc = 0; sc < track->mNumScalingKeys; sc++) {
			aiVector3D scale = track->mScalingKeys[sc].mValue;
			scale_values.push_back(Vector3(scale.x, scale.y, scale.z));
			scale_times.push_back(track->mScalingKeys[sc].mTime / ticks_per_second);
		}
		while (true) {
			Vector3 pos = base_pos;
			Quat rot = base_rot;
			Vector3 scale = base_scale;

			if (pos_values.size()) {
				pos = _interpolate_track<Vector3>(pos_times, pos_values, time, AssetImportAnimation::INTERP_LINEAR);
			}

			if (rot_values.size()) {
				rot = _interpolate_track<Quat>(rot_times, rot_values, time, AssetImportAnimation::INTERP_LINEAR).normalized();
			}

			if (scale_values.size()) {
				scale = _interpolate_track<Vector3>(scale_times, scale_values, time, AssetImportAnimation::INTERP_LINEAR);
			}

			if (sk != NULL && sk->find_bone(node_name) != -1) {
				Transform xform;
				//xform.basis = Basis(rot);
				//xform.basis.scale(scale);
				xform.basis.set_quat_scale(rot, scale);
				xform.origin = pos;

				int bone = sk->find_bone(node_name);

				Transform rest_xform = sk->get_bone_rest(bone);
				xform = rest_xform.affine_inverse() * xform;
				rot = xform.basis.get_rotation_quat();
				scale = xform.basis.get_scale();
				pos = xform.origin;
			}
			if (sk && p_skeleton_root == node_name) {
				Transform anim_xform;
				String ext = p_path.get_file().get_extension().to_lower();
				if (ext == "fbx") {
					real_t factor = 1.0f;
					if (p_scene->mMetaData != NULL) {
						p_scene->mMetaData->Get("UnitScaleFactor", factor);
					}
					//factor = factor * 0.01f;
					anim_xform = anim_xform.scaled(Vector3(factor, factor, factor));
				}
				Transform format_xform = _format_rot_xform(p_path, p_scene);
				anim_xform = format_xform * anim_xform;
				Transform xform;
				xform.basis.set_quat_scale(rot, scale);
				xform.origin = pos;

				Transform mesh_xform = _ai_matrix_transform(_ai_find_node(p_scene->mRootNode, sk->get_parent()->get_name())->mTransformation);
				xform = mesh_xform.affine_inverse() * xform;

				xform = anim_xform * xform;

				rot = xform.basis.get_rotation_quat();
				scale = xform.basis.get_scale();
				pos = xform.origin;
			}
			rot.normalize();

			animation->track_set_interpolation_type(track_idx, Animation::INTERPOLATION_LINEAR);
			animation->transform_track_insert_key(track_idx, time, pos, rot, scale);

			if (last) {
				break;
			}
			time += increment;
			if (time >= length) {
				last = true;
				time = length;
			}
		}
	}
}

void EditorSceneImporterAssetImport::_import_animation(const String p_path, const Map<MeshInstance *, String> p_meshes, const Map<MeshInstance *, String> p_orig_meshes, const aiScene *p_scene, AnimationPlayer *ap, int32_t p_index, int p_bake_fps, Map<Skeleton *, MeshInstance *> p_skeletons, const Set<String> p_removed_nodes, bool p_has_pivot_inverse) {
	String name = "Animation";
	aiAnimation const *anim = NULL;
	if (p_index != -1) {
		anim = p_scene->mAnimations[p_index];
		if (anim->mName.length > 0) {
			name = _ai_anim_string_to_string(anim->mName);
		}
	}

	Ref<Animation> animation;
	animation.instance();
	float length = 0.0f;
	animation->set_name(name);
	float ticks_per_second = p_scene->mAnimations[p_index]->mTicksPerSecond;

	if (p_scene->mMetaData != NULL && Math::is_equal_approx(ticks_per_second, 0.0f)) {
		int32_t time_mode = 0;
		p_scene->mMetaData->Get("TimeMode", time_mode);
		ticks_per_second = _get_fbx_fps(time_mode, p_scene);
	}

	if ((p_path.get_file().get_extension().to_lower() == "glb" || p_path.get_file().get_extension().to_lower() == "gltf") && Math::is_equal_approx(ticks_per_second, 0.0f)) {
		ticks_per_second = 1000.0f;
	}

	if (Math::is_equal_approx(ticks_per_second, 0.0f)) {
		ticks_per_second = 25.0f;
	}

	length = anim->mDuration / ticks_per_second;
	if (anim) {

		for (size_t i = 0; i < anim->mNumChannels; i++) {
			const aiNodeAnim *track = anim->mChannels[i];
			String node_name = _ai_string_to_string(track->mNodeName);
			NodePath node_path = node_name;
			if (node_name.split(ASSIMP_FBX_KEY).size() > 1) {
				String p_track_type = node_name.split(ASSIMP_FBX_KEY)[1];
				if (p_track_type == "_Translation" || p_track_type == "_Rotation" || p_track_type == "_Scaling") {
					continue;
				}
			}
			for (Map<Skeleton *, MeshInstance *>::Element *E = p_skeletons.front(); E; E = E->next()) {
				Skeleton *sk = E->key();
				const String path = ap->get_owner()->get_path_to(sk);
				if (path.empty()) {
					continue;
				}
				if (sk->find_bone(node_name) == -1) {
					continue;
				}
				node_path = path + ":" + node_name;
				ERR_CONTINUE(ap->get_owner()->has_node(node_path) == false);
				String p_skeleton_root;
				if (p_meshes.has(E->get())) {
					p_skeleton_root = p_meshes[E->get()];
				}
				String p_orig_skeleton_root;
				if (p_orig_meshes.has(E->get())) {
					p_orig_skeleton_root = p_orig_meshes[E->get()];
				}
				_insert_animation_track(p_scene, p_path, p_bake_fps, animation, ticks_per_second, length, sk, i, track, node_name, p_skeleton_root, p_orig_skeleton_root, node_path, p_has_pivot_inverse);
			}
		}
		Map<String, Vector<const aiNodeAnim *> > node_tracks;
		for (size_t i = 0; i < anim->mNumChannels; i++) {
			const aiNodeAnim *track = anim->mChannels[i];
			String node_name = _ai_string_to_string(track->mNodeName);
			if (p_removed_nodes.has(node_name)) {
				continue;
			}
			if (p_removed_nodes.has(node_name.split(ASSIMP_FBX_KEY)[0])) {
				continue;
			}
			bool is_bone = false;
			Vector<String> split_name = node_name.split(ASSIMP_FBX_KEY);
			String bare_name = split_name[0];
			for (Map<Skeleton *, MeshInstance *>::Element *E = p_skeletons.front(); E; E = E->next()) {
				Skeleton *sk = E->key();
				if (sk->find_bone(bare_name) != -1) {
					is_bone = true;
					break;
				}
			}
			if (is_bone) {
				continue;
			}
			if (ap->get_owner()->find_node(bare_name) != NULL && split_name.size() > 1) {
				Map<String, Vector<const aiNodeAnim *> >::Element *E = node_tracks.find(bare_name);
				Vector<const aiNodeAnim *> ai_tracks;
				if (E) {
					ai_tracks = E->get();
					ai_tracks.push_back(anim->mChannels[i]);
				} else {
					ai_tracks.push_back(anim->mChannels[i]);
				}
				node_tracks.insert(bare_name, ai_tracks);
				continue;
			}

			Node *node = ap->get_owner()->find_node(node_name);
			if (node == NULL) {
				continue;
			}

			if (node != NULL) {
				NodePath node_path;
				const String path = ap->get_owner()->get_path_to(node);
				// Allow duplicate tracks
				if (path.empty()) {
					print_verbose("Can't animate path");
					continue;
				}
				node_path = path;
				if (ap->get_owner()->has_node(node_path) == false) {
					continue;
				}
				String p_skeleton_root;
				String p_orig_skeleton_root;
				MeshInstance *mi = Object::cast_to<MeshInstance>(node);
				if (mi != NULL) {
					if (p_meshes.has(mi)) {
						p_skeleton_root = p_meshes[mi];
					}
					if (p_orig_meshes.has(mi)) {
						p_orig_skeleton_root = p_orig_meshes[mi];
					}
				}
				_insert_animation_track(p_scene, p_path, p_bake_fps, animation, ticks_per_second, length, NULL, i, track, node_name, p_skeleton_root, p_orig_skeleton_root, node_path, p_has_pivot_inverse);
			}
		}
		for (Map<String, Vector<const aiNodeAnim *> >::Element *E = node_tracks.front(); E; E = E->next()) {
			_insert_pivot_anim_track(p_meshes, p_orig_meshes, E->key(), E->get(), ap, NULL, length, ticks_per_second, animation, p_bake_fps, p_path, p_scene);
		}

		for (Map<Skeleton *, MeshInstance *>::Element *E = p_skeletons.front(); E; E = E->next()) {
			Skeleton *sk = E->key();
			Map<String, Vector<const aiNodeAnim *> > anim_tracks;
			for (size_t i = 0; i < sk->get_bone_count(); i++) {
				String _bone_name = sk->get_bone_name(i);
				Vector<const aiNodeAnim *> ai_tracks;
				for (size_t j = 0; j < anim->mNumChannels; j++) {
					if (_ai_string_to_string(anim->mChannels[j]->mNodeName).split(ASSIMP_FBX_KEY).size() == 1) {
						continue;
					}
					String track_name = _ai_string_to_string(anim->mChannels[j]->mNodeName).split(ASSIMP_FBX_KEY)[0];
					if (track_name != _bone_name) {
						continue;
					}
					if (sk->find_bone(_bone_name) == -1) {
						continue;
					}
					ai_tracks.push_back(anim->mChannels[j]);
				}
				if (ai_tracks.size() == 0) {
					continue;
				}
				anim_tracks.insert(_bone_name, ai_tracks);
			}

			for (Map<String, Vector<const aiNodeAnim *> >::Element *F = anim_tracks.front(); F; F = F->next()) {
				_insert_pivot_anim_track(p_meshes, p_orig_meshes, F->key(), F->get(), ap, sk, length, ticks_per_second, animation, p_bake_fps, p_path, p_scene);
			}
		}
		for (Map<Skeleton *, MeshInstance *>::Element *E = p_skeletons.front(); E; E = E->next()) {
			Skeleton *sk = E->key();
			const String path = ap->get_owner()->get_path_to(sk);
			String p_skeleton_root;
			if (p_meshes.has(E->get())) {
				p_skeleton_root = p_meshes[E->get()];
			}
			const NodePath node_path = path + ":" + p_skeleton_root;

			if (animation->find_track(node_path) != -1) {
				continue;
			}
			int track_idx = animation->get_track_count();
			animation->add_track(Animation::TYPE_TRANSFORM);

			animation->track_set_path(track_idx, node_path);
			animation->track_set_interpolation_type(track_idx, Animation::INTERPOLATION_LINEAR);
			Transform mesh_xform = _get_global_ai_node_transform(p_scene, _ai_find_node(p_scene->mRootNode, sk->get_parent()->get_name()));
			mesh_xform = mesh_xform.affine_inverse();
			animation->transform_track_insert_key(track_idx, 0.0f, mesh_xform.origin, mesh_xform.get_basis().get_quat(), mesh_xform.get_basis().get_scale());
			animation->transform_track_insert_key(track_idx, length, mesh_xform.origin, mesh_xform.get_basis().get_quat(), mesh_xform.get_basis().get_scale());
		}
		for (int i = 0; i < anim->mNumMorphMeshChannels; i++) {
			const aiMeshMorphAnim *anim_mesh = anim->mMorphMeshChannels[i];
			const String prop_name = _ai_string_to_string(anim_mesh->mName);
			const String mesh_name = prop_name.split("*")[0];
			if (p_removed_nodes.has(mesh_name)) {
				continue;
			}
			ERR_CONTINUE(prop_name.split("*").size() != 2);
			const MeshInstance *mesh_instance = Object::cast_to<MeshInstance>(ap->get_owner()->find_node(mesh_name));
			ERR_CONTINUE(mesh_instance == NULL);
			if (ap->get_owner()->find_node(mesh_instance->get_name()) == NULL) {
				print_verbose("Can't find mesh in scene: " + mesh_instance->get_name());
				continue;
			}
			const String path = ap->get_owner()->get_path_to(mesh_instance);
			if (path.empty()) {
				print_verbose("Can't find mesh in scene");
				continue;
			}
			Ref<Mesh> mesh = mesh_instance->get_mesh();
			ERR_CONTINUE(mesh.is_null());

			for (size_t k = 0; k < anim_mesh->mNumKeys; k++) {
				for (size_t j = 0; j < anim_mesh->mKeys[k].mNumValuesAndWeights; j++) {
					const String prop = "blend_shapes/" + mesh->get_blend_shape_name(anim_mesh->mKeys[k].mValues[j]);
					const NodePath node_path = String(path) + ":" + prop;
					ERR_CONTINUE(ap->get_owner()->has_node(node_path) == false);
					int32_t track_idx = -1;
					if (animation->find_track(node_path) == -1) {
						track_idx = animation->get_track_count();
						animation->add_track(Animation::TYPE_VALUE);
						animation->track_set_interpolation_type(track_idx, Animation::INTERPOLATION_NEAREST);
						animation->track_set_path(track_idx, node_path);
					} else {
						track_idx = animation->find_track(node_path);
					}
					float t = anim_mesh->mKeys[k].mTime / ticks_per_second;
					float w = anim_mesh->mKeys[k].mWeights[j];
					animation->track_insert_key(track_idx, t, w);
				}
			}
		}
	}
	animation->set_length(length);
	if (animation->get_track_count()) {
		ap->add_animation(name, animation);
	}
}

void EditorSceneImporterAssetImport::_insert_pivot_anim_track(const Map<MeshInstance *, String> p_meshes, const Map<MeshInstance *, String> p_orig_meshes, const String p_node_name, Vector<const aiNodeAnim *> F, AnimationPlayer *ap, Skeleton *sk, float &length, float ticks_per_second, Ref<Animation> animation, int p_bake_fps, const String &p_path, const aiScene *p_scene) {
	NodePath node_path;
	String p_skeleton_root;
	String p_orig_skeleton_root;
	if (sk != NULL) {
		const String path = ap->get_owner()->get_path_to(sk);
		if (path.empty()) {
			return;
		}
		if (sk->find_bone(p_node_name) == -1) {
			return;
		}
		node_path = path + ":" + p_node_name;
		Node *node = sk->get_parent();
		MeshInstance *mi = Object::cast_to<MeshInstance>(node);
		if (mi != NULL) {
			if (p_meshes.has(mi)) {
				p_skeleton_root = p_meshes[mi];
			}
			if (p_orig_meshes.has(mi)) {
				p_orig_skeleton_root = p_orig_meshes[mi];
			}
		}
	} else {
		Node *node = ap->get_owner()->find_node(p_node_name);
		if (node == NULL) {
			return;
		}
		MeshInstance *mi = Object::cast_to<MeshInstance>(node);
		if (mi != NULL) {
			if (p_meshes.has(mi)) {
				p_skeleton_root = p_meshes[mi];
			}
		}
		if (p_skeleton_root == p_node_name) {
			return;
		}
		const String path = ap->get_owner()->get_path_to(node);
		node_path = path;
	}
	if (node_path.is_empty()) {
		return;
	}

	Vector<Vector3> pos_values;
	Vector<float> pos_times;
	Vector<Vector3> scale_values;
	Vector<float> scale_times;
	Vector<Quat> rot_values;
	Vector<float> rot_times;
	Vector3 base_pos;
	Quat base_rot;
	Vector3 base_scale = Vector3(1, 1, 1);

	for (size_t k = 0; k < F.size(); k++) {
		bool is_translation = false;
		bool is_rotation = false;
		bool is_scaling = false;

		String p_track_type = _ai_string_to_string(F[k]->mNodeName).split(ASSIMP_FBX_KEY)[1];
		if (p_track_type == "_Translation") {
			is_translation = true;
		} else if (p_track_type == "_Rotation") {
			is_rotation = true;
		} else if (p_track_type == "_Scaling") {
			is_scaling = true;
		} else {
			continue;
		}
		ERR_CONTINUE(ap->get_owner()->has_node(node_path) == false);

		if (F[k]->mNumRotationKeys || F[k]->mNumPositionKeys || F[k]->mNumScalingKeys) {

			if (is_rotation) {
				for (int i = 0; i < F[k]->mNumRotationKeys; i++) {
					length = MAX(length, F[k]->mRotationKeys[i].mTime / ticks_per_second);
				}
			}
			if (is_translation) {
				for (int i = 0; i < F[k]->mNumPositionKeys; i++) {
					length = MAX(length, F[k]->mPositionKeys[i].mTime / ticks_per_second);
				}
			}
			if (is_scaling) {
				for (int i = 0; i < F[k]->mNumScalingKeys; i++) {
					length = MAX(length, F[k]->mScalingKeys[i].mTime / ticks_per_second);
				}
			}

			if (is_rotation == false && is_translation == false && is_scaling == false) {
				return;
			}

			if (is_rotation) {
				if (F[k]->mNumRotationKeys != 0) {
					aiQuatKey key = F[k]->mRotationKeys[0];
					real_t x = key.mValue.x;
					real_t y = key.mValue.y;
					real_t z = key.mValue.z;
					real_t w = key.mValue.w;
					Quat q(x, y, z, w);
					q = q.normalized();
					base_rot = q;
				}
			}

			if (is_translation) {
				if (F[k]->mNumPositionKeys != 0) {
					aiVectorKey key = F[k]->mPositionKeys[0];
					real_t x = key.mValue.x;
					real_t y = key.mValue.y;
					real_t z = key.mValue.z;
					base_pos = Vector3(x, y, z);
				}
			}

			if (is_scaling) {
				if (F[k]->mNumScalingKeys != 0) {
					aiVectorKey key = F[k]->mScalingKeys[0];
					real_t x = key.mValue.x;
					real_t y = key.mValue.y;
					real_t z = key.mValue.z;
					base_scale = Vector3(x, y, z);
				}
			}
			if (is_translation) {
				for (size_t p = 0; p < F[k]->mNumPositionKeys; p++) {
					aiVector3D pos = F[k]->mPositionKeys[p].mValue;
					pos_values.push_back(Vector3(pos.x, pos.y, pos.z));
					pos_times.push_back(F[k]->mPositionKeys[p].mTime / ticks_per_second);
				}
			}

			if (is_rotation) {
				for (size_t r = 0; r < F[k]->mNumRotationKeys; r++) {
					aiQuaternion quat = F[k]->mRotationKeys[r].mValue;
					rot_values.push_back(Quat(quat.x, quat.y, quat.z, quat.w).normalized());
					rot_times.push_back(F[k]->mRotationKeys[r].mTime / ticks_per_second);
				}
			}

			if (is_scaling) {
				for (size_t sc = 0; sc < F[k]->mNumScalingKeys; sc++) {
					aiVector3D scale = F[k]->mScalingKeys[sc].mValue;
					scale_values.push_back(Vector3(scale.x, scale.y, scale.z));
					scale_times.push_back(F[k]->mScalingKeys[sc].mTime / ticks_per_second);
				}
			}
		}
	}
	int32_t track_idx = animation->get_track_count();
	animation->add_track(Animation::TYPE_TRANSFORM);
	animation->track_set_path(track_idx, node_path);
	float increment = 1.0 / float(p_bake_fps);
	float time = 0.0;
	bool last = false;
	while (true) {
		Vector3 pos = Vector3();
		Quat rot = Quat();
		Vector3 scale = Vector3(1.0f, 1.0f, 1.0f);

		if (pos_values.size()) {
			pos = _interpolate_track<Vector3>(pos_times, pos_values, time, AssetImportAnimation::INTERP_LINEAR);
		}

		if (rot_values.size()) {
			rot = _interpolate_track<Quat>(rot_times, rot_values, time, AssetImportAnimation::INTERP_LINEAR).normalized();
		}

		if (scale_values.size()) {
			scale = _interpolate_track<Vector3>(scale_times, scale_values, time, AssetImportAnimation::INTERP_LINEAR);
		}

		if (sk != NULL && sk->find_bone(p_node_name) != -1) {
			Transform xform;
			//xform.basis = Basis(rot);
			//xform.basis.scale(scale);
			xform.basis.set_quat_scale(rot, scale);

			xform.origin = pos;

			int bone = sk->find_bone(p_node_name);

			//Transform mesh_xform = _ai_matrix_transform(_ai_find_node(p_scene->mRootNode, sk->get_parent()->get_name())->mTransformation);
			//xform = mesh_xform.affine_inverse() * xform;

			Transform rest_xform = sk->get_bone_rest(bone);
			Vector3 scale = rest_xform.basis.get_scale();
			rest_xform.basis.set_quat_scale(Quat(), scale);

			xform = rest_xform.affine_inverse() * xform;

			rot = xform.basis.get_rotation_quat();
			rot.normalize();
			scale = xform.basis.get_scale();
			pos = xform.origin;

			if (p_orig_skeleton_root != p_skeleton_root && p_orig_skeleton_root == p_node_name) {
				aiNode *orig_root = _ai_find_node(p_scene->mRootNode, p_orig_skeleton_root);
				ERR_CONTINUE(orig_root == NULL);
				aiNode *orig_root_parent = orig_root->mParent;
				ERR_CONTINUE(orig_root_parent == NULL);
				for (size_t i = 0; i < orig_root_parent->mNumChildren; i++) {
					if (_ai_string_to_string(orig_root_parent->mChildren[i]->mName) == p_node_name) {
						pos = Vector3();
					}
				}
			}
		}

		if (sk && p_skeleton_root == p_node_name) {
			Transform anim_xform;
			String ext = p_path.get_file().get_extension().to_lower();
			if (ext == "fbx") {
				real_t factor = 1.0f;
				if (p_scene->mMetaData != NULL) {
					p_scene->mMetaData->Get("UnitScaleFactor", factor);
				}
				//factor = factor * 0.01f;
				anim_xform = anim_xform.scaled(Vector3(factor, factor, factor));
			}
			Transform format_xform = _format_rot_xform(p_path, p_scene);
			anim_xform = format_xform * anim_xform;
			Transform xform;
			xform.basis.set_quat_scale(rot, scale);
			xform.origin = pos;

			Transform mesh_xform = _ai_matrix_transform(_ai_find_node(p_scene->mRootNode, sk->get_parent()->get_name())->mTransformation);
			xform = mesh_xform * xform;

			xform = anim_xform * xform;

			rot = xform.basis.get_rotation_quat();
			rot.normalize();
			scale = xform.basis.get_scale();
			pos = xform.origin;
		}

		animation->track_set_interpolation_type(track_idx, Animation::INTERPOLATION_LINEAR);
		animation->transform_track_insert_key(track_idx, time, pos, rot, scale);

		if (last) {
			break;
		}
		time += increment;
		if (time >= length) {
			last = true;
			time = length;
		}
	}
}

float EditorSceneImporterAssetImport::_get_fbx_fps(int32_t time_mode, const aiScene *p_scene) {
	switch (time_mode) {
		case AssetImportFbx::TIME_MODE_DEFAULT: return 24; //hack
		case AssetImportFbx::TIME_MODE_120: return 120;
		case AssetImportFbx::TIME_MODE_100: return 100;
		case AssetImportFbx::TIME_MODE_60: return 60;
		case AssetImportFbx::TIME_MODE_50: return 50;
		case AssetImportFbx::TIME_MODE_48: return 48;
		case AssetImportFbx::TIME_MODE_30: return 30;
		case AssetImportFbx::TIME_MODE_30_DROP: return 30;
		case AssetImportFbx::TIME_MODE_NTSC_DROP_FRAME: return 29.9700262f;
		case AssetImportFbx::TIME_MODE_NTSC_FULL_FRAME: return 29.9700262f;
		case AssetImportFbx::TIME_MODE_PAL: return 25;
		case AssetImportFbx::TIME_MODE_CINEMA: return 24;
		case AssetImportFbx::TIME_MODE_1000: return 1000;
		case AssetImportFbx::TIME_MODE_CINEMA_ND: return 23.976f;
		case AssetImportFbx::TIME_MODE_CUSTOM:
			int32_t frame_rate;
			p_scene->mMetaData->Get("FrameRate", frame_rate);
			return frame_rate;
	}
	return 0;
}

Transform EditorSceneImporterAssetImport::_get_global_ai_node_transform(const aiScene *p_scene, const aiNode *p_current_node) {
	aiNode const *current_node = p_current_node;
	Transform xform;
	while (current_node != NULL) {
		xform = _ai_matrix_transform(current_node->mTransformation) * xform;
		current_node = current_node->mParent;
	}
	return xform;
}

void EditorSceneImporterAssetImport::_generate_node_bone(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton) {
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];
		for (int j = 0; j < ai_mesh->mNumBones; j++) {
			String bone_name = _ai_string_to_string(ai_mesh->mBones[j]->mName);
			if (p_skeleton->find_bone(bone_name) != -1) {
				continue;
			}
			if (bone_name.split(ASSIMP_FBX_KEY).size() != 1) {
				continue;
			}
			p_mesh_bones.insert(bone_name, true);
			p_skeleton->add_bone(bone_name);
			int32_t idx = p_skeleton->find_bone(bone_name);
			Transform xform = _ai_matrix_transform(ai_mesh->mBones[j]->mOffsetMatrix);
			p_skeleton->set_bone_rest(idx, xform.affine_inverse());
		}
	}
}

void EditorSceneImporterAssetImport::_generate_node_bone_parents(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton, const MeshInstance *p_mi) {
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];

		for (int j = 0; j < ai_mesh->mNumBones; j++) {
			aiNode *bone_node = p_scene->mRootNode->FindNode(ai_mesh->mBones[j]->mName);
			ERR_CONTINUE(bone_node == NULL);
			aiNode *bone_node_parent = bone_node->mParent;
			while (bone_node_parent != NULL) {
				String bone_parent_name = _ai_string_to_string(bone_node_parent->mName);

				if (bone_parent_name == p_mi->get_name()) {
					break;
				}
				if (p_mi->get_parent() == NULL) {
					break;
				}
				if (bone_parent_name == p_mi->get_parent()->get_name()) {
					break;
				}
				if (bone_parent_name.split(ASSIMP_FBX_KEY).size() == 1 && p_skeleton->find_bone(bone_parent_name) == -1) {
					p_mesh_bones.insert(bone_parent_name, true);
				}
				bone_node_parent = bone_node_parent->mParent;
			}
		}
	}
}

void EditorSceneImporterAssetImport::_fill_skeleton(const aiScene *p_scene, const aiNode *p_node, const aiNode *p_skeleton_root, Spatial *p_current, Node *p_owner, Skeleton *p_skeleton, const Map<String, bool> p_mesh_bones, const Map<String, Transform> &p_bone_rests, Set<String> p_tracks) {
	String node_name = _ai_string_to_string(p_node->mName);

	if ((p_mesh_bones.find(node_name) == NULL || p_mesh_bones.find(node_name)->get() == false)) {
		return;
	} else if (p_mesh_bones.find(node_name) != NULL && p_skeleton->find_bone(node_name) == -1) {
		p_skeleton->add_bone(node_name);
		int32_t idx = p_skeleton->find_bone(node_name);
		Transform xform = _get_global_ai_node_transform(p_scene, p_node);
		p_skeleton->set_bone_rest(idx, xform);
	}

	for (int i = 0; i < p_node->mNumChildren; i++) {
		_fill_skeleton(p_scene, p_node->mChildren[i], p_skeleton_root, p_current, p_owner, p_skeleton, p_mesh_bones, p_bone_rests, p_tracks);
	}
}

void EditorSceneImporterAssetImport::_keep_node(const String &p_path, Node *p_current, Node *p_owner, Set<Node *> &r_keep_nodes) {
	if (p_current == p_owner) {
		r_keep_nodes.insert(p_current);
	}

	if (p_current->get_class() != Spatial().get_class()) {
		r_keep_nodes.insert(p_current);
	}

	for (int i = 0; i < p_current->get_child_count(); i++) {
		_keep_node(p_path, p_current->get_child(i), p_owner, r_keep_nodes);
	}
}

void EditorSceneImporterAssetImport::_filter_node(const String &p_path, Node *p_current, Node *p_owner, const Set<Node *> p_keep_nodes, Set<String> &r_removed_nodes) {
	if (p_keep_nodes.has(p_current) == false) {
		r_removed_nodes.insert(p_current->get_name());
		p_current->queue_delete();
	}
	for (int i = 0; i < p_current->get_child_count(); i++) {
		_filter_node(p_path, p_current->get_child(i), p_owner, p_keep_nodes, r_removed_nodes);
	}
}

void EditorSceneImporterAssetImport::_generate_node(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_parent, Node *p_owner, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names, Map<Skeleton *, MeshInstance *> &r_skeletons, const Map<String, Transform> &p_bone_rests, Map<MeshInstance *, String> &r_mesh_instances, Map<MeshInstance *, String> &r_orig_mesh_instances, int32_t &r_mesh_count) {
	Spatial *child_node = NULL;
	if (p_node == NULL) {
		return;
	}
	String node_name = _ai_string_to_string(p_node->mName);
	{
		Transform xform = _ai_matrix_transform(p_node->mTransformation);
		String ext = p_path.get_file().get_extension().to_lower();

		child_node = memnew(Spatial);
		p_parent->add_child(child_node);
		child_node->set_owner(p_owner);

		if (p_node == p_scene->mRootNode) {
			if ((ext == "fbx") && p_node == p_scene->mRootNode) {
				real_t factor = 1.0f;
				if (p_scene->mMetaData != NULL) {
					p_scene->mMetaData->Get("UnitScaleFactor", factor);
				}
				factor = factor * 0.01f;
				xform = xform.scaled(Vector3(factor, factor, factor));
			}
			Transform format_xform = _format_rot_xform(p_path, p_scene);
			xform = format_xform * xform;
		}
		child_node->set_transform(xform * child_node->get_transform());
	}
	if (p_node->mNumMeshes > 0) {
		Skeleton *s = NULL;
		MeshInstance *mesh_node = memnew(MeshInstance);
		p_parent->add_child(mesh_node);
		mesh_node->set_owner(p_owner);
		{
			String node_name = p_parent->get_name();
			Map<String, bool> mesh_bones;
			s = memnew(Skeleton);
			_generate_node_bone(p_scene, p_node, mesh_bones, s);
			Set<String> tracks;
			_get_track_set(p_scene, tracks);
			aiNode *ai_orig_skeleton_root = NULL;
			_calculate_skeleton_root(s, p_scene, ai_orig_skeleton_root, mesh_bones, p_node);
			_generate_node_bone_parents(p_scene, p_node, mesh_bones, s, mesh_node);
			aiNode *ai_skeleton_root = NULL;
			_calculate_skeleton_root(s, p_scene, ai_skeleton_root, mesh_bones, p_node);
			if (s->get_bone_count() > 0) {
				_fill_skeleton(p_scene, ai_skeleton_root, ai_skeleton_root, mesh_node, p_owner, s, mesh_bones, p_bone_rests, tracks);
				r_skeletons.insert(s, mesh_node);
				String skeleton_path = s->get_name();
				mesh_node->set_skeleton_path(skeleton_path);
			}

			if (ai_skeleton_root != NULL) {
				r_mesh_instances.insert(Object::cast_to<MeshInstance>(mesh_node), _ai_string_to_string(ai_skeleton_root->mName));
				r_orig_mesh_instances.insert(Object::cast_to<MeshInstance>(mesh_node), _ai_string_to_string(ai_orig_skeleton_root->mName));
			} else {
				r_mesh_instances.insert(Object::cast_to<MeshInstance>(mesh_node), "");
				r_orig_mesh_instances.insert(Object::cast_to<MeshInstance>(mesh_node), "");
			}
			_add_mesh_to_mesh_instance(p_node, p_scene, s, p_path, mesh_node, p_owner, r_bone_name, r_mesh_count);
		}
		if (mesh_node != NULL && s != NULL) {
			if (s->get_bone_count() > 0) {
				s->set_name(node_name + TTR("Skeleton"));
				mesh_node->add_child(s);
				s->set_owner(p_owner);
				mesh_node->set_skeleton_path(NodePath(s->get_name()));
			}
		}
		child_node->get_parent()->remove_child(child_node);
		mesh_node->set_transform(child_node->get_transform());
		memdelete(child_node);
		child_node = mesh_node;
	} else if (p_light_names.has(node_name)) {
		Spatial *light_node = Object::cast_to<Light>(p_owner->find_node(node_name));
		ERR_FAIL_COND(light_node == NULL);
		p_parent->add_child(light_node);
		light_node->set_owner(p_owner);
		light_node->set_transform(child_node->get_transform() * light_node->get_transform());
		child_node->get_parent()->remove_child(child_node);
		memdelete(child_node);
		child_node = light_node;
	} else if (p_camera_names.has(node_name)) {
		Spatial *camera_node = Object::cast_to<Camera>(p_owner->find_node(node_name));
		ERR_FAIL_COND(camera_node == NULL);
		p_parent->add_child(camera_node);
		camera_node->set_owner(p_owner);
		camera_node->set_transform(child_node->get_transform() * camera_node->get_transform());
		child_node->get_parent()->remove_child(child_node);
		memdelete(child_node);
		child_node = camera_node;
	}
	child_node->set_name(node_name);
	for (int i = 0; i < p_node->mNumChildren; i++) {
		_generate_node(p_path, p_scene, p_node->mChildren[i], child_node, p_owner, r_bone_name, p_light_names, p_camera_names, r_skeletons, p_bone_rests, r_mesh_instances, r_orig_mesh_instances, r_mesh_count);
	}
}

void EditorSceneImporterAssetImport::_calculate_skeleton_root(Skeleton *s, const aiScene *p_scene, aiNode *&p_ai_skeleton_root, Map<String, bool> &mesh_bones, const aiNode *p_node) {
	if (s->get_bone_count() > 0) {
		aiNode *ai_child_node = p_scene->mRootNode;
		String bone_name = s->get_bone_name(0);
		p_ai_skeleton_root = _ai_find_node(ai_child_node, bone_name);
	}
	if (p_ai_skeleton_root != NULL) {
		Map<String, bool>::Element *E = mesh_bones.find(_ai_string_to_string(p_ai_skeleton_root->mName));
		while (p_ai_skeleton_root && E && p_ai_skeleton_root->mParent) {
			E = mesh_bones.find(_ai_string_to_string(p_ai_skeleton_root->mParent->mName));
			if (E == NULL || p_ai_skeleton_root->mParent->mName == p_scene->mRootNode->mName) {
				break;
			}
			p_ai_skeleton_root = p_scene->mRootNode->FindNode(p_ai_skeleton_root->mName)->mParent;
		}
	}
	if (p_ai_skeleton_root == NULL) {
		p_ai_skeleton_root = p_scene->mRootNode->FindNode(p_node->mName);
		while (p_ai_skeleton_root && p_ai_skeleton_root->mParent && p_ai_skeleton_root->mParent != p_scene->mRootNode) {
			p_ai_skeleton_root = p_scene->mRootNode->FindNode(p_ai_skeleton_root->mName)->mParent;
		}
	}
}

aiNode *EditorSceneImporterAssetImport::_ai_find_node(aiNode *ai_child_node, const String bone_name) {

	if (_ai_string_to_string(ai_child_node->mName) == bone_name) {
		return ai_child_node;
	}
	aiNode *target = NULL;
	for (int i = 0; i < ai_child_node->mNumChildren; i++) {

		target = _ai_find_node(ai_child_node->mChildren[i], bone_name);
		if (target != NULL) {
			return target;
		}
	}
	return target;
}

Transform EditorSceneImporterAssetImport::_format_rot_xform(const String p_path, const aiScene *p_scene) {
	String ext = p_path.get_file().get_extension().to_lower();

	Transform xform;
	if (ext == "gltf" || ext == "glb") {
		Transform format_xform;
		Quat quat;
		quat.set_euler(Vector3(Math::deg2rad(-90.f), 0.0f, 0.0f));
		format_xform.basis.rotate(quat);
		xform = format_xform * xform;
	}
	if (ext == "fbx") {
		int32_t up_axis = 0;
		Vector3 up_axis_vec3 = Vector3();
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("UpAxis", up_axis);
			if (up_axis == AssetImportFbx::UP_VECTOR_AXIS_X) {
			} else if (up_axis == AssetImportFbx::UP_VECTOR_AXIS_Y) {
				up_axis_vec3 = Vector3(Math::deg2rad(-90.f), 0.0f, 0.0f);
			} else if (up_axis == AssetImportFbx::UP_VECTOR_AXIS_Z) {
				up_axis_vec3 = Vector3(0.0f, Math ::deg2rad(90.f), 0.0f);
			}
		}

		int32_t up_axis_sign = 0;
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("UpAxisSign", up_axis_sign);
			up_axis_vec3 = up_axis_vec3 * up_axis_sign;
		}

		int32_t front_axis = 0;
		Vector3 front_axis_vec3 = Vector3();
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("FrontAxis", front_axis);
			if (front_axis == AssetImportFbx::FRONT_PARITY_EVEN) {
			} else if (front_axis == AssetImportFbx::FRONT_PARITY_ODD) {
			}
		}

		int32_t front_axis_sign = 0;
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("FrontAxisSign", front_axis_sign);
		}

		int32_t coord_axis = 0;
		Vector3 coord_axis_vec3 = Vector3();
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("CoordAxis", coord_axis);
			if (coord_axis == AssetImportFbx::COORD_LEFT) {
			} else if (coord_axis == AssetImportFbx::COORD_RIGHT) {
			}
		}

		int32_t coord_axis_sign = 0;
		if (p_scene->mMetaData != NULL) {
			p_scene->mMetaData->Get("CoordAxisSign", coord_axis_sign);
		}

		Quat up_quat;
		up_quat.set_euler(up_axis_vec3);

		Quat coord_quat;
		coord_quat.set_euler(coord_axis_vec3);
		xform.basis.set_quat(up_quat * coord_quat);
	}
	return xform;
}

void EditorSceneImporterAssetImport::_get_track_set(const aiScene *p_scene, Set<String> &tracks) {
	for (size_t i = 0; i < p_scene->mNumAnimations; i++) {
		for (size_t j = 0; j < p_scene->mAnimations[i]->mNumChannels; j++) {
			aiString ai_name = p_scene->mAnimations[i]->mChannels[j]->mNodeName;
			String name = _ai_string_to_string(ai_name);
			tracks.insert(name);
		}
	}
}

void EditorSceneImporterAssetImport::_add_mesh_to_mesh_instance(const aiNode *p_node, const aiScene *p_scene, Skeleton *s, const String &p_path, MeshInstance *p_mesh_instance, Node *p_owner, Set<String> &r_bone_name, int32_t &r_mesh_count) {
	Ref<ArrayMesh> mesh;
	mesh.instance();
	bool has_uvs = false;
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];

		Map<uint32_t, Vector<float> > vertex_weight;
		Map<uint32_t, Vector<String> > vertex_bone_name;
		for (size_t b = 0; b < ai_mesh->mNumBones; b++) {
			aiBone *bone = ai_mesh->mBones[b];
			for (size_t w = 0; w < bone->mNumWeights; w++) {
				String name = _ai_string_to_string(bone->mName);
				aiVertexWeight ai_weights = bone->mWeights[w];
				uint32_t vertexId = ai_weights.mVertexId;
				Map<uint32_t, Vector<float> >::Element *result = vertex_weight.find(vertexId);
				Vector<float> weights;
				if (result != NULL) {
					weights.append_array(result->value());
				}
				weights.push_back(ai_weights.mWeight);
				if (vertex_weight.has(vertexId)) {
					vertex_weight[vertexId] = weights;
				} else {
					vertex_weight.insert(vertexId, weights);
				}
				Map<uint32_t, Vector<String> >::Element *bone_result = vertex_bone_name.find(vertexId);
				Vector<String> bone_names;
				if (bone_result != NULL) {
					bone_names.append_array(bone_result->value());
				}
				bone_names.push_back(name);
				if (vertex_bone_name.has(vertexId)) {
					vertex_bone_name[vertexId] = bone_names;
				} else {
					vertex_bone_name.insert(vertexId, bone_names);
				}
			}
		}

		Ref<SurfaceTool> st;
		st.instance();
		st->begin(Mesh::PRIMITIVE_TRIANGLES);

		for (size_t j = 0; j < ai_mesh->mNumVertices; j++) {
			if (ai_mesh->HasTextureCoords(0)) {
				has_uvs = true;
				st->add_uv(Vector2(ai_mesh->mTextureCoords[0][j].x, 1.0f - ai_mesh->mTextureCoords[0][j].y));
			}
			if (ai_mesh->HasTextureCoords(1)) {
				has_uvs = true;
				st->add_uv2(Vector2(ai_mesh->mTextureCoords[1][j].x, 1.0f - ai_mesh->mTextureCoords[1][j].y));
			}
			if (ai_mesh->HasVertexColors(0)) {
				Color color = Color(ai_mesh->mColors[0]->r, ai_mesh->mColors[0]->g, ai_mesh->mColors[0]->b, ai_mesh->mColors[0]->a);
				st->add_color(color);
			}
			if (ai_mesh->mNormals != NULL) {
				const aiVector3D normals = ai_mesh->mNormals[j];
				const Vector3 godot_normal = Vector3(normals.x, normals.y, normals.z);
				st->add_normal(godot_normal);
				if (ai_mesh->HasTangentsAndBitangents()) {
					const aiVector3D tangents = ai_mesh->mTangents[j];
					const Vector3 godot_tangent = Vector3(tangents.x, tangents.y, tangents.z);
					const aiVector3D bitangent = ai_mesh->mBitangents[j];
					const Vector3 godot_bitangent = Vector3(bitangent.x, bitangent.y, bitangent.z);
					float d = godot_normal.cross(godot_tangent).dot(godot_bitangent) > 0.0f ? 1.0f : -1.0f;
					st->add_tangent(Plane(tangents.x, tangents.y, tangents.z, d));
				}
			}

			if (s != NULL && s->get_bone_count() > 0) {
				Map<uint32_t, Vector<String> >::Element *I = vertex_bone_name.find(j);
				Vector<int32_t> bones;

				if (I != NULL) {
					Vector<String> bone_names;
					bone_names.append_array(I->value());
					for (size_t f = 0; f < bone_names.size(); f++) {
						int32_t bone = s->find_bone(bone_names[f]);
						ERR_EXPLAIN("Asset Importer: Mesh can't find bone " + bone_names[f]);
						ERR_FAIL_COND(bone == -1);
						bones.push_back(bone);
					}
					if (s->get_bone_count()) {
						int32_t add = CLAMP(VS::ARRAY_WEIGHTS_SIZE - bones.size(), 0, VS::ARRAY_WEIGHTS_SIZE);
						for (size_t f = 0; f < add; f++) {
							bones.push_back(s->get_bone_count() - 1);
						}
					}
					st->add_bones(bones);
					Map<uint32_t, Vector<float> >::Element *E = vertex_weight.find(j);
					Vector<float> weights;
					if (E != NULL) {
						weights = E->value();
						if (weights.size() < VS::ARRAY_WEIGHTS_SIZE) {
							int32_t add = CLAMP(VS::ARRAY_WEIGHTS_SIZE - weights.size(), 0, VS::ARRAY_WEIGHTS_SIZE);
							for (size_t f = 0; f < add; f++) {
								weights.push_back(0.0f);
							}
						}
					}
					ERR_CONTINUE(weights.size() == 0);
					st->add_weights(weights);
				}
			}
			const aiVector3D pos = ai_mesh->mVertices[j];
			Vector3 godot_pos = Vector3(pos.x, pos.y, pos.z);
			st->add_vertex(godot_pos);
		}
		for (size_t j = 0; j < ai_mesh->mNumFaces; j++) {
			const aiFace face = ai_mesh->mFaces[j];
			Vector<size_t> order;
			order.push_back(2);
			order.push_back(1);
			order.push_back(0);
			for (size_t k = 0; k < order.size(); k++) {
				ERR_FAIL_COND(face.mIndices[order[k]] >= st->get_vertex_array().size())
				st->add_index(face.mIndices[order[k]]);
			}
		}
		if (ai_mesh->HasTangentsAndBitangents() == false && has_uvs) {
			st->generate_tangents();
		}
		aiMaterial *ai_material = p_scene->mMaterials[ai_mesh->mMaterialIndex];
		Ref<SpatialMaterial> mat;
		mat.instance();

		int32_t mat_two_sided = 0;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_TWOSIDED, mat_two_sided)) {
			if (mat_two_sided > 0) {
				mat->set_cull_mode(SpatialMaterial::CULL_DISABLED);
			}
		}

		const String mesh_name = _ai_string_to_string(ai_mesh->mName);
		aiString mat_name;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_NAME, mat_name)) {
			mat->set_name(_ai_string_to_string(mat_name));
		}

		aiTextureType tex_normal = aiTextureType_NORMALS;
		{
			aiString ai_filename = aiString();
			String filename = "";
			aiTextureMapMode map_mode[2];

			if (AI_SUCCESS == ai_material->GetTexture(tex_normal, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode)) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);

					if (texture != NULL) {
						if (map_mode != NULL) {
							_set_texture_mapping_mode(map_mode, texture);
						}
						mat->set_feature(SpatialMaterial::Feature::FEATURE_NORMAL_MAPPING, true);
						mat->set_texture(SpatialMaterial::TEXTURE_NORMAL, texture);
					}
				}
			}
		}

		{
			aiString ai_filename = aiString();
			String filename = "";

			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_NORMAL_TEXTURE, ai_filename)) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						mat->set_feature(SpatialMaterial::Feature::FEATURE_NORMAL_MAPPING, true);
						mat->set_texture(SpatialMaterial::TEXTURE_NORMAL, texture);
					}
				}
			}
		}

		aiTextureType tex_emissive = aiTextureType_EMISSIVE;

		if (ai_material->GetTextureCount(tex_emissive) > 0) {

			aiString ai_filename = aiString();
			String filename = "";
			aiTextureMapMode map_mode[2];

			if (AI_SUCCESS == ai_material->GetTexture(tex_emissive, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode)) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						_set_texture_mapping_mode(map_mode, texture);
						mat->set_feature(SpatialMaterial::FEATURE_EMISSION, true);
						mat->set_texture(SpatialMaterial::TEXTURE_EMISSION, texture);
					}
				}
			}
		}

		aiTextureType tex_albedo = aiTextureType_DIFFUSE;
		if (ai_material->GetTextureCount(tex_albedo) > 0) {

			aiString ai_filename = aiString();
			String filename = "";
			aiTextureMapMode map_mode[2];
			if (AI_SUCCESS == ai_material->GetTexture(tex_albedo, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode)) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						if (texture->get_data()->detect_alpha() != Image::ALPHA_NONE) {
							_set_texture_mapping_mode(map_mode, texture);
							mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
							mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
						}
						mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
					}
				}
			}
		} else {
			aiColor4D clr_diffuse;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_COLOR_DIFFUSE, clr_diffuse)) {
				if (Math::is_equal_approx(clr_diffuse.a, 1.0f) == false) {
					mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
					mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
				}
				mat->set_albedo(Color(clr_diffuse.r, clr_diffuse.g, clr_diffuse.b, clr_diffuse.a));
			}
		}

		aiString tex_gltf_base_color_path = aiString();
		aiTextureMapMode map_mode[2];
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_TEXTURE, &tex_gltf_base_color_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_gltf_base_color_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = _load_texture(p_scene, path);
				_find_texture_path(p_path, path, found);
				aiTextureMapMode *map_mode = NULL;
				if (texture != NULL) {
					if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
						_set_texture_mapping_mode(map_mode, texture);
						mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
						mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
					}
					mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
				}
			}
		} else {
			aiColor4D pbr_base_color;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_FACTOR, pbr_base_color)) {
				if (Math::is_equal_approx(pbr_base_color.a, 1.0f) == false) {
					mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
					mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
				}
				mat->set_albedo(Color(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a));
			}
		}
		{
			aiString tex_fbx_pbs_base_color_path = aiString();
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_BASE_COLOR_TEXTURE, tex_fbx_pbs_base_color_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_base_color_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					_find_texture_path(p_path, path, found);
					aiTextureMapMode *map_mode = NULL;
					if (texture != NULL) {
						if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
							_set_texture_mapping_mode(map_mode, texture);
							mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
							mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
						}
						mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
					}
				}
			} else {
				aiColor4D pbr_base_color;
				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_BASE_COLOR_FACTOR, pbr_base_color)) {
					mat->set_albedo(Color(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a));
				}
			}

			aiUVTransform pbr_base_color_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_METALNESS_UV_XFORM, pbr_base_color_uv_xform)) {
				mat->set_uv1_offset(Vector3(pbr_base_color_uv_xform.mTranslation.x, pbr_base_color_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(pbr_base_color_uv_xform.mScaling.x, pbr_base_color_uv_xform.mScaling.y, 1.0f));
			}
		}
		{
			aiString tex_fbx_pbs_base_color_path = aiString();
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_COLOR_TEXTURE, tex_fbx_pbs_base_color_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_base_color_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					_find_texture_path(p_path, path, found);
					aiTextureMapMode *map_mode = NULL;
					if (texture != NULL) {
						if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
							_set_texture_mapping_mode(map_mode, texture);
							mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
							mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
						}
						mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
					}
				}
			} else {
				aiColor4D pbr_base_color;
				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_BASE_COLOR_FACTOR, pbr_base_color)) {
					mat->set_albedo(Color(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a));
				}
			}

			aiUVTransform pbr_base_color_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_COLOR_UV_XFORM, pbr_base_color_uv_xform)) {
				mat->set_uv1_offset(Vector3(pbr_base_color_uv_xform.mTranslation.x, pbr_base_color_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(pbr_base_color_uv_xform.mScaling.x, pbr_base_color_uv_xform.mScaling.y, 1.0f));
			}
		}

		{
			aiString tex_fbx_pbs_emissive_path = aiString();
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_EMISSIVE_TEXTURE, tex_fbx_pbs_emissive_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_emissive_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					_find_texture_path(p_path, path, found);
					aiTextureMapMode *map_mode = NULL;
					if (texture != NULL) {
						if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
							_set_texture_mapping_mode(map_mode, texture);
							mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
							mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
						}
						mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
					}
				}
			} else {
				aiColor4D pbr_emmissive_color;
				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_EMISSIVE_FACTOR, pbr_emmissive_color)) {
					mat->set_emission(Color(pbr_emmissive_color.r, pbr_emmissive_color.g, pbr_emmissive_color.b, pbr_emmissive_color.a));
				}
			}

			real_t pbr_emission_intensity;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_EMISSIVE_INTENSITY_FACTOR, pbr_emission_intensity)) {
				mat->set_emission_energy(pbr_emission_intensity);
			}

			aiUVTransform pbr_base_color_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_EMISSIVE_UV_XFORM, pbr_base_color_uv_xform)) {
				mat->set_uv1_offset(Vector3(pbr_base_color_uv_xform.mTranslation.x, pbr_base_color_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(pbr_base_color_uv_xform.mScaling.x, pbr_base_color_uv_xform.mScaling.y, 1.0f));
			}
		}

		aiString tex_gltf_pbr_metallicroughness_path;
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE, &tex_gltf_pbr_metallicroughness_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_gltf_pbr_metallicroughness_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = _load_texture(p_scene, path);
				if (texture != NULL) {
					_set_texture_mapping_mode(map_mode, texture);
					mat->set_texture(SpatialMaterial::TEXTURE_METALLIC, texture);
					mat->set_metallic_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_BLUE);
					mat->set_texture(SpatialMaterial::TEXTURE_ROUGHNESS, texture);
					mat->set_roughness_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GREEN);
				}
			}
		} else {
			float pbr_roughness = 0.0f;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_ROUGHNESS_FACTOR, pbr_roughness)) {
				mat->set_roughness(pbr_roughness);
			}
			float pbr_metallic = 0.0f;

			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLIC_FACTOR, pbr_metallic)) {
				mat->set_metallic(pbr_metallic);
			}
		}
		{
			aiString tex_fbx_pbs_metallic_path;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_METALLIC_TEXTURE, tex_fbx_pbs_metallic_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_metallic_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						mat->set_texture(SpatialMaterial::TEXTURE_METALLIC, texture);
						mat->set_metallic_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
					}
				}
			} else {
				float pbr_metallic = 0.0f;
				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_METALLIC_FACTOR, pbr_metallic)) {
					mat->set_metallic(pbr_metallic);
				}
			}

			aiUVTransform metalness_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_METALLIC_UV_XFORM, metalness_uv_xform)) {
				mat->set_uv1_offset(Vector3(metalness_uv_xform.mTranslation.x, metalness_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(metalness_uv_xform.mScaling.x, metalness_uv_xform.mScaling.y, 1.0f));
			}

			aiString tex_fbx_pbs_rough_path;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_ROUGHNESS_TEXTURE, tex_fbx_pbs_rough_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_rough_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						mat->set_texture(SpatialMaterial::TEXTURE_ROUGHNESS, texture);
						mat->set_roughness_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
					}
				}
			} else {
				float pbr_roughness = 0.04f;

				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_ROUGHNESS_FACTOR, pbr_roughness)) {
					mat->set_roughness(pbr_roughness);
				}
			}

			aiUVTransform roughness_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_STINGRAY_ROUGHNESS_UV_XFORM, roughness_uv_xform)) {
				mat->set_uv1_offset(Vector3(roughness_uv_xform.mTranslation.x, roughness_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(roughness_uv_xform.mScaling.x, roughness_uv_xform.mScaling.y, 1.0f));
			}
		}

		{
			aiString tex_fbx_pbs_metallic_path;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_METALNESS_TEXTURE, tex_fbx_pbs_metallic_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_metallic_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						mat->set_texture(SpatialMaterial::TEXTURE_METALLIC, texture);
						mat->set_metallic_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
					}
				}
			} else {
				float pbr_metallic = 0.0f;
				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_METALNESS_FACTOR, pbr_metallic)) {
					mat->set_metallic(pbr_metallic);
				}
			}

			aiUVTransform metalness_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_METALNESS_UV_XFORM, metalness_uv_xform)) {
				mat->set_uv1_offset(Vector3(metalness_uv_xform.mTranslation.x, metalness_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(metalness_uv_xform.mScaling.x, metalness_uv_xform.mScaling.y, 1.0f));
			}

			aiString tex_fbx_pbs_rough_path;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_DIFFUSE_ROUGHNESS_TEXTURE, tex_fbx_pbs_rough_path)) {
				String filename = _ai_raw_string_to_string(tex_fbx_pbs_rough_path);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = _load_texture(p_scene, path);
					if (texture != NULL) {
						mat->set_texture(SpatialMaterial::TEXTURE_ROUGHNESS, texture);
						mat->set_roughness_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
					}
				}
			} else {
				float pbr_roughness = 0.04f;

				if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_DIFFUSE_ROUGHNESS_FACTOR, pbr_roughness)) {
					mat->set_roughness(pbr_roughness);
				}
			}

			aiUVTransform roughness_uv_xform;
			if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_MAYA_DIFFUSE_ROUGHNESS_UV_XFORM, roughness_uv_xform)) {
				mat->set_uv1_offset(Vector3(roughness_uv_xform.mTranslation.x, roughness_uv_xform.mTranslation.y, 0.0f));
				mat->set_uv1_scale(Vector3(roughness_uv_xform.mScaling.x, roughness_uv_xform.mScaling.y, 1.0f));
			}
		}

		Array array_mesh = st->commit_to_arrays();
		Array morphs;
		Mesh::PrimitiveType primitive = Mesh::PRIMITIVE_TRIANGLES;

		for (int i = 0; i < ai_mesh->mNumAnimMeshes; i++) {

			String ai_anim_mesh_name = _ai_string_to_string(ai_mesh->mAnimMeshes[i]->mName);
			mesh->set_blend_shape_mode(Mesh::BLEND_SHAPE_MODE_NORMALIZED);
			if (ai_anim_mesh_name.empty()) {
				ai_anim_mesh_name = String("morph_") + itos(i);
			}
			mesh->add_blend_shape(ai_anim_mesh_name);

			Array array_copy;
			array_copy.resize(Mesh::ARRAY_MAX);

			for (int l = 0; l < Mesh::ARRAY_MAX; l++) {
				array_copy[l] = array_mesh[l].duplicate(true);
			}

			const uint32_t num_vertices = ai_mesh->mAnimMeshes[i]->mNumVertices;
			array_copy[Mesh::ARRAY_INDEX] = Variant();
			if (ai_mesh->mAnimMeshes[i]->HasPositions()) {
				Vector<Vector3> vertices;
				vertices.resize(num_vertices);
				for (int l = 0; l < num_vertices; l++) {
					const aiVector3D ai_pos = ai_mesh->mAnimMeshes[i]->mVertices[l];
					Vector3 position = Vector3(ai_pos.x, ai_pos.y, ai_pos.z);
					vertices.write[l] = position;
				}
				PoolVector3Array original_vertices = array_copy[Mesh::ARRAY_VERTEX].duplicate(true);

				for (int l = 0; l < vertices.size(); l++) {
					PoolVector3Array::Write w = original_vertices.write();
					w[l] = vertices[l];
				}
				ERR_CONTINUE(vertices.size() != original_vertices.size());
				array_copy[Mesh::ARRAY_VERTEX] = original_vertices;
			}

			int32_t color_set = 0;
			if (ai_mesh->mAnimMeshes[i]->HasVertexColors(color_set)) {
				Vector<Color> colors;
				colors.resize(num_vertices);
				for (int l = 0; l < num_vertices; l++) {
					const aiColor4D ai_color = ai_mesh->mAnimMeshes[i]->mColors[color_set][l];
					Color color = Color(ai_color.r, ai_color.g, ai_color.b, ai_color.a);
					colors.write[l] = color;
				}
				PoolColorArray original_colors = array_copy[Mesh::ARRAY_COLOR].duplicate(true);

				for (int l = 0; l < colors.size(); l++) {
					PoolColorArray::Write w = original_colors.write();
					w[l] = colors[l];
				}
				array_copy[Mesh::ARRAY_COLOR] = original_colors;
			}

			if (ai_mesh->mAnimMeshes[i]->HasNormals()) {
				Vector<Vector3> normals;
				normals.resize(num_vertices);
				for (int l = 0; l < num_vertices; l++) {
					const aiVector3D ai_normal = ai_mesh->mAnimMeshes[i]->mNormals[l];
					Vector3 normal = Vector3(ai_normal.x, ai_normal.y, ai_normal.z);
					normals.write[l] = normal;
				}
				PoolVector3Array original_normals = array_copy[Mesh::ARRAY_NORMAL].duplicate(true);

				for (int l = 0; l < normals.size(); l++) {
					PoolVector3Array::Write w = original_normals.write();
					w[l] = normals[l];
				}
				array_copy[Mesh::ARRAY_NORMAL] = original_normals;
			}

			if (ai_mesh->mAnimMeshes[i]->HasTangentsAndBitangents()) {
				PoolColorArray tangents;
				tangents.resize(num_vertices);
				PoolColorArray::Write w = tangents.write();
				for (int l = 0; l < num_vertices; l++) {
					_calc_tangent_from_mesh(ai_mesh, i, l, l, w);
				}
				PoolColorArray original_tangents = array_copy[Mesh::ARRAY_TANGENT].duplicate(true);

				for (int l = 0; l < tangents.size(); l++) {
					PoolColorArray::Write w = original_tangents.write();
					w[l] = tangents[l];
				}
				array_copy[Mesh::ARRAY_TANGENT] = original_tangents;
			}

			morphs.push_back(array_copy);
		}

		mesh->add_surface_from_arrays(primitive, array_mesh, morphs);
		mesh->surface_set_material(i, mat);
		mesh->surface_set_name(i, _ai_string_to_string(ai_mesh->mName));
		r_mesh_count++;
		print_line(String("Open Asset Importer: Created mesh (including instances) ") + _ai_string_to_string(ai_mesh->mName) + " " + itos(r_mesh_count) + " of " + itos(p_scene->mNumMeshes));
	}
	p_mesh_instance->set_mesh(mesh);
}

Ref<Texture> EditorSceneImporterAssetImport::_load_texture(const aiScene *p_scene, String p_path) {
	Vector<String> split_path = p_path.get_basename().split("*");
	if (split_path.size() == 2) {
		int32_t texture_idx = split_path[1].to_int();
		ERR_FAIL_COND_V(texture_idx >= p_scene->mNumTextures, Ref<Texture>());
		aiTexture *tex = p_scene->mTextures[texture_idx];
		String filename = _ai_raw_string_to_string(tex->mFilename);
		filename = filename.get_file();
		print_verbose("Open Asset Importer: Loading embedded texture " + filename);
		if (tex->mHeight == 0) {
			if (tex->CheckFormat("png")) {
				Ref<Image> img = Image::_png_mem_loader_func((uint8_t *)tex->pcData, tex->mWidth);
				ERR_FAIL_COND_V(img.is_null(), Ref<Texture>());

				Ref<ImageTexture> t;
				t.instance();
				t->create_from_image(img);
				t->set_storage(ImageTexture::STORAGE_COMPRESS_LOSSY);
				return t;
			}
			if (tex->CheckFormat("jpg")) {
				Ref<Image> img = Image::_jpg_mem_loader_func((uint8_t *)tex->pcData, tex->mWidth);
				ERR_FAIL_COND_V(img.is_null(), Ref<Texture>());

				Ref<ImageTexture> t;
				t.instance();
				t->create_from_image(img);
				t->set_storage(ImageTexture::STORAGE_COMPRESS_LOSSY);
				return t;
			}
		} else {
			Ref<Image> img;
			img.instance();
			PoolByteArray arr;
			uint32_t size = tex->mWidth * tex->mHeight;
			arr.resize(size);
			memcpy(arr.write().ptr(), tex->pcData, size);
			ERR_FAIL_COND_V(arr.size() % 4 != 0, Ref<Texture>());
			//ARGB8888 to RGBA8888
			for (size_t i = 0; i < arr.size() / 4; i++) {
				arr.write().ptr()[(4 * i) + 3] = arr[(4 * i) + 0];
				arr.write().ptr()[(4 * i) + 0] = arr[(4 * i) + 1];
				arr.write().ptr()[(4 * i) + 1] = arr[(4 * i) + 2];
				arr.write().ptr()[(4 * i) + 2] = arr[(4 * i) + 3];
			}
			img->create(tex->mWidth, tex->mHeight, true, Image::FORMAT_RGBA8, arr);
			ERR_FAIL_COND_V(img.is_null(), Ref<Texture>());

			Ref<ImageTexture> t;
			t.instance();
			t->create_from_image(img);
			t->set_storage(ImageTexture::STORAGE_COMPRESS_LOSSY);
			return t;
		}
		return Ref<Texture>();
	}
	Ref<Texture> p_texture = ResourceLoader::load(p_path, "Texture");
	return p_texture;
}

void EditorSceneImporterAssetImport::_calc_tangent_from_mesh(const aiMesh *ai_mesh, int i, int tri_index, int index, PoolColorArray::Write &w) {
	const aiVector3D normals = ai_mesh->mAnimMeshes[i]->mNormals[tri_index];
	const Vector3 godot_normal = Vector3(normals.x, normals.y, normals.z);
	const aiVector3D tangent = ai_mesh->mAnimMeshes[i]->mTangents[tri_index];
	const Vector3 godot_tangent = Vector3(tangent.x, tangent.y, tangent.z);
	const aiVector3D bitangent = ai_mesh->mAnimMeshes[i]->mBitangents[tri_index];
	const Vector3 godot_bitangent = Vector3(bitangent.x, bitangent.y, bitangent.z);
	float d = godot_normal.cross(godot_tangent).dot(godot_bitangent) > 0.0f ? 1.0f : -1.0f;
	Color plane_tangent = Color(tangent.x, tangent.y, tangent.z, d);
	w[index] = plane_tangent;
}

void EditorSceneImporterAssetImport::_set_texture_mapping_mode(aiTextureMapMode *map_mode, Ref<Texture> texture) {
	ERR_FAIL_COND(map_mode == NULL);
	aiTextureMapMode tex_mode = aiTextureMapMode::aiTextureMapMode_Wrap;
	//for (size_t i = 0; i < 3; i++) {
	tex_mode = map_mode[0];
	//}
	int32_t flags = Texture::FLAGS_DEFAULT;
	if (tex_mode == aiTextureMapMode_Wrap) {
		//Default
	} else if (tex_mode == aiTextureMapMode_Clamp) {
		flags = flags & ~Texture::FLAG_REPEAT;
	} else if (tex_mode == aiTextureMapMode_Mirror) {
		flags = flags | Texture::FLAG_MIRRORED_REPEAT;
	}
	texture->set_flags(flags);
}

void EditorSceneImporterAssetImport::_find_texture_path(const String &r_p_path, String &r_path, bool &r_found) {

	_Directory dir;

	List<String> exts;
	ImageLoader::get_recognized_extensions(&exts);

	Vector<String> split_path = r_path.get_basename().split("*");
	if (split_path.size() == 2) {
		r_found = true;
		return;
	}

	if (dir.file_exists(r_p_path.get_base_dir() + r_path)) {
		r_path = r_p_path.get_base_dir() + r_path;
		r_found = true;
		return;
	}

	for (size_t i = 0; i < exts.size(); i++) {
		if (r_found) {
			return;
		}
		if (r_found == false) {
			_find_texture_path(r_p_path, dir, r_path, r_found, exts[i]);
		}
	}
}

void EditorSceneImporterAssetImport::_find_texture_path(const String &p_path, _Directory &dir, String &path, bool &found, String extension) {
	String name = path.get_basename() + extension;
	if (dir.file_exists(name)) {
		found = true;
		path = name;
		return;
	}
	String name_ignore_sub_directory = p_path.get_base_dir() + "/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_ignore_sub_directory)) {
		found = true;
		path = name_ignore_sub_directory;
		return;
	}

	String name_find_texture_sub_directory = p_path.get_base_dir() + "/textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_sub_directory)) {
		found = true;
		path = name_find_texture_sub_directory;
		return;
	}
	String name_find_texture_upper_sub_directory = p_path.get_base_dir() + "/Textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_upper_sub_directory)) {
		found = true;
		path = name_find_texture_upper_sub_directory;
		return;
	}
	String name_find_texture_outside_sub_directory = p_path.get_base_dir() + "/../textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_outside_sub_directory)) {
		found = true;
		path = name_find_texture_outside_sub_directory;
		return;
	}

	String name_find_upper_texture_outside_sub_directory = p_path.get_base_dir() + "/../Textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_upper_texture_outside_sub_directory)) {
		found = true;
		path = name_find_upper_texture_outside_sub_directory;
		return;
	}
}

String EditorSceneImporterAssetImport::_ai_string_to_string(const aiString p_string) {
	Vector<char> raw_name;
	raw_name.resize(p_string.length);
	memcpy(raw_name.ptrw(), p_string.C_Str(), p_string.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	if (name.find(":") != -1) {
		String replaced_name = name.split(":")[1];
		print_verbose("Replacing " + name + " containing : with " + replaced_name);
		name = replaced_name;
	}
	if (name.find(".") != -1) {
		String replaced_name = name.replace(".", "");
		print_verbose("Replacing " + name + " containing . with " + replaced_name);
		name = replaced_name;
	}
	return name;
}

String EditorSceneImporterAssetImport::_ai_anim_string_to_string(const aiString p_string) {
	Vector<char> raw_name;
	raw_name.resize(p_string.length);
	memcpy(raw_name.ptrw(), p_string.C_Str(), p_string.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	if (name.find(":") != -1) {
		String replaced_name = name.split(":")[1];
		print_verbose("Replacing " + name + " containing : with " + replaced_name);
		name = replaced_name;
	}
	return name;
}

String EditorSceneImporterAssetImport::_ai_raw_string_to_string(const aiString p_string) {
	Vector<char> raw_name;
	raw_name.resize(p_string.length);
	memcpy(raw_name.ptrw(), p_string.C_Str(), p_string.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	return name;
}

Ref<Animation> EditorSceneImporterAssetImport::import_animation(const String &p_path, uint32_t p_flags, int p_bake_fps) {
	return Ref<Animation>();
}

const Transform EditorSceneImporterAssetImport::_ai_matrix_transform(const aiMatrix4x4 p_matrix) {
	aiMatrix4x4 matrix = p_matrix;
	Transform xform;
	xform.set(matrix.a1, matrix.b1, matrix.c1, matrix.a2, matrix.b2, matrix.c2, matrix.a3, matrix.b3, matrix.c3, matrix.a4, matrix.b4, matrix.c4);
	xform.basis.inverse();
	xform.basis.transpose();
	Vector3 scale = xform.basis.get_scale();
	Quat rot = xform.basis.get_rotation_quat();
	xform.basis.set_quat_scale(rot, scale);
	return xform;
}
