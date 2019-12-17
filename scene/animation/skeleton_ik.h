/*************************************************************************/
/*  skeleton_ik.h                                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2020 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2020 Godot Engine contributors (cf. AUTHORS.md).   */
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

#ifndef SKELETON_IK_H
#define SKELETON_IK_H

#ifndef _3D_DISABLED

/**
 * @author AndreaCatania
 */

#include "core/math/transform.h"
#include "scene/3d/skeleton.h"

class FabrikInverseKinematic {

	struct EndEffector {
		BoneId tip_bone;
		Transform goal_transform;
	};

	struct ChainItem {

		Vector<ChainItem> children;
		ChainItem *parent_item;

		// Bone info
		BoneId bone;
		PhysicalBone *pb;

		real_t length;
		/// Positions relative to root bone
		Transform initial_transform;
		Vector3 current_pos;
		// Direction from this bone to child
		Vector3 current_ori;

		ChainItem() :
				parent_item(NULL),
				bone(-1),
				pb(NULL),
				length(0) {}

		ChainItem *find_child(const BoneId p_bone_id);
		ChainItem *add_child(const BoneId p_bone_id);
	};

	struct ChainTip {
		ChainItem *chain_item;
		const EndEffector *end_effector;

		ChainTip() :
				chain_item(NULL),
				end_effector(NULL) {}

		ChainTip(ChainItem *p_chain_item, const EndEffector *p_end_effector) :
				chain_item(p_chain_item),
				end_effector(p_end_effector) {}

		ChainTip(const ChainTip &p_other_ct) :
				chain_item(p_other_ct.chain_item),
				end_effector(p_other_ct.end_effector) {}
	};

	struct Chain {
		ChainItem chain_root;
		ChainItem *middle_chain_item;
		Vector<ChainTip> tips;
		Vector3 magnet_position;
	};

public:
	struct Task : public RID_Data {
		RID self;
		Skeleton *skeleton;

		Chain chain;

		// Settings
		real_t min_distance;
		int max_iterations;

		// Bone data
		BoneId root_bone;
		Vector<EndEffector> end_effectors;

		Transform goal_global_transform;

		Task() :
				skeleton(NULL),
				min_distance(0.01),
				max_iterations(10),
				root_bone(-1) {}
	};

private:
	/// Init a chain that starts from the root to tip
	static bool build_chain(Task *p_task, bool p_force_simple_chain = true);

	static void update_chain(const Skeleton *p_sk, ChainItem *p_chain_item);

	static void solve_simple(Task *p_task, bool p_solve_magnet);
	/// Special solvers that solve only chains with one end effector
	static void solve_simple_backwards(Chain &r_chain, bool p_solve_magnet);
	static void solve_simple_forwards(Chain &r_chain, bool p_solve_magnet);

public:
	static Task *create_simple_task(Skeleton *p_sk, BoneId root_bone, BoneId tip_bone, const Transform &goal_transform);
	static void free_task(Task *p_task);
	// The goal of chain should be always in local space
	static void set_goal(Task *p_task, const Transform &p_goal);
	static void make_goal(Task *p_task, const Transform &p_inverse_transf, real_t blending_delta);
	static void solve(Task *p_task, real_t blending_delta, bool override_tip_basis, bool p_use_magnet, const Vector3 &p_magnet_position);
};

class SkeletonIKBase : public Node {
	GDCLASS(SkeletonIKBase, Node);

protected:
	virtual void
	_validate_property(PropertyInfo &property) const {}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("set_root_bone", "root_bone"), &SkeletonIKBase::set_root_bone);
		ClassDB::bind_method(D_METHOD("get_root_bone"), &SkeletonIKBase::get_root_bone);

		ClassDB::bind_method(D_METHOD("set_tip_bone", "tip_bone"), &SkeletonIKBase::set_tip_bone);
		ClassDB::bind_method(D_METHOD("get_tip_bone"), &SkeletonIKBase::get_tip_bone);

		ClassDB::bind_method(D_METHOD("set_interpolation", "interpolation"), &SkeletonIKBase::set_interpolation);
		ClassDB::bind_method(D_METHOD("get_interpolation"), &SkeletonIKBase::get_interpolation);

		ClassDB::bind_method(D_METHOD("set_target_transform", "target"), &SkeletonIKBase::set_target_transform);
		ClassDB::bind_method(D_METHOD("get_target_transform"), &SkeletonIKBase::get_target_transform);

		ClassDB::bind_method(D_METHOD("set_target_node", "node"), &SkeletonIKBase::set_target_node);
		ClassDB::bind_method(D_METHOD("get_target_node"), &SkeletonIKBase::get_target_node);

		ClassDB::bind_method(D_METHOD("set_override_tip_basis", "override"), &SkeletonIKBase::set_override_tip_basis);
		ClassDB::bind_method(D_METHOD("is_override_tip_basis"), &SkeletonIKBase::is_override_tip_basis);

		ClassDB::bind_method(D_METHOD("set_use_magnet", "use"), &SkeletonIKBase::set_use_magnet);
		ClassDB::bind_method(D_METHOD("is_using_magnet"), &SkeletonIKBase::is_using_magnet);

		ClassDB::bind_method(D_METHOD("set_magnet_position", "local_position"), &SkeletonIKBase::set_magnet_position);
		ClassDB::bind_method(D_METHOD("get_magnet_position"), &SkeletonIKBase::get_magnet_position);

		ClassDB::bind_method(D_METHOD("get_parent_skeleton"), &SkeletonIKBase::get_parent_skeleton);
		ClassDB::bind_method(D_METHOD("is_running"), &SkeletonIKBase::is_running);

		ClassDB::bind_method(D_METHOD("set_min_distance", "min_distance"), &SkeletonIKBase::set_min_distance);
		ClassDB::bind_method(D_METHOD("get_min_distance"), &SkeletonIKBase::get_min_distance);

		ClassDB::bind_method(D_METHOD("set_max_iterations", "iterations"), &SkeletonIKBase::set_max_iterations);
		ClassDB::bind_method(D_METHOD("get_max_iterations"), &SkeletonIKBase::get_max_iterations);

		ClassDB::bind_method(D_METHOD("start", "one_time"), &SkeletonIKBase::start, DEFVAL(false));
		ClassDB::bind_method(D_METHOD("stop"), &SkeletonIKBase::stop);

		ADD_PROPERTY(PropertyInfo(Variant::STRING, "root_bone"), "set_root_bone", "get_root_bone");
		ADD_PROPERTY(PropertyInfo(Variant::STRING, "tip_bone"), "set_tip_bone", "get_tip_bone");
		ADD_PROPERTY(PropertyInfo(Variant::REAL, "interpolation", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_interpolation", "get_interpolation");
		ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM, "target"), "set_target_transform", "get_target_transform");
		ADD_PROPERTY(PropertyInfo(Variant::BOOL, "override_tip_basis"), "set_override_tip_basis", "is_override_tip_basis");
		ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_magnet"), "set_use_magnet", "is_using_magnet");
		ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "magnet"), "set_magnet_position", "get_magnet_position");
		ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "target_node"), "set_target_node", "get_target_node");
		ADD_PROPERTY(PropertyInfo(Variant::REAL, "min_distance"), "set_min_distance", "get_min_distance");
		ADD_PROPERTY(PropertyInfo(Variant::INT, "max_iterations"), "set_max_iterations", "get_max_iterations");
	}
	virtual void _notification(int p_what) {}

public:
	SkeletonIKBase() {}
	virtual ~SkeletonIKBase() {}

	virtual void set_root_bone(const StringName &p_root_bone) = 0;
	virtual StringName get_root_bone() const = 0;

	virtual void set_tip_bone(const StringName &p_tip_bone) = 0;
	virtual StringName get_tip_bone() const = 0;

	virtual void set_interpolation(real_t p_interpolation) = 0;
	virtual real_t get_interpolation() const = 0;

	virtual void set_target_transform(const Transform &p_target) = 0;
	virtual const Transform &get_target_transform() const = 0;

	virtual void set_target_node(const NodePath &p_node) = 0;
	virtual NodePath get_target_node() = 0;

	virtual void set_override_tip_basis(bool p_override) = 0;
	virtual bool is_override_tip_basis() const = 0;

	virtual void set_use_magnet(bool p_use) = 0;
	virtual bool is_using_magnet() const = 0;

	virtual void set_magnet_position(const Vector3 &p_local_position) = 0;
	virtual const Vector3 &get_magnet_position() const = 0;

	virtual void set_min_distance(real_t p_min_distance) = 0;
	virtual real_t get_min_distance() const = 0;

	virtual void set_max_iterations(int p_iterations) = 0;
	virtual int get_max_iterations() const = 0;

	virtual Skeleton *get_parent_skeleton() const = 0;

	virtual bool is_running() = 0;

	virtual void start(bool p_one_time = false) = 0;
	virtual void stop() = 0;
};

class SkeletonIK : public SkeletonIKBase {
	GDCLASS(SkeletonIK, SkeletonIKBase);

	StringName root_bone;
	StringName tip_bone;
	real_t interpolation;
	Transform target;
	NodePath target_node_path_override;
	bool override_tip_basis;
	bool use_magnet;
	Vector3 magnet_position;

	real_t min_distance;
	int max_iterations;

	Skeleton *skeleton;
	Spatial *target_node_override;
	FabrikInverseKinematic::Task *task;

protected:
	virtual void
	_validate_property(PropertyInfo &property) const;

	static void _bind_methods();
	virtual void _notification(int p_what);

public:
	SkeletonIK();
	virtual ~SkeletonIK();

	void set_root_bone(const StringName &p_root_bone);
	StringName get_root_bone() const;

	void set_tip_bone(const StringName &p_tip_bone);
	StringName get_tip_bone() const;

	void set_interpolation(real_t p_interpolation);
	real_t get_interpolation() const;

	void set_target_transform(const Transform &p_target);
	const Transform &get_target_transform() const;

	void set_target_node(const NodePath &p_node);
	NodePath get_target_node();

	void set_override_tip_basis(bool p_override);
	bool is_override_tip_basis() const;

	void set_use_magnet(bool p_use);
	bool is_using_magnet() const;

	void set_magnet_position(const Vector3 &p_local_position);
	const Vector3 &get_magnet_position() const;

	void set_min_distance(real_t p_min_distance);
	real_t get_min_distance() const { return min_distance; }

	void set_max_iterations(int p_iterations);
	int get_max_iterations() const { return max_iterations; }

	Skeleton *get_parent_skeleton() const { return skeleton; }

	bool is_running();

	void start(bool p_one_time = false);
	void stop();

private:
	Transform _get_target_transform();
	void reload_chain();
	void reload_goal();
	void _solve_chain();
};

#endif // _3D_DISABLED

#endif // SKELETON_IK_H
