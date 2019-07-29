/*************************************************************************/
/*  skeleton_ik.h                                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
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

	struct ChainItem;

	// IKConstraints based on
	// https://github.com/hacoo/rtik
	struct IKConstraint {
		virtual bool initialize() { return true; }
		virtual void enforce_constraint(
				ChainItem *item) = 0;
		virtual void setup(ChainItem *item) { return; };
	};

	struct IKConstraintPlanarRotation : public IKConstraint {
		// It should be normalized.
		// The Vector is in component space.
		// The bone direction (parent to child) will rotate around this vector, based at the parent.
		Vector3 rotation_axis;

		// It should be normalized and normal to RotationAxis.
		// Vector in component space.
		// This represents the '0 degree' rotation; for example, the bone has a rotation of 0 degrees if the parent-child vector points in this direction.
		Vector3 forward_direction;

		// The vector must be normalized.
		// Vector in component space.
		// The bone will point in this direction if the constraint method fails (i.e., if the bone direction is normal to the rotation plane).
		Vector3 failsafe_direction;

		// The maximum angle in the positive direction (toward rotation_axis X forward_direction), relative to forward_direction
		real_t max_degree;

		// The minimum angle in the positive direction (toward rotation_axis X forward_direction), relative to forward_direction
		real_t min_degree;

		virtual bool initialize() override {
			// make sure axes are normalized; compute up axis
			bool bAxesOK = true;
			bAxesOK &= forward_direction.is_normalized();
			bAxesOK &= rotation_axis.is_normalized();
			bAxesOK &= failsafe_direction.is_normalized();

			ERR_EXPLAIN("Planar Rotation Constraint was set up incorrectly. Forward direction direction and rotation axis must not be colinear.")
			ERR_FAIL_COND_V(!bAxesOK, false);
		}
		virtual void enforce_constraint(
				ChainItem *item) override {
			if (!item->children.size()) {
				// This constraint is not meaningful on the tip bone
				return;
			}

			for (int32_t i = 0; i < item->children.size(); i++) {
				Vector3 up_direction = rotation_axis.cross(forward_direction);

				Vector3 parent_loc = item->parent_item->current_pos;
				Vector3 child_loc = item->children[0].current_pos;

				// Step 1: project onto rotation plane
				Vector3 bone_direction = (child_loc - parent_loc).project(rotation_axis);
				float bone_length = (child_loc - parent_loc).length();

				if (!bone_direction.is_normalized()) {
					bone_direction = failsafe_direction;
				}

				// Step 2: Find the current angle
				float angle_rad = (bone_direction.dot(up_direction) > 0.0f) ?
										  Math::acos(bone_direction.dot(forward_direction)) :
										  -1 * Math::acos(bone_direction.dot(forward_direction));

				// Step 3: clamp to within allowed angle
				float target_deg = CLAMP(Math::rad2deg(angle_rad), min_degree, max_degree);

				bone_direction = forward_direction.rotated(rotation_axis, target_deg);

				bone_direction *= bone_length;

				// Move the child. Don't update the rotations yet; that's done in the fabrik solver.
				item->children.write[i].current_pos = parent_loc + bone_direction;
			}
		}

		IKConstraintPlanarRotation() :
				rotation_axis(0.0f, 1.0f, 10.0f),
				forward_direction(1.0f, 0.0f, 0.0f),
				failsafe_direction(1.0f, 0.0f, 0.0f),
				max_degree(45.0f),
				min_degree(-45.0f) {}
	};

	struct ChainItem {

		Vector<ChainItem> children;
		ChainItem *parent_item;

		// Bone info
		BoneId bone;
		PhysicalBone *pb;

		IKConstraint *constraint;

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
				constraint(NULL),
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
		real_t max_root_drag_distance;
		real_t root_drag_stiffness;

		// Bone data
		BoneId root_bone;
		Vector<EndEffector> end_effectors;

		Transform goal_global_transform;

		Task() :
				skeleton(NULL),
				min_distance(0.01),
				max_iterations(10),
				max_root_drag_distance(10),
				root_drag_stiffness(1.0),
				root_bone(-1) {}
	};

	enum {
		IK_SOLVER_NORMAL,
		IK_SOLVER_CLOSED_LOOP,
	};

private:
	/// Init a chain that starts from the root to tip
	static bool build_chain(Task *p_task, bool p_force_simple_chain = true);

	static void update_chain(const Skeleton *p_sk, ChainItem *p_chain_item);

	static void solve_simple(Task *p_task, bool p_solve_magnet);
	static void solve_closed_loop(Task *p_task, bool p_solve_magnet);
	/// Special solvers that solve only chains with one end effector
	static void solve_simple_backwards(Chain &r_chain, bool p_solve_magnet);
	static void solve_simple_forwards(Chain &r_chain, bool p_solve_magnet);
	static void drag_point_tethered(
			ChainItem *chain,
			const Vector3 &maintain_distance_point,
			float bone_length,
			float max_drag_distance,
			float drag_stiffness,
			Vector3 &point_to_drag);

public:
	static Task *create_simple_task(Skeleton *p_sk, BoneId root_bone, BoneId tip_bone, const Transform &goal_transform);
	static void free_task(Task *p_task);
	// The goal of chain should be always in local space
	static void set_goal(Task *p_task, const Transform &p_goal);
	static void make_goal(Task *p_task, const Transform &p_inverse_transf, real_t blending_delta);
	static void solve(Task *p_task, real_t blending_delta, bool override_tip_basis, bool p_use_magnet, const Vector3 &p_magnet_position, int32_t mode = IK_SOLVER_NORMAL);
};

class SkeletonIK : public Node {
	GDCLASS(SkeletonIK, Node);

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
