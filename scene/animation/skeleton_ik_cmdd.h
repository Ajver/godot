/*************************************************************************/
/*  skeleton_ik_cmdd.h                                                   */
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

// Based upon https://github.com/EGjoni/Everything-WIll-Be-IK

#ifndef SKELETON_IK_CMDD_H
#define SKELETON_IK_CMDD_H

#ifndef _3D_DISABLED

#include "core/math/transform.h"
#include "scene/3d/skeleton.h"
#include "scene/animation/skeleton_ik.h"
#include "scene/animation/skeleton_ik_object.h"
#include <limits>

struct Ray {
    Vector3 position = Vector3();
    Vector3 normal = Vector3();

    /**
     * sets the values of the given vector to where the
     * tip of this Ray would be if the ray were inverted
     * @param p_vec
     * @return the vector that was passed in after modification (for chaining)
     */
    Vector3 set_to_inverted_tip(Vector3 p_vec);

    Ray() {
    }

    Ray(Vector3 p_position, Vector3 p_normal) {
        position = p_position;
        normal = p_normal;
    }

    /**
     *  adds the specified length to the ray in both directions.
     */
    void elongate(real_t p_amount);

    /* Find where this ray intersects a sphere
        * @param SGVec_3d the center of the sphere to test against.
        * @param radius radius of the sphere
        * @param S1 reference to variable in which the first intersection will be placed
        * @param S2 reference to variable in which the second intersection will be placed
        * @return number of intersections found;
        */
    int intersects_sphere(Vector3 sphereCenter, double radius, Vector3 S1, Vector3 S2);

    /* Find where this ray intersects a sphere
        * @param radius radius of the sphere
        * @param S1 reference to variable in which the first intersection will be placed
        * @param S2 reference to variable in which the second intersection will be placed
        * @return number of intersections found;
        */
    int intersects_sphere(Vector3 rp1, Vector3 rp2, double radius, Vector3 &S1, Vector3 &S2);
};

class IKQuat : public Quat {
public:
    Vector<IKQuat> get_swing_twist(Vector3 p_axis);

    void clamp_to_quadrance_angle(real_t p_cos_half_angle);

    void clamp_to_angle(real_t p_angle);

    inline IKQuat(real_t p_x, real_t p_y, real_t p_z, real_t p_w) :
            Quat(p_x,
                 p_y,
                 p_z,
                 p_w) {
    }

    IKQuat(Quat p_quat) {
        x = p_quat.x;
        y = p_quat.y;
        z = p_quat.z;
        w = p_quat.w;
    }

    IKQuat() {
    }

    ~IKQuat() {
    }
};

class IKConstraintKusudama;

class IKLimitCone : public Object {
    /**
     * a triangle where the [1] is th tangentCircleNext_n, and [0] and [2]
     * are the points at which the tangent circle intersects this limitCone and the
     * next limitCone
     */
    Vector<Vector3> first_triangle_next;
    Vector<Vector3> second_triangle_next;

    //radius stored as  cosine to save on the acos call necessary for angleBetween.
    real_t radius_cosine = 0.f;
    real_t radius = 0.f;
    Ref<IKConstraintKusudama> parent_kusudama;
    Vector3 tangent_circle_center_next_1;
    Vector3 tangent_circle_center_next_2;
    real_t tangent_circle_radius_next = 0.0f;
    real_t tangent_circle_radius_next_cos = 0.0f;
    Vector3 tangent_circle_center_previous_1;
    Vector3 tangent_circle_center_previous_2;
    real_t tangent_circle_radius_previous = 0.0f;
    real_t tangent_circle_radius_previous_cos = 0.0f;

    void compute_triangles(IKLimitCone p_next);

public:
    Vector3 control_point;
    Vector3 radial_point;

    Vector3 get_on_path_sequence(IKLimitCone p_next, Vector3 p_input) const;

    Vector3 closest_cone(IKLimitCone p_next, Vector3 p_input) const;

    Vector3 get_closest_path_point(IKLimitCone p_next, Vector3 p_input) const;

    real_t get_radius() const;

    real_t get_radius_cosine();

    Vector3 get_control_point();

    Vector3 get_orthogonal(Vector3 p_vec);

    IKLimitCone() {}

    void set_control_point(Vector3 p_control_point);

    void update_tangent_handles(IKLimitCone p_next);

    IKLimitCone(Vector3 p_location, real_t p_rad, Ref<IKConstraintKusudama> p_attached_to);

    void set_radius(real_t p_radius);
};

class IKAxes : public Transform {

    static const int LEFT = -1;
    static const int RIGHT = 1;

    int chirality = RIGHT;

public:
    void operator=(const IKAxes &p_axes);

    int get_global_chirality();

    int get_local_chirality();

    IKAxes() {}

    IKAxes(const Basis &p_basis, const Vector3 &p_origin) {
        set_basis(p_basis);
        set_origin(p_origin);
    }

    IKAxes(const Transform &p_xform) {
        IKAxes(p_xform.basis, p_xform.origin);
    }
};

struct IKConstraint : public Resource {
GDCLASS(IKConstraint, Resource);

public:
    virtual void snap_to_limits() = 0;

    virtual void disable() = 0;

    virtual void enable() = 0;

    virtual bool is_enabled() const = 0;

    //returns true if the ray from the constraint origin to the globalPoint is within the constraint's limits
    //false otherwise.
    virtual bool is_in_limits_(const Vector3 globalPoint) const = 0;

    virtual IKAxes get_limiting_axes() const = 0;

    /**
     * @return a measure of the rotational freedom afforded by this constraint.
     * with 0 meaning no rotational freedom (the bone is essentially stationary in relation to its parent)
     * and 1 meaning full rotational freedom (the bone is completely unconstrained).
     *
     * This should be computed as ratio between orientations a bone can be in and orientations
     * a bone cannot be in as defined by its representation as a point on the surface of a hypersphere.
     */
    virtual float get_rotational_freedom() const = 0;
};

struct SkeletonIKConstraint : public Object {
GDCLASS(SkeletonIKConstraint, Object);

public:
    StringName chain_item_name;
    Ref<IKConstraintKusudama> constraint;

    SkeletonIKConstraint() {
        constraint.instance();
    }
};

class SkeletonIKConstraints : public Resource {
GDCLASS(SkeletonIKConstraints, Resource);

public:
    void set_constraint_count(int32_t p_value);

    int32_t get_constraint_count() const;

    void set_chain_item(int32_t p_index, StringName p_value);

    StringName get_chain_item(int32_t p_index) const;

    void set_bone_names(PoolStringArray p_bones);

    PoolStringArray get_bone_names() const;

    void set_constraint(int32_t p_index, Ref<IKConstraintKusudama> p_constraint);

    Ref<IKConstraintKusudama> get_constraint(int32_t p_index) const;

    SkeletonIKConstraints() {}

    ~SkeletonIKConstraints() {}

protected:
    PoolStringArray bone_names;
    Vector<SkeletonIKConstraint> constraints;

    bool _set(const StringName &p_name, const Variant &p_value);

    bool _get(const StringName &p_name, Variant &r_ret) const;

    void _get_property_list(List<PropertyInfo> *p_list) const;

    static void _bind_methods();
};

class QCP : public Reference {

    /**
     * Implementation of the Quaternionff-Based Characteristic Polynomial algorithm
     * for RMSD and Superposition calculations.
     * <p>
     * Usage:
     * <p>
     * The input consists of 2 SGVec_3f arrays of equal length. The input coordinates
     * are not changed.
     *
     * <pre>
     *    SGVec_3f[] x = ...
     *    SGVec_3f[] y = ...
     *    SuperPositionQCP qcp = new SuperPositionQCP();
     *    qcp.set(x, y);
     * </pre>
     * <p>
     * or with weighting factors [0 - 1]]
     *
     * <pre>
     *    float[] weights = ...
     *    qcp.set(x, y, weights);
     * </pre>
     * <p>
     * For maximum efficiency, create a SuperPositionQCP object once and reuse it.
     * <p>
     * A. Calculate rmsd only
     *
     * <pre>
     * float rmsd = qcp.getRmsd();
     * </pre>
     * <p>
     * B. Calculate a 4x4 transformation (rotation and translation) matrix
     *
     * <pre>
     * Matrix4f rottrans = qcp.getTransformationMatrix();
     * </pre>
     * <p>
     * C. Get transformated points (y superposed onto the reference x)
     *
     * <pre>
     * SGVec_3f[] ySuperposed = qcp.getTransformedCoordinates();
     * </pre>
     * <p>
     * Citations:
     * <p>
     * Liu P, Agrafiotis DK, & Theobald DL (2011) Reply to comment on: "Fast
     * determination of the optimal rotation matrix for macromolecular
     * superpositions." Journal of Computational Chemistry 32(1):185-186.
     * [http://dx.doi.org/10.1002/jcc.21606]
     * <p>
     * Liu P, Agrafiotis DK, & Theobald DL (2010) "Fast determination of the optimal
     * rotation matrix for macromolecular superpositions." Journal of Computational
     * Chemistry 31(7):1561-1563. [http://dx.doi.org/10.1002/jcc.21439]
     * <p>
     * Douglas L Theobald (2005) "Rapid calculation of RMSDs using a
     * quaternion-based characteristic polynomial." Acta Crystallogr A
     * 61(4):478-480. [http://dx.doi.org/10.1107/S0108767305015266 ]
     * <p>
     * This is an adoption of the original C code QCProt 1.4 (2012, October 10) to
     * Java. The original C source code is available from
     * http://theobald.brandeis.edu/qcp/ and was developed by
     * <p>
     * Douglas L. Theobald Department of Biochemistry MS 009 Brandeis University 415
     * South St Waltham, MA 02453 USA
     * <p>
     * dtheobald@brandeis.edu
     * <p>
     * Pu Liu Johnson & Johnson Pharmaceutical Research and Development, L.L.C. 665
     * Stockton Drive Exton, PA 19341 USA
     * <p>
     * pliu24@its.jnj.com
     * <p>
     *
     * @author Douglas L. Theobald (original C code)
     * @author Pu Liu (original C code)
     * @author Peter Rose (adopted to Java)
     * @author Aleix Lafita (adopted to Java)
     * @author Eron Gjoni (adopted to EWB IK)
     */

    real_t evec_prec = (real_t) 1E-6;
    real_t eval_prec = (real_t) 1E-11;
    int max_iterations = 5;

    PoolVector3Array moved;

    PoolRealArray weight;
    real_t wsum = 0;

    Vector3 targetCenter;
    Vector3 movedCenter;

    real_t e0 = 0;
    real_t rmsd = 0;
    real_t Sxy, Sxz, Syx, Syz, Szx, Szy = 0;
    real_t SxxpSyy, Szz, mxEigenV, SyzmSzy, SxzmSzx, SxymSyx = 0;
    real_t SxxmSyy, SxypSyx, SxzpSzx = 0;
    real_t Syy, Sxx, SyzpSzy = 0;
    bool rmsd_calculated = false;
    bool transformation_calculated = false;

public:
    PoolVector3Array target;

    /**
     * Constructor with option to set the precision values.
     *
     * @param p_eval_prec
     *            required eigenvector precision
     * @param p_eval_prec
     *            required eigenvalue precision
     */
    QCP(real_t p_evec_prec, real_t p_eval_prec) {
        evec_prec = p_evec_prec;
        eval_prec = p_eval_prec;
    }

    /**
     * Sets the maximum number of iterations QCP should run before giving up.
     * In most situations QCP converges in 3 or 4 iterations, but in some situations convergence
     * occurs slowly or not at all, and so an exit condition is used. The default value is 20.
     * Increase it for more stability.
     * @param p_max
     */
    void set_max_iterations(int p_max);

    /**
     * Sets the two input coordinate arrays. These input arrays must be of equal
     * length. Input coordinates are not modified.
     *
     * @param p_target
     *            3f points of reference coordinate set
     * @param p_moved
     *            3f points of coordinate set for superposition
     */
private:
    void set(PoolVector3Array p_target, PoolVector3Array p_moved);

    /**
     * Sets the two input coordinate arrays and weight array. All input arrays
     * must be of equal length. Input coordinates are not modified.
     *
     * @param p_target
     *            3f points of reference coordinate set
     * @param p_moved
     *            3f points of coordinate set for superposition
     * @param p_weight
     *            a weight in the inclusive range [0,1] for each point
     */
public:
    void set(PoolVector3Array p_moved, PoolVector3Array p_target, PoolRealArray p_weight, bool p_translate);

    /**
     * Return the RMSD of the superposition of input coordinate set y onto x.
     * Note, this is the fasted way to calculate an RMSD without actually
     * superposing the two sets. The calculation is performed "lazy", meaning
     * calculations are only performed if necessary.
     *
     * @return root mean square deviation for superposition of y onto x
     */
public:
    float get_rmsd();

    /**
     * Weighted superposition.
     *
     * @param p_target
     * @param p_moved
     * @param p_weight
     *            array of weigths for each equivalent point position
     * @return
     */
    IKQuat
    weighted_superpose(PoolVector3Array p_moved, PoolVector3Array p_target, PoolRealArray p_weight, bool p_translate);

private:
    IKQuat get_rotation();

    /**
     * Calculates the RMSD value for superposition of y onto x. This requires
     * the coordinates to be precentered.
     *
     * @param p_x
     *            3f points of reference coordinate set
     * @param p_y
     *            3f points of coordinate set for superposition
     */
    void calc_rmsd(PoolVector3Array p_x, PoolVector3Array p_y);

    /**
     * Calculates the inner product between two coordinate sets x and y
     * (optionally weighted, if weights set through
     * {@link #set(SGVec_3f[], SGVec_3f[], float[])}). It also calculates an
     * upper bound of the most positive root of the key matrix.
     * http://theobald.brandeis.edu/qcp/qcprot.c
     *
     * @param p_coords1
     * @param p_coords2
     * @return
     */
    void inner_product(PoolVector3Array p_coords1, PoolVector3Array p_coords2);

    int calc_rmsd(real_t p_len);

    IKQuat calc_rotation();

public:
    float get_rmsd(PoolVector3Array p_fixed, PoolVector3Array p_moved);

    void translate(Vector3 p_trans, PoolVector3Array p_x);

    Vector3 get_weighted_center(PoolVector3Array p_to_center, PoolRealArray p_weight, Vector3 p_center);

    Vector3 get_translation();
};

// CMDD is Cyclic Mean Deviation Descent
class CMDDInverseKinematic : public Object {
GDCLASS(CMDDInverseKinematic, Object);

public:
    struct EndEffector {
        BoneId effector_bone;
        Transform goal_transform;
    };
    struct Chain;

    struct ChainItem {
        Chain *parent_armature;

        Vector<ChainItem> children;
        ChainItem *parent_item;

        // Bone info
        BoneId bone;
        PhysicalBone *pb;
        bool springy;
        real_t cos_half_dampen = 0.0f;
        PoolRealArray cos_half_returnful_dampened;
        PoolRealArray half_returnful_dampened;
        bool ik_orientation_lock = false;
        real_t stiffness_scalar = 0.0f;

        real_t length;
        // Positions relative to the root bone
        Transform initial_transform;
        // Position from this bone to the child
        Vector3 current_pos;
        // Direction from this bone to the child
        Vector3 current_ori;

        Ref<IKConstraintKusudama> constraint;

        ChainItem() :
                parent_item(NULL),
                bone(-1),
                pb(NULL),
                springy(false),
                length(0) {}

        ChainItem *find_child(const BoneId p_bone_id);

        ChainItem *add_child(const BoneId p_bone_id);

        void set_stiffness(real_t p_stiffness);

        real_t get_stiffness() const;

        void update_cos_dampening();

        void
        set_axes_to_returned(IKAxes p_global, IKAxes p_to_set, IKAxes p_limiting_axes, real_t p_cos_half_angle_dampen,
                             real_t p_angle_dampen);

        void set_axes_to_be_snapped(IKAxes p_to_set, IKAxes p_limiting_axes, real_t p_cos_half_angle_dampen);

        void populate_return_dampening_iteration_array(Ref<IKConstraintKusudama> k);
    };

    struct ChainTarget {
        ChainItem *chain_item;
        const EndEffector *end_effector;

        ChainTarget() :
                chain_item(NULL),
                end_effector(NULL) {}

        ChainTarget(ChainItem *p_chain_item, const EndEffector *p_end_effector) :
                chain_item(p_chain_item),
                end_effector(p_end_effector) {}

        ChainTarget(const ChainTarget &p_other_ct) :
                chain_item(p_other_ct.chain_item),
                end_effector(p_other_ct.end_effector) {}

        ChainTarget(ChainItem *p_chain_item, const EndEffector *p_end_effector, bool p_enabled) {
            enabled = p_enabled;
            set_target_priorities(xPriority, yPriority, zPriority);
        }

    protected:
        bool enabled;
        ChainTarget *parent_target;
        Vector<ChainTarget *> childPins;
        float pin_weight = 1;
        uint8_t modeCode = 7;
        int subTargetCount = 4;
        real_t xPriority = 1.0f, yPriority = 1.0f, zPriority = 1.0f;
        real_t depthFalloff = 0.0f;

    public:
        static const short XDir = 1, YDir = 2, ZDir = 4;

        bool is_enabled() const;

        void toggle();

        void enable();

        void disable();

        /**
         * Pins can be ultimate targets, or intermediary targets.
         * By default, each pin is treated as an ultimate target, meaning
         * any bones which are ancestors to that pin's end-effector
         * are not aware of any pins wich are target of bones descending from that end effector.
         *
         * Changing this value makes ancestor bones aware, and also determines how much less
         * they care with each level down.
         *
         * Presuming all descendants of this pin have a falloff of 1, then:
         * A pin falloff of 0 on this pin means only this pin is reported to ancestors.
         * A pin falloff of 1 on this pin means ancestors care about all descendant pins equally (after accounting for their pinWeight),
         * regardless of how many levels down they are.
         * A pin falloff of 0.5 means each descendant pin is cared about half as much as its ancestor.
         *
         * With each level, the pin falloff of a descendant is taken account for each level.
         *  Meaning, if this pin has a falloff of 1, and its descendent has a falloff of 0.5
         *  then this pin will be reported with full weight,
         *  it descendant will be reported with full weight,
         *  the descendant of that pin will be reported with half weight.
         *  the desecendant of that one's descendant will be reported with quarter weight.
         *
         * @param depth
         */
        void set_depth_falloff(float depth);

        real_t get_depth_falloff() const;

        /**
         * Sets  the priority of the orientation bases which effectors reaching for this target will and won't align with.
         * If all are set to 0, then the target is treated as a simple position target.
         * It's usually better to set at least on of these three values to 0, as giving a nonzero value to all three is most often redundant.
         *
         *  This values this function sets are only considered by the orientation aware solver.
         *
         * @param position
         * @param xPriority set to a positive value (recommended between 0 and 1) if you want the bone's x basis to point in the same direction as this target's x basis (by this library's convention the x basis corresponds to a limb's twist)
         * @param yPriority set to a positive value (recommended between 0 and 1)  if you want the bone's y basis to point in the same direction as this target's y basis (by this library's convention the y basis corresponds to a limb's direction)
         * @param zPriority set to a positive value (recommended between 0 and 1)  if you want the bone's z basis to point in the same direction as this target's z basis (by this library's convention the z basis corresponds to a limb's twist)
         */
        void set_target_priorities(float p_x_priority, float p_y_priority, float p_z_priority);

        /**
         * @return the number of bases an effector to this target will attempt to align on.
         */
        int get_subtarget_count();

        uint8_t get_mode_code() const;

        /**
         * @return the priority of this pin's x axis;
         */
        real_t get_x_priority() const;

        /**
         * @return the priority of this pin's y axis;
         */
        real_t get_y_priority() const;

        /**
         * @return the priority of this pin's z axis;
         */
        real_t get_z_priority() const;

        IKAxes get_axes() const;

        /**
         * translates and rotates the pin to match the position
         * and orientation of the input Axes. The orientation
         * is only relevant for orientation aware solvers.
         * @param inAxes
         */
        void align_to_axes(IKAxes inAxes);

        /**
         * translates the pin to the location specified in global coordinates
         * @param location
         */
        void translate_global(Vector3 location);

        /**
         * translates the pin to the location specified in local coordinates
         * (relative to any other Axes objects the pin may be parented to)
         * @param location
         */
        void translate(Vector3 location);

        /**
         * @return the pin locationin global coordinates
         */
        Vector3 get_location();

        ChainItem *for_bone();

        /**
         * called when this pin is being removed entirely from the Armature. (as opposed to just being disabled)
         */
        void removal_notification();

        void set_parent_pin(ChainTarget *parent);

        void remove_child_pin(ChainTarget *child);

        void add_child_pin(ChainTarget *newChild);

        ChainTarget *get_parent_pin();

        bool is_ancestor_of(ChainTarget *potentialDescendent);

        real_t get_pin_weight();
    };

    struct Chain {
    private:
    public:
        ChainItem chain_root;
        ChainItem *middle_chain_item;
        Vector<ChainTarget> targets;
        Vector3 magnet_position;
        PoolVector3Array localized_target_headings;
        PoolVector3Array localized_effector_headings;
        PoolRealArray weights;
        Ref<SkeletonIKConstraints> constraints;
        real_t dampening = Math::deg2rad(5.0f);
        Map<BoneId, ChainItem *> bone_segment_map;
        int ik_iterations = 15;

        int get_default_iterations() const;
    };

public:
    struct Task : public RID_Data {
        RID self;
        Skeleton *skeleton;

        Chain chain;

        // Settings
        real_t min_distance;
        int iterations;
        int max_iterations;
        // dampening dampening angle in radians.
        // Set this to -1 if you want to use the armature's default.
        real_t dampening;
        // stabilizing_passes number of stabilization passes to run.
        // Set this to -1 if you want to use the armature's default.
        int stabilizing_passes;

        // Bone data
        BoneId root_bone;
        Vector<EndEffector> end_effectors;

        Transform goal_global_transform;

        Task() :
                skeleton(NULL),
                min_distance(0.01),
                iterations(4),
                max_iterations(10),
                dampening(-1),
                stabilizing_passes(-1),
                root_bone(-1) {}
    };

private:
    /**
     * The default maximum number of radians a bone is allowed to rotate per solver iteration.
     * The lower this value, the more natural the pose results. However, this will  the number of iterations
     * the solver requires to converge.
     *
     * !!THIS IS AN EXPENSIVE OPERATION.
     * This updates the entire armature's cache of precomputed quadrance angles.
     * The cache makes things faster in general, but if you need to dynamically change the dampening during a call to IKSolver, use
     * the IKSolver(bone, dampening, iterations, stabilizationPasses) function, which clamps rotations on the fly.
     * @param damp
     */
    static void set_default_dampening(Chain &r_chain, real_t p_damp);

    static void update_armature_segments(Chain &r_chain);

    static void update_optimal_rotation_to_target_descendants(
            ChainItem *p_chain_item,
            real_t p_dampening,
            bool p_is_translate,
            PoolVector3Array p_localized_tip_headings,
            PoolVector3Array p_localized_target_headings,
            PoolRealArray p_weights,
            QCP p_qcp_orientation_aligner,
            int p_iteration,
            real_t p_total_iterations);

    static void recursively_update_bone_segment_map_from(Chain &r_chain, ChainItem *p_start_from);

    static void QCPSolver(
            Chain p_chain,
            float p_dampening,
            bool p_inverse_weighting,
            int p_stabilization_passes,
            int p_iteration,
            float p_total_iterations);

    static bool build_chain(Task *p_task, bool p_force_simple_chain = true);

    static void update_chain(const Skeleton *p_sk, ChainItem *p_chain_item);

    static void solve_simple(Task *p_task, bool p_solve_magnet);

public:
    static const int32_t x_axis = 0;
    static const int32_t y_axis = 1;
    static const int32_t z_axis = 2;

    /**
     *
     * @param for_bone
     * @param dampening
     * @param translate set to true if you wish to allow translation in addition to rotation of the bone (should only be used for unpinned root bones)
     * @param stabilization_passes If you know that your armature isn't likely to succumb to instability in unsolvable configurations, leave this value set to 0.
     * If you value stability in extreme situations more than computational speed, then increase this value. A value of 1 will be completely stable, and just as fast
     * as a value of 0, however, it might result in small levels of robotic looking jerk. The higher the value, the less jerk there will be (but at potentially significant computation cost).
     */
    static void update_optimal_rotation_to_target_descendants(
            Chain &r_chain,
            ChainItem *p_for_bone,
            float p_dampening,
            bool p_translate,
            int p_stabilization_passes,
            int p_iteration,
            int p_total_iterations);

    static real_t
    get_manual_msd(PoolVector3Array &r_localized_effector_headings, PoolVector3Array &r_localized_target_headings,
                   const PoolRealArray &p_weights);

    static void update_target_headings(Chain &r_chain, PoolVector3Array &r_localized_target_headings,
                                       PoolRealArray p_weights, Transform p_bone_xform);

    static void update_effector_headings(Chain &r_chain, PoolVector3Array &r_localized_effector_headings,
                                         Transform p_bone_xform);

    static Task *
    create_simple_task(Skeleton *p_sk, BoneId root_bone, BoneId effector_bone, const Transform &goal_transform,
                       real_t p_dampening = -1, int p_stabilizing_passes = -1,
                       Ref<SkeletonIKConstraints> p_constraints = NULL);

    static void free_task(Task *p_task);

    // The goal of chain should be always in local space
    static void set_goal(Task *p_task, const Transform &p_goal);

    static void make_goal(Task *p_task, const Transform &p_inverse_transf, real_t blending_delta);

    static void solve(Task *p_task, real_t blending_delta, bool override_effector_basis, bool p_use_magnet,
                      const Vector3 &p_magnet_position);
};


class IKConstraintKusudama : public IKConstraint {
GDCLASS(IKConstraintKusudama, IKConstraint);

private:
    /**
     * An array containing all of the Kusudama's limitCones. The kusudama is built up
     * with the expectation that any limitCone in the array is connected to the cone at the previous element in the array,
     * and the cone at the next element in the array.
     */
    Vector<IKLimitCone> limit_cones;
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being equivalent to the
     * limitingAxes Z axis.
     */
    float minAxialAngle = Math_PI;

    bool orientation_constrained = false;
    bool axial_constrained = false;
    IKAxes limiting_axes;
    CMDDInverseKinematic::ChainItem *attached_to = NULL;
    Ray bone_ray;
    Ray constrained_ray;

protected:
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being equivalent to the
     * limitingAxes Z axis.
     */
    real_t min_axial_angle = Math_PI;
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being equivalent to the
     * minAxialAngle
     */
    real_t range = Math_PI * 3.0f;
    real_t pain;
    real_t rotational_freedom = 1.0f;

    real_t get_rotational_freedom() const;

    /**
     * @return a measure of the rotational freedom afforded by this constraint.
     * with 0 meaning no rotational freedom (the bone is essentially stationary in relation to its parent)
     * and 1 meaning full rotational freedom (the bone is completely unconstrained).
     *
     * This should be computed as ratio between orientations a bone can be in and orientations
     * a bone cannot be in as defined by its representation as a point on the surface of a hypersphere.
     */
    virtual void update_rotational_freedom();

public:
    IKConstraintKusudama() {
    }

    IKConstraintKusudama(CMDDInverseKinematic::ChainItem *p_for_bone) {
        attached_to = p_for_bone;
//     limiting_axes = p_for_bone.getMajorRotationAxes();
//     attached_to->parent_armature->addConstraint(this);
        enable();
    }

    virtual void snap_to_limits() {}

    virtual void disable() {}

    virtual void enable() {}

    virtual bool is_enabled() const { return false; }

    //returns true if the ray from the constraint origin to the globalPoint is within the constraint's limits
    //false otherwise.
    virtual bool is_in_limits_(const Vector3 p_global_point) const;

    virtual IKAxes get_limiting_axes() const;

    virtual void set_limiting_axes(const IKAxes &p_limiting_axes);

    virtual real_t get_pain();

    /**
     * A value between (ideally between 0 and 1) dictating
     * how much the bone to which this kusudama belongs
     * prefers to be away from the edges of the kusudama
     * if it can. This is useful for avoiding unnatural poses,
     * as the kusudama will push bones back into their more
     * "comfortable" regions. Leave this value at its default of
     * 0 unless you empirical observations show you need it.
     * Setting this value to anything higher than 0.4 is probably overkill
     * in most situations.
     *
     * @param p_amount
     */
    virtual void set_pain(real_t p_amount);

    virtual real_t to_tau(real_t p_angle);

    virtual Vector3 point_on_path_sequence(IKAxes p_global_xform, Vector3 p_in_point, IKAxes p_limiting_axes);

    virtual real_t signed_angle_difference(real_t p_min_angle, real_t p_base);

    virtual real_t angle_to_twist_center(IKAxes p_global_xform, IKAxes p_to_set, IKAxes p_limiting_axes);

    virtual void set_axes_to_returnfulled(IKAxes p_global_xform, IKAxes p_to_set, IKAxes p_limiting_axes,
                                          real_t p_cos_half_angle_dampen, real_t p_angle_dampen);

    /**
     *
     * @param p_to_set
     * @param p_limiting_axes
     * @return radians of twist required to snap bone into twist limits (0 if bone is already in twist limits)
     */
    virtual real_t snap_to_twist_limits(IKAxes p_to_set, IKAxes p_limiting_axes);

    /**
     * Presumes the input axes are the bone's localAxes, and rotates
     * them to satisfy the snap limits.
     *
     * @param p_to_set
     */
    virtual void set_axes_to_orientation_snap(IKAxes p_to_set, IKAxes p_limiting_axes, float p_cos_half_angle_dampen);;

    virtual void set_axes_to_snapped(IKAxes p_to_set, IKAxes p_limiting_axes, float p_cos_half_angle_dampen);;

    virtual void constraint_update_notification();

    /**
     * This function should be called after you've set all of the Limiting Cones
     * for this Kusudama. It will orient the axes relative to which constrained rotations are computed
     * so as to minimize the potential for undesirable twist rotations due to antipodal singularities.
     *
     * In general, auto-optimization attempts to point the y-component of the constraint
     * axes in the direction that places it within an oreintation allowed by the constraint,
     * and roughly as far as possible from any orientations not allowed by the constraint.
     */
    virtual void optimize_limiting_axes();

    virtual void update_tangent_radii();

    virtual IKLimitCone create_limit_cone_for_index(int p_insert_at, Vector3 p_new_point, float p_radius);

    /**
     * Adds a LimitCone to the Kusudama. LimitCones are reach cones which can be arranged sequentially. The Kusudama will infer
     * a smooth path leading from one LimitCone to the next.
     *
     * Using a single LimitCone is functionally equivalent to a classic reachCone constraint.
     *
     * @param p_insert_at the intended index for this LimitCone in the sequence of LimitCones from which the Kusudama will infer a path. @see IK.AbstractKusudama.limitCones limitCones array.
     * @param p_new_point where on the Kusudama to add the LimitCone (in Kusudama's local coordinate frame defined by its bone's majorRotationAxes))
     * @param p_radius the radius of the limitCone
     */
    void add_limit_cone_at_index(int p_insert_at, Vector3 p_new_point, float p_radius);

    /**
     * Kusudama constraints decompose the bone orientation into a swing component, and a twist component.
     * The "Swing" component is the final direction of the bone. The "Twist" component represents how much
     * the bone is rotated about its own final direction. Where limit cones allow you to constrain the "Swing"
     * component, this method lets you constrain the "twist" component.
     *
     * @param p_min_angle some angle in radians about the major rotation frame's y-axis to serve as the first angle within the range that the bone is allowed to twist.
     * @param p_in_range some angle in radians added to the minAngle. if the bone's local Z goes p_max_angle radians beyond the p_min_angle, it is considered past the limit.
     * This value is always interpreted as being in the positive direction. For example, if this value is -PI/2, the entire range from p_min_angle to p_min_angle + 3PI/4 is
     * considered valid.
     */
    void set_axial_limits(float p_min_angle, float p_in_range);
};

class SkeletonIKCMDD : public SkeletonIKObject {
GDCLASS(SkeletonIKCMDD, SkeletonIKObject);

    StringName root_bone;
    StringName effector_bone;
    real_t interpolation;
    Transform target;
    NodePath target_node_path_override;
    bool override_effector_basis;
    bool use_magnet;
    Vector3 magnet_position;
    real_t min_distance;
    int max_iterations;
    Skeleton *skeleton;
    Spatial *target_node_override;
    CMDDInverseKinematic::Task *task;
    Ref<SkeletonIKConstraints> constraints;

    Transform _get_target_transform();

    void reload_chain();

    void reload_goal();

    void _solve_chain();

protected:
    virtual void
    _validate_property(PropertyInfo &property) const;

    static void _bind_methods();

    virtual void _notification(int p_what);

public:
    SkeletonIKCMDD();

    virtual ~SkeletonIKCMDD();

    void set_root_bone(const StringName &p_root_bone);

    StringName get_root_bone() const;

    // Tip is an effector bone
    void set_tip_bone(const StringName &p_effector_bone);

    // Tip is an effector bone
    StringName get_tip_bone() const;

    void set_interpolation(real_t p_interpolation);

    real_t get_interpolation() const;

    void set_target_transform(const Transform &p_target);

    const Transform &get_target_transform() const;

    void set_target_node(const NodePath &p_node);

    NodePath get_target_node();

    // Tip is an effector bone
    void set_override_tip_basis(bool p_override);

    // Tip is an effector bone
    bool is_override_tip_basis() const;

    void set_use_magnet(bool p_use);

    bool is_using_magnet() const;

    void set_magnet_position(const Vector3 &p_local_position);

    const Vector3 &get_magnet_position() const;

    void set_min_distance(real_t p_min_distance);

    real_t get_min_distance() const { return min_distance; }

    void set_max_iterations(int p_iterations);

    int get_max_iterations() const { return max_iterations; }

    virtual void set_constraints(const Ref<SkeletonIKConstraints> p_constraints);

    virtual Ref<SkeletonIKConstraints> get_constraints() const;

    Skeleton *get_parent_skeleton() const;

    bool is_running();

    void start(bool p_one_time = false);

    void stop();
};

#endif // _3D_DISABLED

#endif // SKELETON_IK_CMDD_H
