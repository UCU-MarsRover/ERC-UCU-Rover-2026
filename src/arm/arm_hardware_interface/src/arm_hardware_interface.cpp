#include "arm_hardware_interface/arm_hardware_interface.hpp"
#include "arm_hardware_interface/steadywin_protocol.hpp"
#include "arm_hardware_interface/damiao_wrist_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmCanSystem,
    hardware_interface::SystemInterface)

namespace arm_hardware_interface {

namespace sw = steadywin_protocol;
namespace dm = damiao_wrist_protocol;

static inline double smoothstep01(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

static double param_or(const std::unordered_map<std::string, std::string>& params,
                       const std::string& key, double fallback)
{
    auto it = params.find(key);
    if (it == params.end()) return fallback;
    try { return std::stod(it->second); } catch (...) { return fallback; }
}

// param_or() expects a NUMBER (std::stod) — "true"/"false" silently fails to
// parse and falls back, always. Use this instead for boolean URDF params.
static bool param_bool(const std::unordered_map<std::string, std::string>& params,
                       const std::string& key, bool fallback)
{
    auto it = params.find(key);
    if (it == params.end()) return fallback;
    const std::string& v = it->second;
    if (v == "true" || v == "1") return true;
    if (v == "false" || v == "0") return false;
    return fallback;
}

// =============================================================================
// Gravity compensation — minimal hand-rolled 3D math + fixed chain geometry.
//
// No new dependency (Eigen/KDL) on purpose: this arm has exactly 6 fixed
// revolute joints with axes that are always local X or local Z, so plain
// 3x3 rotation matrices are enough and keep this file self-contained.
// =============================================================================
namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major

constexpr double kPi = 3.14159265358979323846;
constexpr double kG  = 9.80665;      // m/s^2

Vec3 vec_add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
Vec3 vec_sub(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec3 vec_cross(const Vec3& a, const Vec3& b)
{
    return { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] };
}
double vec_dot(const Vec3& a, const Vec3& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

Mat3 mat_mul(const Mat3& A, const Mat3& B)
{
    Mat3 R{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 3; ++k)
                R[r*3+c] += A[r*3+k] * B[k*3+c];
    return R;
}

Vec3 mat_vec(const Mat3& A, const Vec3& v)
{
    return {
        A[0]*v[0] + A[1]*v[1] + A[2]*v[2],
        A[3]*v[0] + A[4]*v[1] + A[5]*v[2],
        A[6]*v[0] + A[7]*v[1] + A[8]*v[2],
    };
}

Mat3 mat_identity() { return {1,0,0, 0,1,0, 0,0,1}; }

// URDF <origin rpy="r p y"> convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Mat3 rot_rpy(double r, double p, double y)
{
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    const Mat3 Rx = {1,0,0,  0,cr,-sr,  0,sr,cr};
    const Mat3 Ry = {cp,0,sp,  0,1,0,  -sp,0,cp};
    const Mat3 Rz = {cy,-sy,0,  sy,cy,0,  0,0,1};
    return mat_mul(mat_mul(Rz, Ry), Rx);
}

// Rotation about a LOCAL basis axis — the only two axes used in this URDF.
Mat3 rot_local_x(double a) { const double c=std::cos(a), s=std::sin(a); return {1,0,0, 0,c,-s, 0,s,c}; }
Mat3 rot_local_z(double a) { const double c=std::cos(a), s=std::sin(a); return {c,-s,0, s,c,0, 0,0,1}; }

struct JointGeom {
    Vec3 xyz;         // <origin xyz="...">  — fixed, copied from arm_macro.xacro
    Vec3 rpy;         // <origin rpy="...">  — fixed, copied from arm_macro.xacro
    bool axis_is_z;   // true: <axis xyz="0 0 1"/>, false: <axis xyz="1 0 0"/>
};

// KEEP THIS IN SYNC WITH arm_macro.xacro <joint><origin>/<axis> — nothing
// enforces that automatically. Index order matches motor_ids_/joint_kp_/etc:
// 0 mount_base, 1 base_shoulder, 2 shoulder_forearm, 3 forearm_wrist_1,
// 4 wrist_1_wrist_2, 5 wrist_2_end_effector.
const std::array<JointGeom, NUM_JOINTS> kJointGeom = {{
    /*0 mount_base      */ { {0.06,      0.0,      0.021 }, {0.0,          0.0,      0.0        }, true  },
    /*1 base_shoulder    */ { {-0.040366,-0.000325, 0.103 }, {kPi/3.0,      0.0,     -kPi        }, false },
    /*2 shoulder_forearm */ { {0.0,       0.3,      0.0   }, {5.0*kPi/12.0, 0.0,      0.0        }, false },
    /*3 forearm_wrist_1  */ { {-0.0173,   0.3,      0.0   }, {0.0,          kPi/2.0,  0.0        }, true  },
    /*4 wrist_1_wrist_2  */ { {0.0323,    0.0,     -0.035 }, {-kPi/2.0,     0.0,      kPi/2.0    }, true  },
    /*5 wrist_2_ee       */ { {0.0323,    0.0,     -0.035 }, {-kPi/2.0,     kPi/2.0, -kPi/2.0    }, true  },
}};

struct LinkMass {
    double mass_kg;     // structure only, no motor — see kMotorMass
    Vec3   com_local;   // COM offset, in the CHILD link's own frame (meters)
};

struct PointMass {
    double mass_kg;
    Vec3   com_local;   // offset from the joint axis to the motor's COM,
                         // in the frame just BEFORE the joint's own rotation
                         // (housing/stator side)
};

// Links and motors are modeled separately: a motor sits on its own joint's
// axis and creates no moment about its own shaft, so lumping it into a
// link's COM would misplace that mass.
//
// com_local for links 1-4 is a MIDPOINT approximation (half the vector to
// the next joint's origin, from kJointGeom above) — uniform-mass-distribution
// assumption, not a measured COM. Good enough for sign/rough magnitude;
// refine with the balance-point method if more precision is needed.
//
// Index 0 (arm_base_link) is unused: mount_base is a vertical-axis yaw
// joint, so gravity torque there is exactly zero regardless of any mass.
const std::array<LinkMass, NUM_JOINTS> kLinkMass = {{
    /*0 arm_base_link         (unused — see note above) */ { 0.0,   {0.0,     0.0,  0.0    } },
    /*1 arm_shoulder_link                                */ { 0.730, {0.0,     0.15, 0.0    } },
    /*2 arm_forearm_link                                 */ { 0.630, {-0.00865,0.15, 0.0    } },
    /*3 arm_wrist_1_link                                 */ { 0.100, {0.01615, 0.0, -0.0175 } },
    /*4 arm_wrist_2_link                                 */ { 0.100, {0.01615, 0.0, -0.0175 } },
    // Index 5 is documentation only — compute_gravity_feedforward() always uses
    // ee_tool_mass_kg_/ee_tool_com_{x,y,z}_ instead. Edit the header initializer, not this line.
    /*5 arm_end_effector_link + everything mounted on it (DEFAULT, see above) */ { 0.400, {0.0, 0.0, 0.06} },
}};

// kMotorMass[i] — the motor that drives joint i, pinned at joint i's own axis
// (index 0 unused, same reason as arm_base_link). Masses from vendor
// datasheets (960 g Steadywin GIM8115-36 w/driver, 362 g Damiao DM-J4340-2EC).
const std::array<PointMass, NUM_JOINTS> kMotorMass = {{
    /*0 motor 20 mount_base      (unused — see note above) */ { 0.0,   {0.0, 0.0, 0.0} },
    /*1 motor 21 base_shoulder   */ { 0.960, {0.0, 0.0, 0.0} },
    /*2 motor 22 shoulder_forearm*/ { 0.960, {0.0, 0.0, 0.0} },
    /*3 motor 23 forearm_wrist_1 */ { 0.362, {0.0, 0.0, 0.0} },
    /*4 motor 24 wrist_1_wrist_2 */ { 0.362, {0.0, 0.0, 0.0} },
    /*5 motor 25 wrist_2_ee      */ { 0.362, {0.0, 0.0, 0.0} },
}};

} // namespace

// =============================================================================
// Lifecycle
// =============================================================================

#ifdef JAZZY_OR_LATER
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return init_from_info(params.hardware_info);
}
#else
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return init_from_info(info);
}
#endif

hardware_interface::CallbackReturn ArmCanSystem::init_from_info(
    const hardware_interface::HardwareInfo & info)
{
    if (info.joints.size() != NUM_JOINTS) {
        RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu", NUM_JOINTS, info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ---- Hardware-level parameters ----
    if (info.hardware_parameters.count("can_interface")) {
        can_interface_ = info.hardware_parameters.at("can_interface");
    } else {
        RCLCPP_WARN(logger_, "can_interface param missing in URDF, defaulting to can0");
    }
    gain_ramp_secs_        = param_or(info.hardware_parameters, "gain_ramp_secs", gain_ramp_secs_);
    max_cmd_speed_rad_s_   = param_or(info.hardware_parameters, "max_cmd_speed_rad_s", max_cmd_speed_rad_s_);
    feedback_timeout_secs_ = param_or(info.hardware_parameters, "feedback_timeout_secs", feedback_timeout_secs_);
    gravity_ff_enabled_    = param_bool(info.hardware_parameters, "gravity_ff_enabled", gravity_ff_enabled_);
    gravity_ff_max_nm_sw_  = param_or(info.hardware_parameters, "gravity_ff_max_nm_steadywin", gravity_ff_max_nm_sw_);
    gravity_ff_max_nm_dm_  = param_or(info.hardware_parameters, "gravity_ff_max_nm_damiao", gravity_ff_max_nm_dm_);
    ee_tool_mass_kg_       = param_or(info.hardware_parameters, "ee_tool_mass_kg", ee_tool_mass_kg_);
    ee_tool_com_x_         = param_or(info.hardware_parameters, "ee_tool_com_x", ee_tool_com_x_);
    ee_tool_com_y_         = param_or(info.hardware_parameters, "ee_tool_com_y", ee_tool_com_y_);
    ee_tool_com_z_         = param_or(info.hardware_parameters, "ee_tool_com_z", ee_tool_com_z_);

    // ---- Per-joint parameters: direction, offset, kp, kd, motor_id ----
    // These live in arm_macro.xacro so recalibration never requires recompiling.
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& j = info.joints[i];
        joint_names_[i] = j.name;

        const double dir = param_or(j.parameters, "direction", joint_directions_[i]);
        if (dir != 1.0 && dir != -1.0) {
            RCLCPP_ERROR(logger_, "Joint '%s': direction must be 1 or -1 (got %f)",
                         j.name.c_str(), dir);
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_directions_[i] = dir;
        joint_offsets_[i]    = param_or(j.parameters, "offset", joint_offsets_[i]);
        joint_kp_[i]         = param_or(j.parameters, "kp", joint_kp_[i]);
        joint_kd_[i]         = param_or(j.parameters, "kd", joint_kd_[i]);
        motor_ids_[i]        = static_cast<uint8_t>(
                                   param_or(j.parameters, "motor_id",
                                            static_cast<double>(motor_ids_[i])));

        // Damiao manual: kd must not be 0 while kp > 0 or the motor oscillates.
        if (i >= NUM_STEADYWIN && joint_kp_[i] > 0.0 && joint_kd_[i] <= 0.0) {
            RCLCPP_ERROR(logger_, "Joint '%s' (Damiao): kd must be > 0 when kp > 0", j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_,
            "Joint %zu '%s' -> motor %u  dir=%+.0f offset=%.4f rad  kp=%.1f kd=%.2f",
            i, j.name.c_str(), motor_ids_[i], joint_directions_[i],
            joint_offsets_[i], joint_kp_[i], joint_kd_[i]);
    }

    joint_position_command_.fill(0.0);
    joint_velocity_command_.fill(0.0);
    joint_position_state_.fill(0.0);
    joint_velocity_state_.fill(0.0);
    hw_position_states_.fill(0.0);
    hw_velocity_states_.fill(0.0);
    feedback_seen_.fill(false);
    dm_last_err_.fill(0x01);

    RCLCPP_INFO(logger_, "ArmCanSystem initialized (CAN: %s, ramp: %.1fs, cmd speed limit: %.2f rad/s)",
                can_interface_.c_str(), gain_ramp_secs_, max_cmd_speed_rad_s_);
    RCLCPP_INFO(logger_, "Gravity feed-forward: %s (max %.2f Nm Steadywin, %.2f Nm Damiao)",
                gravity_ff_enabled_ ? "ENABLED" : "disabled",
                gravity_ff_max_nm_sw_, gravity_ff_max_nm_dm_);
    RCLCPP_INFO(logger_, "End-effector payload: %.3f kg @ (%.4f, %.4f, %.4f) m",
                ee_tool_mass_kg_, ee_tool_com_x_, ee_tool_com_y_, ee_tool_com_z_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::array<float, NUM_JOINTS> ArmCanSystem::compute_gravity_feedforward() const
{
    std::array<float, NUM_JOINTS> tau{};
    tau.fill(0.0f);

    const Vec3 g_vec = {0.0, 0.0, -kG};   // base frame: +z assumed vertical/up

    // Forward-kinematics chain, base (arm_mount_link) -> end effector, using
    // the CURRENT measured pose (joint_position_state_, URDF/world convention).
    Mat3 R = mat_identity();
    Vec3 p = {0.0, 0.0, 0.0};

    std::array<Vec3, NUM_JOINTS> joint_pos{};   // p_i: joint i's origin, base frame
    std::array<Vec3, NUM_JOINTS> joint_axis{};  // z_i: joint i's axis,   base frame
    std::array<Vec3, NUM_JOINTS> link_com{};    // COM of joint i's child link (structure only), base frame
    std::array<Vec3, NUM_JOINTS> motor_com{};   // COM of the motor driving joint i, base frame

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& jg = kJointGeom[i];
        const Mat3 R_origin = rot_rpy(jg.rpy[0], jg.rpy[1], jg.rpy[2]);
        const Mat3 R_pre    = mat_mul(R, R_origin);            // orientation just before this joint's own rotation
        const Vec3 p_joint  = vec_add(p, mat_vec(R, jg.xyz));  // this joint's origin, base frame

        const Vec3 axis_local = jg.axis_is_z ? Vec3{0.0,0.0,1.0} : Vec3{1.0,0.0,0.0};
        const Vec3 z_i = mat_vec(R_pre, axis_local);

        const double theta  = joint_position_state_[i];        // measured pose, URDF frame
        const Mat3 R_theta  = jg.axis_is_z ? rot_local_z(theta) : rot_local_x(theta);
        const Mat3 R_link   = mat_mul(R_pre, R_theta);          // child link's orientation, base frame

        // Index NUM_JOINTS-1 (arm_end_effector_link) uses the runtime
        // end-effector payload override instead of kLinkMass's compile-time
        // entry — see ee_tool_mass_kg_ / ee_tool_com_{x,y,z}_ in the header.
        const Vec3 link_com_local = (i == NUM_JOINTS - 1)
            ? Vec3{ee_tool_com_x_, ee_tool_com_y_, ee_tool_com_z_}
            : kLinkMass[i].com_local;

        joint_pos[i]  = p_joint;
        joint_axis[i] = z_i;
        link_com[i]   = vec_add(p_joint, mat_vec(R_link, link_com_local));
        // Motor i is pinned to its own joint's position; com_local is a small
        // offset expressed on the housing/stator side (R_pre, not R_link) —
        // see the assumption noted on PointMass above.
        motor_com[i]  = vec_add(p_joint, mat_vec(R_pre, kMotorMass[i].com_local));

        R = R_link;   // advance chain for next joint
        p = p_joint;
    }

    // tau_i = -g_vec . sum_{k>=i} [ m_link_k * (z_i x (link_com_k - p_i))
    //                              + m_motor_k * (z_i x (motor_com_k - p_i)) ]
    // Summing motors over k>=i (not k>i) is deliberate and needs no special
    // case: for k==i the motor sits ~at p_i itself, so its own contribution
    // to ITS OWN joint's torque comes out ~0 automatically from the cross
    // product — it only matters for joints upstream of it (k>i terms below).
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        Vec3 sum{0.0, 0.0, 0.0};
        for (std::size_t k = i; k < NUM_JOINTS; ++k) {
            const double m_link  = (k == NUM_JOINTS - 1) ? ee_tool_mass_kg_ : kLinkMass[k].mass_kg;
            if (m_link > 0.0) {
                const Vec3 r       = vec_sub(link_com[k], joint_pos[i]);
                const Vec3 contrib = vec_cross(joint_axis[i], r);
                sum = { sum[0] + m_link*contrib[0], sum[1] + m_link*contrib[1], sum[2] + m_link*contrib[2] };
            }
            const double m_motor = kMotorMass[k].mass_kg;
            if (m_motor > 0.0) {
                const Vec3 r       = vec_sub(motor_com[k], joint_pos[i]);
                const Vec3 contrib = vec_cross(joint_axis[i], r);
                sum = { sum[0] + m_motor*contrib[0], sum[1] + m_motor*contrib[1], sum[2] + m_motor*contrib[2] };
            }
        }
        const double tau_urdf  = -vec_dot(g_vec, sum);
        double tau_motor       = tau_urdf * joint_directions_[i];   // same sign convention as velocity feedforward
        const double clamp_nm = gravity_ff_max_nm(i);
        tau_motor = std::clamp(tau_motor, -clamp_nm, clamp_nm);
        tau[i] = static_cast<float>(tau_motor);
    }

    return tau;
}

std::vector<hardware_interface::StateInterface> ArmCanSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_state_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmCanSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type ArmCanSystem::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>& stop_interfaces)
{
    // JointGroupPositionController never claims velocity, so when JTC
    // releases it the buffer would otherwise keep JTC's last value —
    // write() reads it straight through as Servo VFF.
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const std::string iface = joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;
        if (std::find(stop_interfaces.begin(), stop_interfaces.end(), iface) != stop_interfaces.end()) {
            joint_velocity_command_[i] = 0.0;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn ArmCanSystem::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(logger_, "Configuring ArmCanSystem...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_activate(const rclcpp_lifecycle::State&)
{
    if (!open_can_socket()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    feedback_seen_.fill(false);
    rx_running_.store(true);
    rx_thread_ = std::thread(&ArmCanSystem::rx_thread_fn, this);

    // 1) Enable: Steadywin gets its 0xF0 limits config (does NOT apply torque —
    //    it only enters MIT mode on the first 0x4xx frame). Damiao gets 0xFC
    //    (enabled but torqueless until its first MIT command).
    send_enable_frames();

    // 2) ACTIVELY poll every motor for its true position: they only reply when
    //    spoken to, and waiting passively leaves the state at 0, which makes
    //    the first command jump the arm. Abort rather than guess.
    if (!wait_for_all_feedback(feedback_timeout_secs_)) {
        RCLCPP_FATAL(logger_,
            "Not all motors reported their position — REFUSING to enable torque "
            "with an unknown arm pose. Check CAN wiring / IDs / power.");
        safe_stop();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3) Sync commands and states to the measured pose (URDF frame),
    //    so the first MIT frame holds the arm exactly where it is.
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        joint_position_state_  = hw_position_states_;
        joint_velocity_state_  = hw_velocity_states_;
    }
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        joint_position_command_[i] = joint_position_state_[i];
        joint_velocity_command_[i] = 0.0;
        last_sent_command_[i]      = joint_position_state_[i];
        RCLCPP_INFO(logger_, "  '%s' start pose: %.4f rad (motor frame %.4f)",
                    joint_names_[i].c_str(), joint_position_state_[i],
                    urdf_to_motor(i, joint_position_state_[i]));
    }
    have_last_sent_ = true;
    ramp_started_   = false;   // ramp clock starts at the first write()
    motors_enabled_.store(true);

    RCLCPP_INFO(logger_,
        "ArmCanSystem activated. Stiffness will ramp 0 -> 100%% over %.1f s. "
        "Do not command motion until the ramp completes.", gain_ramp_secs_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_WARN(logger_,
        "Deactivating WITHOUT disabling: Steadywin motors (base/shoulder/"
        "elbow) hold their last commanded position indefinitely — they need "
        "no further CAN traffic to stay stiff. Damiao wrist motors (forearm/"
        "wrist/end-effector) WILL go limp shortly after this process stops "
        "sending frames (their own comm-loss watchdog disables them) — "
        "support the wrist/end-effector before deactivating.");
    stop_holding();
    RCLCPP_INFO(logger_, "ArmCanSystem deactivated (motors left enabled, holding last position).");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
    stop_holding();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_error(const rclcpp_lifecycle::State&)
{
    // A real fault means we no longer trust our own state (bad feedback,
    // CAN issues, etc.) — unlike a clean deactivate, holding position here
    // could mean holding a WRONG position with real torque. Fail-safe: cut
    // power instead.
    RCLCPP_ERROR(logger_, "on_error: disabling all motors (fail-safe).");
    safe_stop();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void ArmCanSystem::safe_stop()
{
    motors_enabled_.store(false);
    if (can_fd_ >= 0) {
        send_disable_frames();
    }
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_can_socket();
}

void ArmCanSystem::stop_holding()
{
    // Deliberately does NOT call send_disable_frames(): Steadywin keeps
    // executing its last MIT command (position/kp/kd) with no further
    // traffic required, so it stays holding after this process exits.
    // Damiao motors will drop out on their own comm-loss watchdog shortly
    // after we stop streaming — nothing we send here changes that.
    motors_enabled_.store(false);
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_can_socket();
}

// =============================================================================
// read / write
// =============================================================================

hardware_interface::return_type ArmCanSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    joint_position_state_ = hw_position_states_;
    joint_velocity_state_ = hw_velocity_states_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmCanSystem::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
    if (!motors_enabled_.load()) return hardware_interface::return_type::OK;

    // ---- Stiffness ramp: 0 -> 1 over gain_ramp_secs_ from the first write ----
    if (!ramp_started_) {
        ramp_start_   = std::chrono::steady_clock::now();
        ramp_started_ = true;
    }
    double ramp = 1.0;
    if (gain_ramp_secs_ > 0.05) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ramp_start_).count();
        ramp = smoothstep01(elapsed / gain_ramp_secs_);
    }

    // ---- Command rate limiter: never step the target faster than
    //      max_cmd_speed_rad_s_, no matter what the controller asks for.
    //      This is the last line of defense against "jump to wrong pose". ----
    double dt = period.seconds();
    if (!(dt > 0.0) || dt > 0.5) dt = 0.01;
    const double max_step = max_cmd_speed_rad_s_ * dt;

    // Per-joint deadband drops the small wrist terms of a Cartesian step and
    // leaves only shoulder/elbow — TCP orientation then walks on every WASD
    // move. Reject noise only when *all* joints are below the threshold;
    // otherwise keep the full coordinated delta (still rate-limited).
    constexpr double kPosDeadbandRad = 0.0005;

    std::array<double, NUM_JOINTS> cmd{};
    std::array<double, NUM_JOINTS> cmd_delta{};
    double max_abs_delta = 0.0;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        double target = joint_position_command_[i];
        if (!std::isfinite(target)) {
            target = last_sent_command_[i];
        }
        cmd_delta[i] = target - last_sent_command_[i];
        max_abs_delta = std::max(max_abs_delta, std::abs(cmd_delta[i]));
    }
    const bool hold = max_abs_delta < kPosDeadbandRad;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        double delta = hold ? 0.0 : std::clamp(cmd_delta[i], -max_step, max_step);
        cmd_delta[i] = delta;
        cmd[i] = last_sent_command_[i] + delta;
        last_sent_command_[i] = cmd[i];
    }

    const std::array<float, NUM_JOINTS> gravity_tff =
        gravity_ff_enabled_ ? compute_gravity_feedforward() : std::array<float, NUM_JOINTS>{};

    {
        // One combined snapshot of all 6 joints together, instead of each
        // motor logging independently as its own CAN frame happens to
        // arrive (staggered, arbitrary order) — much easier to read live.
        // DEBUG, not INFO: this was spamming the default log output at 1 Hz;
        // enable with --ros-args --log-level ArmCanSystem:=debug when needed.
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        std::array<float, NUM_JOINTS> torque_snapshot;
        {
            std::lock_guard<std::mutex> fb_lock(feedback_mutex_);
            torque_snapshot = last_torque_nm_;
        }
        RCLCPP_DEBUG_THROTTLE(logger_, steady_clock, 1000,
            "Torque (Nm): %s=%.2f  %s=%.2f  %s=%.2f  %s=%.2f  %s=%.2f  %s=%.2f",
            joint_names_[0].c_str(), torque_snapshot[0],
            joint_names_[1].c_str(), torque_snapshot[1],
            joint_names_[2].c_str(), torque_snapshot[2],
            joint_names_[3].c_str(), torque_snapshot[3],
            joint_names_[4].c_str(), torque_snapshot[4],
            joint_names_[5].c_str(), torque_snapshot[5]);
    }

    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const float target_pos = static_cast<float>(urdf_to_motor(i, cmd[i]));
        double vff = joint_velocity_command_[i];
        if (!std::isfinite(vff)) vff = 0.0;
        // Forward position controller does not write velocity. Without VFF the
        // MIT loop chases a held Pos_des with Vel_des=0 between Servo ticks →
        // each joint "runs a bit and stops" (teleop buzz). Use the slew-limited
        // position step as feedforward — consistent with Pos_des.
        constexpr double kControllerVffEps = 1e-4;  // rad/s
        if (std::abs(vff) < kControllerVffEps) {
            vff = cmd_delta[i] / dt;
        }
        // Pos_des above is slew-limited to max_cmd_speed_rad_s_; keep Vel_des
        // on the same ceiling so Kp/Kd do not fight.
        vff = std::clamp(vff, -max_cmd_speed_rad_s_, max_cmd_speed_rad_s_);
        const float target_vel = static_cast<float>(vff * joint_directions_[i]);
        const float kp = static_cast<float>(joint_kp_[i] * ramp);
        const float kd = static_cast<float>(joint_kd_[i] * ramp);
        // Ramped in with the gains — no sudden torque at activation.
        const float tff = gravity_tff[i] * static_cast<float>(ramp);

        can_msgs::msg::Frame f = (i < NUM_STEADYWIN)
            ? sw::build_mit_command_frame(motor_ids_[i], target_pos, target_vel, kp, kd, tff)
            : dm::build_mit_command_frame(motor_ids_[i], target_pos, target_vel, kp, kd, tff);
        send_can_frame(f.id, f.data, f.dlc);
    }

    return hardware_interface::return_type::OK;
}

// =============================================================================
// SocketCAN
// =============================================================================

bool ArmCanSystem::open_can_socket()
{
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to open CAN socket: %s", std::strerror(errno));
        return false;
    }

    // Receive timeout so rx_thread_fn can notice rx_running_ == false; a
    // blocking read would hang the deactivate path on join().
    struct timeval tv{};
    tv.tv_sec = 0; tv.tv_usec = 100000;   // 100 ms
    setsockopt(can_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "ioctl failed for %s: %s", can_interface_.c_str(), std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "CAN bind failed: %s", std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "SocketCAN interface %s opened.", can_interface_.c_str());
    return true;
}

void ArmCanSystem::close_can_socket()
{
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
    }
}

bool ArmCanSystem::send_can_frame(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t dlc)
{
    if (can_fd_ < 0) return false;
    struct can_frame frame{};
    frame.can_id  = id;            // all IDs here fit in 11-bit standard frames
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data.data(), dlc);
    return ::write(can_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

// =============================================================================
// Motor sequences
// =============================================================================

void ArmCanSystem::send_enable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
        auto f = sw::build_config_limits_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto f = dm::build_enable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void ArmCanSystem::send_disable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    // Damiao: zero-gain MIT frame first to avoid a torque discontinuity
    // (mirrors dm_disable() in the proven Python tools), then 0xFD.
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto probe = dm::build_zero_gain_probe_frame(motor_ids_[i]);
        send_can_frame(probe.id, probe.data, probe.dlc);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto f = dm::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
    for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
        auto f = sw::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
}

bool ArmCanSystem::wait_for_all_feedback(double timeout_sec)
{
    RCLCPP_INFO(logger_, "Polling all motors for their true positions (timeout %.1f s)...",
                timeout_sec);

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(can_tx_mutex_);
            for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
                // 0xF1: reads state WITHOUT entering MIT mode — zero torque.
                auto f = sw::build_read_state_frame(motor_ids_[i]);
                send_can_frame(f.id, f.data, f.dlc);
            }
            for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
                // kp=0/kd=0/tff=0 MIT frame: zero torque, elicits feedback.
                auto f = dm::build_zero_gain_probe_frame(motor_ids_[i]);
                send_can_frame(f.id, f.data, f.dlc);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        bool all = true;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) all = all && feedback_seen_[i];
        }
        if (all) {
            RCLCPP_INFO(logger_, "All %zu motors reported.", NUM_JOINTS);
            return true;
        }
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!feedback_seen_[i]) {
            RCLCPP_ERROR(logger_, "  motor %u ('%s') never replied",
                         motor_ids_[i], joint_names_[i].c_str());
        }
    }
    return false;
}

// =============================================================================
// RX thread
// =============================================================================

void ArmCanSystem::rx_thread_fn()
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    struct can_frame frame{};
    while (rx_running_.load()) {
        const ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (!rx_running_.load()) break;
            continue;
        }
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) continue;
        if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) continue;

        const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;

        // ---------- Steadywin: replies arrive on StdID == Dev_addr ----------
        bool is_sw = false;
        std::size_t sw_idx = 0;
        for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
            if (raw_id == motor_ids_[i]) { is_sw = true; sw_idx = i; break; }
        }
        if (is_sw) {
            sw::Feedback fb;
            // parse_feedback rejects 0xF0/0xB1/etc. replies whose bytes [1..2]
            // are not a position — decoded as one, the 0xF0 config echo reads
            // as ~92 rad and glitches the joint state.
            if (sw::parse_feedback(frame.data, frame.can_dlc, fb)) {
                if (fb.fault) {
                    RCLCPP_WARN_THROTTLE(logger_, steady_clock, 2000,
                        "Steadywin motor %u reports FAULT", motor_ids_[sw_idx]);
                }
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                hw_position_states_[sw_idx] = motor_to_urdf(sw_idx, fb.pos_rad);
                hw_velocity_states_[sw_idx] = fb.vel_rps * joint_directions_[sw_idx];
                feedback_seen_[sw_idx] = true;
                last_torque_nm_[sw_idx] = fb.torque_nm;
            }
            continue;
        }

        // ---------- Damiao: replies arrive on that motor's own Master ID,
        //            0x400 | CAN-ID (these wrists were re-flashed away from
        //            the factory-default shared Master ID 0) ----------------
        std::size_t dm_idx = NUM_JOINTS;
        for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
            if (raw_id == dm::master_id_for(motor_ids_[i])) { dm_idx = i; break; }
        }
        if (dm_idx == NUM_JOINTS) continue;

        dm::Feedback fb;
        if (!dm::parse_feedback(frame.data, frame.can_dlc, fb)) continue;

        // The Master ID already told us who sent this; data[0]'s low nibble is
        // only a sanity check that the motor's own CAN-ID matches what we
        // expect for that slot (catches a mis-flashed / duplicated ID).
        if ((motor_ids_[dm_idx] & 0x0F) != fb.motor_id_nibble) {
            RCLCPP_WARN_THROTTLE(logger_, steady_clock, 5000,
                "Damiao reply on 0x%03X carries CAN-ID nibble %u, expected %u "
                "(motor %u) — check the motor's flashed ID/Master ID.",
                raw_id, fb.motor_id_nibble, motor_ids_[dm_idx] & 0x0F, motor_ids_[dm_idx]);
            continue;
        }

        if (fb.err >= 0x8) {
            RCLCPP_ERROR_THROTTLE(logger_, steady_clock, 2000,
                "Damiao motor %u error: %s (T_mos=%u°C T_rotor=%u°C)",
                motor_ids_[dm_idx], dm::err_to_string(fb.err), fb.t_mos_c, fb.t_rotor_c);
        } else if (fb.err == 0x0 && motors_enabled_.load() && dm_last_err_[dm_idx] == 0x1) {
            RCLCPP_WARN_THROTTLE(logger_, steady_clock, 2000,
                "Damiao motor %u dropped to DISABLED (comm-loss watchdog?)",
                motor_ids_[dm_idx]);
        }

        std::lock_guard<std::mutex> lock(feedback_mutex_);
        dm_last_err_[dm_idx] = fb.err;
        hw_position_states_[dm_idx] = motor_to_urdf(dm_idx, fb.pos_rad);
        hw_velocity_states_[dm_idx] = fb.vel_rps * joint_directions_[dm_idx];
        feedback_seen_[dm_idx] = true;
        last_torque_nm_[dm_idx] = fb.torque_nm;
    }
}

} // namespace arm_hardware_interface