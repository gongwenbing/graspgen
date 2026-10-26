#include "piper_urdf_kinematics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <iterator>
#include <unordered_map>

class PiperIKNode : public rclcpp::Node {
    using Joints = PiperUrdfKinematics::Joints;
    using Clock = std::chrono::steady_clock;
public:
    PiperIKNode() : Node("piper_ik_node") {
        hardware_ = declare_parameter("hardware_mode", false);
        base_ = declare_parameter<std::string>("base_frame", "base_link");
        tip_ = declare_parameter<std::string>("tip_link", "link6");
        const auto default_model = ament_index_cpp::get_package_share_directory("piper_description") +
                                   "/urdf/piper_description.urdf";
        const auto path = declare_parameter<std::string>("model_path", default_model);
        auto xml = declare_parameter<std::string>("robot_description", "");
        if (xml.empty()) {
            std::ifstream file(path);
            if (!file) throw std::invalid_argument("Cannot read model_path: " + path);
            xml.assign(std::istreambuf_iterator<char>(file), {});
        }
        const auto xyz = declare_parameter<std::vector<double>>("tool_xyz", {0.0, 0.0, 0.0});
        const auto rpy = declare_parameter<std::vector<double>>("tool_rpy", {0.0, 0.0, 0.0});
        if (xyz.size() != 3 || rpy.size() != 3) throw std::invalid_argument("tool_xyz/tool_rpy need three values");
        Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(xyz[i]) || !std::isfinite(rpy[i])) throw std::invalid_argument("Invalid tool transform");
        tool.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
        tool.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        model_ = std::make_unique<PiperUrdfKinematics>(xml, base_, tip_, tool);
        // Firmware Cartesian coordinates have not been calibrated against this URDF/TCP.
        if (declare_parameter("use_analytic_ik", false))
            throw std::invalid_argument("use_analytic_ik /pos_cmd is disabled: firmware TCP is unverified; use joint commands");
        iterations_ = declare_parameter("max_iterations", 200);
        damping_ = declare_parameter("damping_factor", 0.03);
        pos_tol_ = declare_parameter("position_tolerance", 0.001);
        rot_tol_ = declare_parameter("orientation_tolerance", 0.01);
        const double rate = declare_parameter("publish_rate", 30.0);
        feedback_timeout_ = declare_parameter("feedback_timeout", 0.5);
        command_speed_ = declare_parameter("max_joint_velocity", 0.2);
        jaw_speed_ = declare_parameter("max_gripper_velocity", 0.01);
        for (double value : {damping_, pos_tol_, rot_tol_, rate, feedback_timeout_, command_speed_, jaw_speed_})
            if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Rates, timeouts and tolerances must be positive/finite");
        if (iterations_ <= 0) throw std::invalid_argument("max_iterations must be positive");
        const auto visual_topic = declare_parameter<std::string>("joint_state_topic", "/ik/joint_states");
        const auto command_topic = declare_parameter<std::string>("joint_command_topic", "/ik/joint_commands");
        const auto feedback_topic = declare_parameter<std::string>("sub_real_joint_position_topic", "/joint_states_single");
        visual_pub_ = create_publisher<sensor_msgs::msg::JointState>(visual_topic, 10);
        command_pub_ = create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
        if (visual_pub_->get_topic_name() == std::string(command_pub_->get_topic_name()) ||
            get_node_topics_interface()->resolve_topic_name(feedback_topic) == std::string(command_pub_->get_topic_name()) ||
            get_node_topics_interface()->resolve_topic_name(feedback_topic) == std::string(visual_pub_->get_topic_name()))
            throw std::invalid_argument("Visual, command and feedback topics must be separate");
        current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ik/current_pose", 10);
        status_pub_ = create_publisher<std_msgs::msg::Bool>("/ik_status", 10);
        feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(feedback_topic, rclcpp::SensorDataQoS(),
            std::bind(&PiperIKNode::feedback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            declare_parameter<std::string>("target_pose_topic", "/target_pose"), 10,
            std::bind(&PiperIKNode::target, this, std::placeholders::_1));
        gripper_sub_ = create_subscription<std_msgs::msg::Float64>(
            declare_parameter<std::string>("gripper_cmd_topic", "/gripper_cmd_topic"), 10,
            std::bind(&PiperIKNode::gripper, this, std::placeholders::_1));
        enable_srv_ = create_service<std_srvs::srv::SetBool>("~/enable_commands",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                if (!req->data) { stop(); res->success = true; res->message = "Command output disabled"; return; }
                if (!hardware_ || !fresh()) {
                    res->success = false;
                    res->message = "Requires hardware_mode and fresh, valid full joint/gripper feedback";
                    return;
                }
                commanded_ = desired_ = measured_;
                opening_ = desired_opening_ = measured_opening_;
                armed_ = true;
                have_target_ = false;
                last_tick_ = Clock::now();
                res->success = true;
                res->message = "Ready; output starts only after a new valid target or gripper command";
            });
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&PiperIKNode::tick, this));
        RCLCPP_INFO(get_logger(), "URDF model %s -> %s + tool transform; %s; hardware output starts DISABLED",
                    base_.c_str(), tip_.c_str(), hardware_ ? "hardware feedback mode" : "visualization mode");
    }
private:
    bool fresh() const {
        return have_feedback_ && std::chrono::duration<double>(Clock::now() - feedback_at_).count() <= feedback_timeout_;
    }
    void status(bool ok) { std_msgs::msg::Bool msg; msg.data = ok; status_pub_->publish(msg); }
    void stop() { armed_ = false; have_target_ = false; }
    void feedback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        try {
            if (hardware_) {
                const rclcpp::Time stamp(msg->header.stamp);
                const double age = (now() - stamp).seconds();
                if (stamp.nanoseconds() == 0 || age > feedback_timeout_ || age < -0.1)
                    throw std::invalid_argument("Feedback timestamp stale, missing or in the future");
            }
            if (msg->name.size() != msg->position.size()) throw std::invalid_argument("Feedback name/position length mismatch");
            std::unordered_map<std::string, double> values;
            for (size_t i = 0; i < msg->name.size(); ++i)
                if (!std::isfinite(msg->position[i]) || !values.emplace(msg->name[i], msg->position[i]).second)
                    throw std::invalid_argument("Nonfinite/duplicate feedback joint");
            Joints q;
            for (size_t i = 0; i < 6; ++i) q[i] = values.at("joint" + std::to_string(i + 1));
            if (!model_->valid(q)) throw std::invalid_argument("Feedback outside URDF limits");
            // Driver reports full opening as 'gripper'; URDF joint7 is one jaw's travel.
            const double width = values.count("gripper") ? values.at("gripper") : 2.0 * values.at("joint7");
            if (width < 0 || width > model_->maxOpening()) throw std::invalid_argument("Gripper feedback outside URDF limits");
            if (hardware_ && have_feedback_ && !fresh()) stop();
            measured_ = q;
            measured_opening_ = width;
            have_feedback_ = true;
            feedback_at_ = Clock::now();
            if (hardware_ && !armed_) {
                commanded_ = desired_ = q;
                opening_ = desired_opening_ = width;
            }
        } catch (const std::exception& e) {
            have_feedback_ = false;
            if (hardware_) stop();
            RCLCPP_WARN(get_logger(), "Rejected joint feedback: %s", e.what());
        }
    }
    void target(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        try {
            if (hardware_ && (!armed_ || !fresh())) { stop(); throw std::runtime_error("Command output disabled or feedback stale"); }
            if (msg->header.frame_id != base_) throw std::invalid_argument("Target must use configured base_frame");
            const auto& p = msg->pose.position;
            const auto& r = msg->pose.orientation;
            Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-12)
                throw std::invalid_argument("Invalid target pose");
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            pose.linear() = q.normalized().toRotationMatrix();
            const auto seed = have_target_ ? desired_ : (hardware_ ? measured_ : commanded_);
            // RViz can repeat the same failed pose at its feedback rate. Do not
            // spend every callback solving it again while the seed is unchanged.
            if (have_failed_pose_ &&
                std::chrono::duration<double>(Clock::now() - failed_at_).count() < 0.5 &&
                pose.matrix().isApprox(failed_pose_.matrix(), 1e-8) &&
                std::equal(seed.begin(), seed.end(), failed_seed_.begin(),
                           [](double a, double b) { return std::abs(a-b) < 0.001; })) {
                status(false);
                return;
            }
            Joints result;
            try {
                result = model_->solve(seed, pose, iterations_, pos_tol_, rot_tol_, damping_);
                have_failed_pose_ = false;
            } catch (const std::exception&) {
                have_failed_pose_ = true;
                failed_pose_ = pose;
                failed_seed_ = seed;
                failed_at_ = Clock::now();
                throw;
            }
            // Verify the actual bounded result before making it the active goal.
            const auto error = model_->error(result, pose);
            if (!model_->valid(result) || error.head<3>().norm() > pos_tol_ || error.tail<3>().norm() > rot_tol_)
                throw std::runtime_error("IK result failed final verification");
            if (hardware_ && !fresh()) { stop(); throw std::runtime_error("Feedback expired during solve"); }
            desired_ = result;
            target_pose_ = pose;
            have_pose_ = true;
            have_target_ = true;
            status(true);  // Solvable, not an assertion that hardware has arrived.
        } catch (const std::exception& e) {
            status(false);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Target rejected: %s", e.what());
        }
    }
    void gripper(const std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (!std::isfinite(msg->data) || msg->data < 0 || msg->data > model_->maxOpening()) {
            RCLCPP_WARN(get_logger(), "Rejected gripper opening outside URDF limits"); return;
        }
        if (hardware_ && (!armed_ || !fresh())) { stop(); return; }
        desired_opening_ = msg->data;
        have_target_ = true;
    }
    sensor_msgs::msg::JointState jointState(const Joints& q, double width, bool driver) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        for (size_t i = 0; i < 6; ++i) {
            msg.name.push_back("joint" + std::to_string(i + 1)); msg.position.push_back(q[i]);
        }
        msg.name.push_back(driver ? "gripper" : "joint7");
        msg.position.push_back(driver ? width : width / 2.0);
        return msg;
    }
    void transform(const Eigen::Isometry3d& pose, const std::string& child) {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = now(); msg.header.frame_id = base_; msg.child_frame_id = child;
        msg.transform.translation.x = pose.translation().x();
        msg.transform.translation.y = pose.translation().y();
        msg.transform.translation.z = pose.translation().z();
        Eigen::Quaterniond q(pose.linear());
        msg.transform.rotation.x = q.x(); msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z(); msg.transform.rotation.w = q.w();
        broadcaster_->sendTransform(msg);
    }
    static double approach(double value, double goal, double step) {
        return value + std::clamp(goal - value, -step, step);
    }
    void tick() {
        const auto time = Clock::now();
        const double dt = std::clamp(std::chrono::duration<double>(time - last_tick_).count(), 0.0, 0.1);
        last_tick_ = time;
        if (hardware_ && !fresh()) { if (armed_) { stop(); status(false); } return; }
        if (!hardware_ || (armed_ && have_target_)) {
            for (size_t i = 0; i < 6; ++i)
                commanded_[i] = approach(commanded_[i], desired_[i],
                    dt * std::min(command_speed_, model_->limits()[i].velocity));
            opening_ = approach(opening_, desired_opening_, dt * jaw_speed_);
        }
        visual_pub_->publish(jointState(hardware_ ? measured_ : commanded_, hardware_ ? measured_opening_ : opening_, false));
        if (hardware_ && armed_ && have_target_)
            command_pub_->publish(jointState(commanded_, opening_, true));
        const auto current = model_->fk(hardware_ ? measured_ : commanded_);
        transform(current, "ik_current");
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now(); pose.header.frame_id = base_;
        pose.pose.position.x = current.translation().x();
        pose.pose.position.y = current.translation().y();
        pose.pose.position.z = current.translation().z();
        Eigen::Quaterniond rotation(current.linear());
        pose.pose.orientation.x = rotation.x(); pose.pose.orientation.y = rotation.y();
        pose.pose.orientation.z = rotation.z(); pose.pose.orientation.w = rotation.w();
        current_pose_pub_->publish(pose);
        transform(model_->fk(commanded_), "ik_solution");
        if (have_pose_) transform(target_pose_, "ik_target");
    }
    std::unique_ptr<PiperUrdfKinematics> model_;
    bool hardware_, armed_ = false, have_feedback_ = false, have_target_ = false, have_pose_ = false;
    std::string base_, tip_;
    int iterations_;
    double damping_, pos_tol_, rot_tol_, feedback_timeout_, command_speed_, jaw_speed_;
    Joints measured_{}, commanded_{}, desired_{};
    bool have_failed_pose_ = false;
    Joints failed_seed_{};
    Eigen::Isometry3d failed_pose_ = Eigen::Isometry3d::Identity();
    Clock::time_point failed_at_{};
    double measured_opening_ = 0, opening_ = 0, desired_opening_ = 0;
    Clock::time_point feedback_at_{}, last_tick_ = Clock::now();
    Eigen::Isometry3d target_pose_ = Eigen::Isometry3d::Identity();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr visual_pub_, command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<PiperIKNode>()); }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("piper_ik_node"), "%s", e.what());
        rclcpp::shutdown(); return 1;
    }
    rclcpp::shutdown(); return 0;
}
