#pragma once
#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iomanip>

// The displayed URDF is the single source of geometry and joint limits.
class PiperUrdfKinematics {
public:
    using Joints = std::array<double, 6>;
    struct Limit { double lower, upper, velocity; };
    PiperUrdfKinematics(const std::string& xml, const std::string& base,
                        const std::string& tip, const Eigen::Isometry3d& tool)
        : tool_(tool) {
        urdf::Model model;
        KDL::Tree tree;
        if (!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model, tree) ||
            !tree.getChain(base, tip, chain_) || chain_.getNrOfJoints() != 6) {
            throw std::invalid_argument("URDF must contain a six-joint base-to-tip chain");
        }
        unsigned index = 0;
        for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i) {
            const auto& joint = chain_.getSegment(i).getJoint();
            if (joint.getType() == KDL::Joint::None) continue;
            const auto expected = "joint" + std::to_string(index + 1);
            const auto spec = model.getJoint(joint.getName());
            if (joint.getName() != expected || !spec || !spec->limits ||
                spec->type != urdf::Joint::REVOLUTE || spec->mimic) {
                throw std::invalid_argument("Expected bounded revolute joint1 through joint6");
            }
            const auto& limit = *spec->limits;
            if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
                !std::isfinite(limit.velocity) || limit.lower >= limit.upper || limit.velocity <= 0)
                throw std::invalid_argument("Invalid URDF joint limit");
            limits_[index++] = {spec->limits->lower, spec->limits->upper, spec->limits->velocity};
        }
        auto jaw = model.getJoint("joint7");
        if (!jaw || !jaw->limits || jaw->limits->lower != 0 || !std::isfinite(jaw->limits->upper) || jaw->limits->upper <= 0) {
            throw std::invalid_argument("URDF must define joint7 jaw travel");
        }
        const auto other_jaw = model.getJoint("joint8");
        if (!other_jaw || !other_jaw->mimic || other_jaw->mimic->joint_name != "joint7" ||
            other_jaw->mimic->multiplier != -1.0 || other_jaw->mimic->offset != 0.0)
            throw std::invalid_argument("joint8 must mimic joint7 with multiplier -1 and offset 0");
        max_opening_ = 2.0 * jaw->limits->upper;
    }
    const std::array<Limit, 6>& limits() const { return limits_; }
    double maxOpening() const { return max_opening_; }
    bool valid(const Joints& q) const {
        for (size_t i = 0; i < 6; ++i)
            if (!std::isfinite(q[i]) || q[i] < limits_[i].lower || q[i] > limits_[i].upper) return false;
        return true;
    }
    Eigen::Isometry3d fk(const Joints& q) const {
        KDL::JntArray joints(6);
        for (size_t i = 0; i < 6; ++i) joints(i) = q[i];
        KDL::Frame frame;
        KDL::ChainFkSolverPos_recursive solver(chain_);
        if (solver.JntToCart(joints, frame) < 0) throw std::runtime_error("FK failed");
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        for (int r = 0; r < 3; ++r) {
            result.translation()(r) = frame.p(r);
            for (int c = 0; c < 3; ++c) result.linear()(r, c) = frame.M(r, c);
        }
        return result * tool_;
    }
    Eigen::Matrix<double, 6, 1> error(const Joints& q, const Eigen::Isometry3d& target) const {
        const auto current = fk(q);
        Eigen::Matrix<double, 6, 1> e;
        e.head<3>() = target.translation() - current.translation();
        const Eigen::AngleAxisd rotation(target.linear() * current.linear().transpose());
        e.tail<3>() = rotation.angle() * rotation.axis();
        return e;
    }
    Joints solve(Joints seed, const Eigen::Isometry3d& target, int iterations,
                 double position_tolerance, double orientation_tolerance, double damping) const {
        if (!valid(seed)) throw std::invalid_argument("Invalid IK seed");
        if (!target.matrix().allFinite() || iterations <= 0 ||
            !std::isfinite(position_tolerance) || position_tolerance <= 0 ||
            !std::isfinite(orientation_tolerance) || orientation_tolerance <= 0 ||
            !std::isfinite(damping) || damping <= 0)
            throw std::invalid_argument("Invalid IK configuration/target");
        // Always try the previous solution first. Retry only after a local solve fails.
        std::vector<Joints> seeds{seed};
        Joints middle;
        for (size_t i = 0; i < 6; ++i) middle[i] = (limits_[i].lower + limits_[i].upper) / 2.0;
        seeds.push_back(middle);
        for (double sign : {-1.0, 1.0}) {
            auto nearby = seed;
            for (size_t i = 0; i < 6; ++i)
                nearby[i] = std::clamp(seed[i] + sign * (i % 2 ? 0.25 : -0.25), limits_[i].lower, limits_[i].upper);
            seeds.push_back(nearby);
        }
        // Deterministic interior seeds avoid singular wrists and joint-limit traps.
        for (unsigned sample = 1; sample <= 16; ++sample) {
            Joints interior;
            for (size_t i = 0; i < 6; ++i) {
                const double fraction = 0.2 + 0.6 * std::fmod(sample * (0.61803398875 + i * 0.137), 1.0);
                interior[i] = limits_[i].lower + fraction * (limits_[i].upper - limits_[i].lower);
            }
            seeds.push_back(interior);
        }
        const double position_weight = orientation_tolerance / position_tolerance;
        // Minimize violations of the two acceptance bounds. Minimizing a raw
        // weighted sum alone can reject a feasible pose: improving orientation
        // beyond its tolerance can sacrifice the last fraction of a mm.
        const double position_margin = position_tolerance * 0.999;
        const double orientation_margin = orientation_tolerance * 0.999;
        auto violation = [](const Eigen::Vector3d& e, double margin) -> Eigen::Vector3d {
            const double norm = e.norm();
            return norm > margin ? (e * (1.0 - margin/norm)).eval() : Eigen::Vector3d::Zero();
        };
        auto score = [&](const Eigen::Matrix<double, 6, 1>& e, bool refinement) {
            if (!refinement) return position_weight*position_weight*e.head<3>().squaredNorm() + e.tail<3>().squaredNorm();
            return position_weight * position_weight * violation(e.head<3>(),position_margin).squaredNorm() +
                   violation(e.tail<3>(),orientation_margin).squaredNorm();
        };
        Eigen::Matrix<double, 6, 1> best_error = error(seed, target);
        auto diagnostic_score = [&](const Eigen::Matrix<double,6,1>& e) {
            return std::max(e.head<3>().norm()/position_tolerance, e.tail<3>().norm()/orientation_tolerance);
        };
        double best_score = diagnostic_score(best_error);
        for (auto q : seeds) {
          // Preserve the accurate ordinary solve; use tolerance-bound refinement
          // only if it stalls, before abandoning this initial guess.
          for (bool refinement : {false, true}) {
            double lambda = damping;
            for (int iter = 0; iter <= iterations; ++iter) {
                const auto e = error(q, target);
                const double current_score = score(e, refinement);
                if (diagnostic_score(e) < best_score) { best_score = diagnostic_score(e); best_error = e; }
                if (e.head<3>().norm() <= position_tolerance && e.tail<3>().norm() <= orientation_tolerance)
                    return q;
                if (iter == iterations) break;
                KDL::JntArray joints(6);
                for (size_t i = 0; i < 6; ++i) joints(i) = q[i];
                KDL::Jacobian jacobian(6);
                KDL::ChainJntToJacSolver solver(chain_);
                if (solver.JntToJac(joints, jacobian) < 0) throw std::runtime_error("Jacobian failed");
                const Eigen::Vector3d offset = fk(q).linear() * tool_.linear().transpose() * tool_.translation();
                Eigen::Matrix<double, 6, 6> j = jacobian.data;
                for (int i = 0; i < 6; ++i) j.block<3, 1>(0, i) += j.block<3, 1>(3, i).cross(offset);
                auto derivative = [](const Eigen::Vector3d& e, double margin) -> Eigen::Matrix3d {
                    const double norm = e.norm();
                    if (norm <= margin) return Eigen::Matrix3d::Zero();
                    const Eigen::Vector3d unit = e / norm;
                    return (1.0 - margin/norm) * Eigen::Matrix3d::Identity() +
                           (margin/norm) * unit * unit.transpose();
                };
                if (refinement) {
                j.topRows<3>() = (position_weight * derivative(e.head<3>(),position_margin) * j.topRows<3>()).eval();
                j.bottomRows<3>() = (derivative(e.tail<3>(),orientation_margin) * j.bottomRows<3>()).eval();
                }
                else { j.topRows<3>() *= position_weight; }
                Eigen::Matrix<double, 6, 1> weighted_error;
                weighted_error.head<3>() = position_weight * (refinement ? violation(e.head<3>(),position_margin) : e.head<3>().eval());
                weighted_error.tail<3>() = refinement ? violation(e.tail<3>(),orientation_margin) : e.tail<3>().eval();
                Eigen::Matrix<double, 6, 6> normal = j * j.transpose();
                normal.diagonal().array() += lambda * lambda;
                Eigen::Matrix<double, 6, 1> step = j.transpose() * normal.ldlt().solve(weighted_error);
                if (!step.allFinite()) break;
                const double largest = step.cwiseAbs().maxCoeff();
                if (largest > 0.15) step *= 0.15 / largest;
                bool improved = false;
                for (double scale = 1.0; scale >= 1.0/128; scale *= 0.5) {
                    auto candidate = q;
                    for (size_t i = 0; i < 6; ++i)
                        candidate[i] = std::clamp(q[i] + scale * step(i), limits_[i].lower, limits_[i].upper);
                    if (score(error(candidate, target), refinement) < current_score) {
                        q = candidate;
                        lambda = std::max(damping * 0.1, lambda * 0.7);
                        improved = true;
                        break;
                    }
                }
                if (!improved) {
                    lambda *= 3.0;
                    if (lambda > 10.0 || step.norm() < 1e-10) break;
                }
            }
        }
          }
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(2)
               << "No verified IK solution within URDF limits; best residual "
               << best_error.head<3>().norm() * 1000.0 << " mm, "
               << best_error.tail<3>().norm() * 180.0 / M_PI
               << " deg (unreachable target or local convergence failure)";
        throw std::runtime_error(detail.str());
    }

private:
    KDL::Chain chain_;
    Eigen::Isometry3d tool_;
    std::array<Limit, 6> limits_;
    double max_opening_;
};
