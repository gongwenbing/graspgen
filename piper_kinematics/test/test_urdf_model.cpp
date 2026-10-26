#include "piper_urdf_kinematics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <random>

static std::string xml() {
    std::ifstream file(ament_index_cpp::get_package_share_directory("piper_description") + "/urdf/piper_description.urdf");
    return std::string(std::istreambuf_iterator<char>(file), {});
}
// Independent composition of URDF joint origins and axes, without KDL.
static Eigen::Isometry3d reference(const PiperUrdfKinematics::Joints& q) {
    urdf::Model model; model.initString(xml());
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    for (size_t i = 0; i < 6; ++i) {
        const auto joint = model.getJoint("joint" + std::to_string(i+1));
        const auto& p = joint->parent_to_joint_origin_transform;
        Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
        origin.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        origin.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
        const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
        t = t * origin * Eigen::AngleAxisd(q[i], axis);
    }
    return t;
}
TEST(UrdfModel, FKAndLimitsMatchUrdf) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    const std::vector<PiperUrdfKinematics::Joints> samples = {
        {0,0,0,0,0,0}, {0.2,0.8,-0.7,0.3,0.4,-0.2}, {-0.8,1.4,-1.8,-0.6,-0.5,1.0},
        {2.618,3.14,-2.967,1.745,1.22,2.0944}};
    for (const auto& q : samples) {
        EXPECT_TRUE(model.valid(q));
        EXPECT_TRUE(model.fk(q).matrix().isApprox(reference(q).matrix(), 1e-10));
    }
    EXPECT_DOUBLE_EQ(model.limits()[3].upper, 1.745);
    EXPECT_DOUBLE_EQ(model.limits()[2].lower, -2.967);
    EXPECT_DOUBLE_EQ(model.maxOpening(), 0.07);
    EXPECT_FALSE(model.valid({0,0,0,1.75,0,0}));
}
TEST(UrdfModel, SolveMultiplePosesAndToolOffset) {
    Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
    tool.translation() = Eigen::Vector3d(0.01,-0.02,0.12);
    tool.linear() = Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitY()).toRotationMatrix();
    for (const auto& offset : {Eigen::Isometry3d::Identity(), tool}) {
        PiperUrdfKinematics model(xml(), "base_link", "link6", offset);
        for (const auto& q : std::vector<PiperUrdfKinematics::Joints>{
                {0.2,0.8,-0.7,0.3,0.4,-0.2}, {-0.8,1.4,-1.8,-0.6,-0.5,1.0}, {0.5,1.8,-2.0,0.7,0.6,-0.5}}) {
            const Eigen::Isometry3d target = reference(q) * offset;
            auto seed = q;
            for (auto& v : seed) v += 0.05;
            const auto solved = model.solve(seed, target, 400, 1e-5, 1e-4, 0.01);
            EXPECT_TRUE(model.valid(solved));
            const auto error = model.error(solved, target);
            EXPECT_LE(error.head<3>().norm(), 1e-5);
            EXPECT_LE(error.tail<3>().norm(), 1e-4);
        }
    }
}
TEST(UrdfModel, RejectsUnreachablePoseAndWrongChain) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity(); target.translation().x() = 100;
    EXPECT_THROW(model.solve({0,0,0,0,0,0},target,50,0.001,0.01,0.03), std::runtime_error);
    EXPECT_THROW(PiperUrdfKinematics(xml(),"base_link","link7",Eigen::Isometry3d::Identity()),std::invalid_argument);
}
TEST(UrdfModel, ReachableTargetsFromStartupSeed) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    for (const auto& q : std::vector<PiperUrdfKinematics::Joints>{
            {0.2,0.8,-0.7,0.3,0.4,-0.2}, {-0.8,1.4,-1.8,-0.6,-0.5,1.0},
            {0.5,1.8,-2.0,0.7,0.6,-0.5}, {0.0,0.5,-1.0,0.0,0.5,0.0}}) {
        const auto target = reference(q);
        EXPECT_NO_THROW({
            const auto result = model.solve({0,0,0,0,0,0}, target, 200, 0.001, 0.01, 0.03);
            EXPECT_TRUE(model.valid(result));
            EXPECT_LE(model.error(result,target).head<3>().norm(), 0.001);
            EXPECT_LE(model.error(result,target).tail<3>().norm(), 0.01);
        });
    }
}

TEST(UrdfModel, StartupSeedAcrossWorkspace) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    std::mt19937 generator(42);
    std::uniform_real_distribution<double> fraction(0.2, 0.8);
    for (int sample = 0; sample < 120; ++sample) {
        PiperUrdfKinematics::Joints q;
        for (size_t i = 0; i < 6; ++i) {
            const auto limit = model.limits()[i];
            q[i] = limit.lower + fraction(generator) * (limit.upper-limit.lower);
        }
        SCOPED_TRACE(sample);
        const auto target = reference(q);
        EXPECT_NO_THROW({
            const auto result = model.solve({0,0,0,0,0,0}, target, 200, 0.001, 0.01, 0.03);
            EXPECT_TRUE(model.valid(result));
            EXPECT_LE(model.error(result,target).head<3>().norm(), 0.001);
            EXPECT_LE(model.error(result,target).tail<3>().norm(), 0.01);
        });
    }
}

TEST(UrdfModel, SmallAxisDragsInsideWorkspace) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    const PiperUrdfKinematics::Joints seed{0.2,0.8,-0.7,0.3,0.4,-0.2};
    const auto original = reference(seed);
    for (int axis = 0; axis < 6; ++axis) for (double sign : {-1.0, 1.0}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(sign);
        auto target = original;
        if (axis < 3) target.translation()(axis) += sign * 0.005;
        else target.linear() = Eigen::AngleAxisd(sign * 2.0*M_PI/180.0,
            Eigen::Vector3d::Unit(axis-3)).toRotationMatrix() * original.linear();
        EXPECT_NO_THROW({
            const auto solved = model.solve(seed,target,200,0.001,0.01,0.03);
            EXPECT_TRUE(model.valid(solved));
            EXPECT_LE(model.error(solved,target).head<3>().norm(),0.001);
            EXPECT_LE(model.error(solved,target).tail<3>().norm(),0.01);
        });
    }
}
TEST(UrdfModel, ContinuousReachablePath) {
    PiperUrdfKinematics model(xml(), "base_link", "link6", Eigen::Isometry3d::Identity());
    PiperUrdfKinematics::Joints seed{0.2,0.8,-0.7,0.3,0.4,-0.2};
    for (int sample = 0; sample <= 100; ++sample) {
        auto q = PiperUrdfKinematics::Joints{0.2,0.8,-0.7,0.3,0.4,-0.2};
        for (size_t i=0;i<6;++i) q[i] += 0.1*std::sin(sample*0.04+i*0.2);
        const auto target = reference(q);
        ASSERT_NO_THROW(seed = model.solve(seed,target,200,0.001,0.01,0.03));
        EXPECT_TRUE(model.valid(seed));
        EXPECT_LE(model.error(seed,target).head<3>().norm(),0.001);
        EXPECT_LE(model.error(seed,target).tail<3>().norm(),0.01);
    }
}

TEST(UrdfModel, ZeroPosePositiveXMeetsBothToleranceBounds) {
    PiperUrdfKinematics model(xml(),"base_link","link6",Eigen::Isometry3d::Identity());
    auto target = reference({0,0,0,0,0,0});
    target.translation().x() += 0.005;
    const auto q = model.solve({0,0,0,0,0,0},target,200,0.001,0.01,0.03);
    EXPECT_TRUE(model.valid(q));
    EXPECT_LE(model.error(q,target).head<3>().norm(),0.001);
    EXPECT_LE(model.error(q,target).tail<3>().norm(),0.01);
}
