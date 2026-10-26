#include "piper_urdf_kinematics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iterator>
#include <iostream>

// Diagnostic CSV; a rejection is not proof that the target is unreachable.
int main() {
    std::ifstream file(ament_index_cpp::get_package_share_directory("piper_description") + "/urdf/piper_description.urdf");
    const std::string xml(std::istreambuf_iterator<char>(file), {});
    PiperUrdfKinematics model(xml,"base_link","link6",Eigen::Isometry3d::Identity());
    std::cout << "start,axis,sign,accepted,position_mm,orientation_deg\n";
    for (int start=0;start<2;++start) {
        const auto seed = start ? PiperUrdfKinematics::Joints{0.2,0.8,-0.7,0.3,0.4,-0.2} : PiperUrdfKinematics::Joints{};
        const auto original = model.fk(seed);
        for (int axis=0;axis<6;++axis) for (double sign : {-1.0,1.0}) {
            auto target = original;
            if (axis<3) target.translation()(axis) += sign*0.005;
            else target.linear() = Eigen::AngleAxisd(sign*2*M_PI/180.0,Eigen::Vector3d::Unit(axis-3)).toRotationMatrix()*original.linear();
            try {
                const auto solved=model.solve(seed,target,200,0.001,0.01,0.03);
                const auto error=model.error(solved,target);
                std::cout << start << ',' << axis << ',' << sign << ",1," << error.head<3>().norm()*1000 << ',' << error.tail<3>().norm()*180/M_PI << '\n';
            } catch (const std::exception& e) {
                std::cout << start << ',' << axis << ',' << sign << ",0,nan,nan\n";
                std::cerr << "start=" << start << " axis=" << axis << " sign=" << sign << " " << e.what() << '\n';
            }
        }
    }
}
