#!/usr/bin/env bash
set -euo pipefail

PREFIX="${PREFIX:-/opt/robot/devel}"
SRC_DIR="${SRC_DIR:-/tmp/pivot-local-deps-src}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"

export CMAKE_PREFIX_PATH="$PREFIX:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:$PREFIX/lib/$(gcc -dumpmachine)/pkgconfig:$PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$PREFIX/lib:$PREFIX/lib/$(gcc -dumpmachine):${LD_LIBRARY_PATH:-}"
export PATH="$PREFIX/bin:$PATH"

mkdir -p "$PREFIX" "$SRC_DIR"

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "missing command: $1" >&2
        exit 1
    }
}

fetch_git() {
    local name="$1"
    local url="$2"
    local ref="$3"
    local dir="$SRC_DIR/$name"

    if [ ! -d "$dir/.git" ]; then
        git clone --depth 1 --branch "$ref" "$url" "$dir"
    else
        git -C "$dir" fetch --depth 1 origin "$ref"
        git -C "$dir" checkout FETCH_HEAD
    fi
}

fetch_tar() {
    local name="$1"
    local url="$2"
    local strip="${3:-1}"
    local archive="$SRC_DIR/$name.tar"
    local dir="$SRC_DIR/$name"

    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        curl -L "$url" -o "$archive"
        tar -xf "$archive" -C "$dir" --strip-components="$strip"
    fi
}

cmake_install() {
    local name="$1"
    local source_dir="$2"
    shift 2
    local build_dir="$SRC_DIR/build-$name"

    cmake -S "$source_dir" -B "$build_dir" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DBUILD_SHARED_LIBS=ON \
        "$@"
    cmake --build "$build_dir" -j"$JOBS"
    cmake --install "$build_dir"
}

make_install() {
    local name="$1"
    local source_dir="$2"
    shift 2
    make -C "$source_dir" -j"$JOBS" "$@"
    make -C "$source_dir" install "$@"
}

install_lz4() {
    fetch_git lz4 https://github.com/lz4/lz4.git v1.9.4
    make -C "$SRC_DIR/lz4" -j"$JOBS"
    make -C "$SRC_DIR/lz4" install PREFIX="$PREFIX"
}

install_eigen3() {
    fetch_git eigen https://gitlab.com/libeigen/eigen.git 3.4.0
    cmake_install eigen "$SRC_DIR/eigen" -DBUILD_TESTING=OFF
}

install_boost() {
    fetch_tar boost_1_84_0 https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz
    if [ ! -x "$SRC_DIR/boost_1_84_0/b2" ]; then
        (cd "$SRC_DIR/boost_1_84_0" && ./bootstrap.sh --prefix="$PREFIX")
    fi
    (cd "$SRC_DIR/boost_1_84_0" && ./b2 -j"$JOBS" install \
        --prefix="$PREFIX" \
        link=shared runtime-link=shared threading=multi variant=release \
        --with-atomic --with-chrono --with-date_time --with-filesystem \
        --with-iostreams --with-program_options --with-regex \
        --with-serialization --with-system --with-thread)
}

install_flann() {
    fetch_git flann https://github.com/flann-lib/flann.git 1.9.2
    cmake_install flann "$SRC_DIR/flann" \
        -DBUILD_C_BINDINGS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_MATLAB_BINDINGS=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_TESTS=OFF \
        -DUSE_MPI=OFF
}

install_giflib() {
    fetch_tar giflib-5.2.2 https://downloads.sourceforge.net/project/giflib/giflib-5.x/giflib-5.2.2.tar.gz
    make -C "$SRC_DIR/giflib-5.2.2" -j"$JOBS" libgif.so libgif.a
    make -C "$SRC_DIR/giflib-5.2.2" install-include install-lib PREFIX="$PREFIX"
}

install_ffmpeg() {
    fetch_git ffmpeg https://github.com/FFmpeg/FFmpeg.git n6.1.2
    (cd "$SRC_DIR/ffmpeg" && ./configure \
        --prefix="$PREFIX" \
        --enable-shared \
        --disable-static \
        --disable-programs \
        --disable-doc \
        --disable-debug \
        --disable-x86asm \
        --disable-avdevice \
        --disable-postproc \
        --disable-network \
        --disable-everything \
        --enable-encoder=mpeg4 \
        --enable-muxer=mp4 \
        --enable-protocol=file \
        --enable-swscale)
    make -C "$SRC_DIR/ffmpeg" -j"$JOBS"
    make -C "$SRC_DIR/ffmpeg" install
}

install_glew() {
    fetch_tar glew-2.2.0 https://downloads.sourceforge.net/project/glew/glew/2.2.0/glew-2.2.0.tgz
    make -C "$SRC_DIR/glew-2.2.0" -j"$JOBS" GLEW_DEST="$PREFIX"
    make -C "$SRC_DIR/glew-2.2.0" install GLEW_DEST="$PREFIX"
}

install_glm() {
    fetch_git glm https://github.com/g-truc/glm.git 1.0.1
    cmake_install glm "$SRC_DIR/glm" \
        -DGLM_BUILD_TESTS=OFF \
        -DBUILD_SHARED_LIBS=OFF
}

install_yaml_cpp() {
    fetch_git yaml-cpp https://github.com/jbeder/yaml-cpp.git 0.8.0
    cmake_install yaml-cpp "$SRC_DIR/yaml-cpp" \
        -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF
}

install_tinyxml2() {
    fetch_git tinyxml2 https://github.com/leethomason/tinyxml2.git 10.0.0
    cmake_install tinyxml2 "$SRC_DIR/tinyxml2" -Dtinyxml2_BUILD_TESTING=OFF
}

install_tinyxml() {
    fetch_git tinyxml https://github.com/jslee02/tinyxml.git main
    cmake_install tinyxml "$SRC_DIR/tinyxml" -DBUILD_TESTING=OFF
}

install_glog() {
    fetch_git glog https://github.com/google/glog.git v0.7.1
    cmake_install glog "$SRC_DIR/glog" -DWITH_GTEST=OFF -DWITH_GFLAGS=OFF
}

install_glfw() {
    fetch_git glfw https://github.com/glfw/glfw.git 3.4
    cmake_install glfw "$SRC_DIR/glfw" \
        -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_DOCS=OFF \
        -DGLFW_BUILD_WAYLAND=OFF -DGLFW_BUILD_X11=ON
}

install_assimp() {
    fetch_git assimp https://github.com/assimp/assimp.git v5.4.3
    cmake_install assimp "$SRC_DIR/assimp" \
        -DASSIMP_BUILD_TESTS=OFF -DASSIMP_INSTALL_PDB=OFF \
        -DASSIMP_BUILD_ASSIMP_TOOLS=OFF
}

install_console_bridge() {
    fetch_git console_bridge https://github.com/ros/console_bridge.git 1.0.2
    cmake_install console_bridge "$SRC_DIR/console_bridge"
}

install_urdfdom_headers() {
    fetch_git urdfdom_headers https://github.com/ros/urdfdom_headers.git 1.1.2
    cmake_install urdfdom_headers "$SRC_DIR/urdfdom_headers"
}

install_urdfdom() {
    fetch_git urdfdom https://github.com/ros/urdfdom.git 3.1.1
    rm -rf "$SRC_DIR/build-urdfdom"
    cmake_install urdfdom "$SRC_DIR/urdfdom" \
        -DTinyXML_INCLUDE_DIR="$PREFIX/include" \
        -DTinyXML_LIBRARY="$PREFIX/lib/libtinyxml.so" \
        -DURDFDOM_BUILD_TESTS=OFF
}

install_orocos_kdl() {
    fetch_git orocos_kinematics_dynamics https://github.com/orocos/orocos_kinematics_dynamics.git v1.5.1
    cmake_install orocos_kdl "$SRC_DIR/orocos_kinematics_dynamics/orocos_kdl" \
        -DBUILD_TESTING=OFF -DENABLE_TESTS=OFF
}

install_qpoases() {
    fetch_git qpOASES https://github.com/coin-or/qpOASES.git releases/3.2.2
    cmake_install qpOASES "$SRC_DIR/qpOASES" -DBUILD_EXAMPLES=OFF
}

install_nlopt() {
    fetch_git nlopt https://github.com/stevengj/nlopt.git v2.7.1
    cmake_install nlopt "$SRC_DIR/nlopt" \
        -DNLOPT_PYTHON=OFF \
        -DNLOPT_OCTAVE=OFF \
        -DNLOPT_MATLAB=OFF \
        -DNLOPT_GUILE=OFF \
        -DNLOPT_SWIG=OFF \
        -DNLOPT_TESTS=OFF
}

install_pinocchio() {
    fetch_git pinocchio https://github.com/stack-of-tasks/pinocchio.git v3.4.0
    perl -0pi -e 's/cmake_minimum_required\(VERSION 3\.10\)/cmake_minimum_required(VERSION 3.22)/' \
        "$SRC_DIR/pinocchio/CMakeLists.txt"
    rm -rf "$SRC_DIR/build-pinocchio"
    cmake_install pinocchio "$SRC_DIR/pinocchio" \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=OFF \
        -DBUILD_WITH_HPP_FCL_SUPPORT=OFF \
        -DBUILD_WITH_URDF_SUPPORT=ON \
        -DBUILD_TESTING=OFF
}

install_trac_ik() {
    fetch_git trac_ik https://github.com/traclabs/trac_ik.git master
    fetch_git kdl_parser https://github.com/ros/kdl_parser.git 1.14.2

    local compat_dir="$SRC_DIR/trac_ik_local"
    mkdir -p "$compat_dir/ros" "$compat_dir/urdf"
    cat > "$compat_dir/ros/ros.h" <<'EOF'
#pragma once
#define ROS_DEBUG(...) do {} while (0)
#define ROS_DEBUG_NAMED(...) do {} while (0)
#define ROS_DEBUG_STREAM_NAMED(...) do {} while (0)
#define ROS_ERROR(...) do {} while (0)
#define ROS_ERROR_THROTTLE(...) do {} while (0)
#define ROS_FATAL(...) do {} while (0)
#define ROS_FATAL_NAMED(...) do {} while (0)
#define ROS_FATAL_STREAM(...) do {} while (0)
#define ROS_WARN_THROTTLE(...) do {} while (0)
#define ROS_WARN_STREAM_NAMED(...) do {} while (0)
EOF
    cat > "$compat_dir/urdf/model.h" <<'EOF'
#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf {
class Model : public ModelInterface {
public:
  bool initString(const std::string& xml_string) {
    ModelInterfaceSharedPtr parsed = parseURDF(xml_string);
    if (!parsed) {
      clear();
      return false;
    }
    *static_cast<ModelInterface*>(this) = *parsed;
    return true;
  }
};
}  // namespace urdf
EOF

    perl -ni -e 'print unless /max_iterations, double timeout, double epsilon/' \
        "$SRC_DIR/trac_ik/trac_ik_lib/include/trac_ik/trac_ik.hpp"
    perl -0pi -e '
        s/TRAC_IK\(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param = "\/robot_description", double _maxtime = 0\.005, double _eps = 1e-5, SolveType _type = Speed\);/TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param = "\/robot_description", double _maxtime = 0.005, double _eps = 1e-5, SolveType _type = Speed);\n\n  TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_path, int max_iterations, double timeout, double epsilon, int num_threads, bool position_only, bool use_seed, SolveType type = Speed);/
    ' "$SRC_DIR/trac_ik/trac_ik_lib/include/trac_ik/trac_ik.hpp"

    perl -0pi -e '
        s/#include <urdf\/model.h>/#include <urdf\/model.h>\n#include <fstream>\n#include <sstream>\n#include <iostream>/;
        s/TRAC_IK::TRAC_IK\(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type\) :\n  initialized\(false\),\n  eps\(_eps\),\n  maxtime\(_maxtime\),\n  solvetype\(_type\)\n\{.*?\n\}\n\n\nTRAC_IK::TRAC_IK\(const KDL::Chain& _chain/TRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type) :\n  initialized(false),\n  eps(_eps),\n  maxtime(_maxtime),\n  solvetype(_type)\n{\n  std::ifstream input(URDF_param);\n  std::stringstream buffer;\n  if (input.good()) {\n    buffer << input.rdbuf();\n  } else {\n    buffer << URDF_param;\n  }\n  const std::string xml_string = buffer.str();\n\n  urdf::Model robot_model;\n  if (!robot_model.initString(xml_string)) {\n    std::cerr << "TRAC-IK failed to parse URDF input: " << URDF_param << std::endl;\n    return;\n  }\n\n  KDL::Tree tree;\n  if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {\n    std::cerr << "TRAC-IK failed to build KDL tree from URDF: " << URDF_param << std::endl;\n    return;\n  }\n  if (!tree.getChain(base_link, tip_link, chain)) {\n    std::cerr << "TRAC-IK could not find chain " << base_link << " -> " << tip_link << std::endl;\n    return;\n  }\n\n  std::vector<KDL::Segment> chain_segs = chain.segments;\n  urdf::JointConstSharedPtr joint;\n  lb.resize(chain.getNrOfJoints());\n  ub.resize(chain.getNrOfJoints());\n\n  unsigned int joint_num = 0;\n  for (unsigned int i = 0; i < chain_segs.size(); ++i) {\n    joint = robot_model.getJoint(chain_segs[i].getJoint().getName());\n    if (joint && joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {\n      joint_num++;\n      if (joint->type != urdf::Joint::CONTINUOUS && joint->limits) {\n        if (joint->safety) {\n          lb(joint_num - 1) = std::max(joint->limits->lower, joint->safety->soft_lower_limit);\n          ub(joint_num - 1) = std::min(joint->limits->upper, joint->safety->soft_upper_limit);\n        } else {\n          lb(joint_num - 1) = joint->limits->lower;\n          ub(joint_num - 1) = joint->limits->upper;\n        }\n      } else {\n        lb(joint_num - 1) = std::numeric_limits<float>::lowest();\n        ub(joint_num - 1) = std::numeric_limits<float>::max();\n      }\n    }\n  }\n\n  initialize();\n}\n\nTRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_path, int, double timeout, double epsilon, int, bool, bool, SolveType type) :\n  TRAC_IK(base_link, tip_link, urdf_path, timeout, epsilon, type)\n{\n}\n\n\nTRAC_IK::TRAC_IK(const KDL::Chain& _chain/s
    ' "$SRC_DIR/trac_ik/trac_ik_lib/src/trac_ik.cpp"

    cat > "$compat_dir/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(trac_ik_local VERSION 1.6.7 LANGUAGES CXX)

find_package(orocos_kdl REQUIRED CONFIG)
find_package(Eigen3 REQUIRED CONFIG)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
pkg_check_modules(URDFDOM_HEADERS REQUIRED urdfdom_headers)
pkg_check_modules(TINYXML REQUIRED tinyxml)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

add_library(trac_ik SHARED
    "$SRC_DIR/trac_ik/trac_ik_lib/src/kdl_tl.cpp"
    "$SRC_DIR/trac_ik/trac_ik_lib/src/nlopt_ik.cpp"
    "$SRC_DIR/trac_ik/trac_ik_lib/src/trac_ik.cpp"
    "$SRC_DIR/kdl_parser/kdl_parser/src/kdl_parser.cpp")

target_compile_features(trac_ik PUBLIC cxx_std_11)
target_include_directories(trac_ik PUBLIC
    "\$<BUILD_INTERFACE:$SRC_DIR/trac_ik/trac_ik_lib/include>"
    "\$<BUILD_INTERFACE:$SRC_DIR/kdl_parser/kdl_parser/include>"
    "\$<BUILD_INTERFACE:$compat_dir>"
    "\$<INSTALL_INTERFACE:include>")
target_include_directories(trac_ik PRIVATE
    \${NLOPT_INCLUDE_DIRS}
    \${URDFDOM_INCLUDE_DIRS}
    \${URDFDOM_HEADERS_INCLUDE_DIRS}
    \${TINYXML_INCLUDE_DIRS}
    \${TINYXML2_INCLUDE_DIRS}
    \${Boost_INCLUDE_DIRS})
target_compile_options(trac_ik PRIVATE -include boost/math/tools/precision.hpp)
target_link_directories(trac_ik PRIVATE
    \${NLOPT_LIBRARY_DIRS}
    \${URDFDOM_LIBRARY_DIRS}
    \${TINYXML_LIBRARY_DIRS}
    \${TINYXML2_LIBRARY_DIRS})
target_link_libraries(trac_ik PUBLIC
    orocos-kdl
    Eigen3::Eigen
    Boost::date_time
    \${NLOPT_LIBRARIES}
    \${URDFDOM_LIBRARIES}
    \${TINYXML_LIBRARIES}
    \${TINYXML2_LIBRARIES})

install(TARGETS trac_ik EXPORT trac_ikTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY "$SRC_DIR/trac_ik/trac_ik_lib/include/" DESTINATION include)
install(DIRECTORY "$SRC_DIR/kdl_parser/kdl_parser/include/" DESTINATION include)
install(EXPORT trac_ikTargets
    FILE trac_ikTargets.cmake
    NAMESPACE trac_ik::
    DESTINATION lib/cmake/trac_ik)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "\${CMAKE_CURRENT_BINARY_DIR}/trac_ikConfigVersion.cmake"
    VERSION \${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
    "$compat_dir/trac_ikConfig.cmake.in"
    "\${CMAKE_CURRENT_BINARY_DIR}/trac_ikConfig.cmake"
    INSTALL_DESTINATION lib/cmake/trac_ik)
install(FILES
    "\${CMAKE_CURRENT_BINARY_DIR}/trac_ikConfig.cmake"
    "\${CMAKE_CURRENT_BINARY_DIR}/trac_ikConfigVersion.cmake"
    DESTINATION lib/cmake/trac_ik)
EOF

    cat > "$compat_dir/trac_ikConfig.cmake.in" <<'EOF'
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(orocos_kdl CONFIG)
find_dependency(Boost COMPONENTS date_time)
include("${CMAKE_CURRENT_LIST_DIR}/trac_ikTargets.cmake")
set(trac_ik_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set(trac_ik_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
set(trac_ik_LIBRARIES trac_ik::trac_ik)
EOF

    rm -rf "$SRC_DIR/build-trac_ik_local"
    cmake_install trac_ik_local "$compat_dir"
}

install_pcl() {
    fetch_git pcl https://github.com/PointCloudLibrary/pcl.git pcl-1.13.1
    if ! grep -q '^#include <algorithm>' "$SRC_DIR/pcl/io/src/ply/ply_parser.cpp"; then
        perl -0pi -e 's/#include <pcl\/io\/ply\/ply_parser\.h>/#include <pcl\/io\/ply\/ply_parser.h>\n#include <algorithm>/' \
            "$SRC_DIR/pcl/io/src/ply/ply_parser.cpp"
    fi
    rm -rf "$SRC_DIR/build-pcl"
    cmake_install pcl "$SRC_DIR/pcl" \
        -DBUILD_apps=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_global_tests=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_2d=OFF \
        -DBUILD_common=ON \
        -DBUILD_octree=ON \
        -DBUILD_search=ON \
        -DBUILD_kdtree=ON \
        -DBUILD_sample_consensus=ON \
        -DBUILD_filters=ON \
        -DBUILD_io=ON \
        -DBUILD_features=OFF \
        -DBUILD_geometry=OFF \
        -DBUILD_keypoints=OFF \
        -DBUILD_ml=OFF \
        -DBUILD_outofcore=OFF \
        -DBUILD_people=OFF \
        -DBUILD_recognition=OFF \
        -DBUILD_registration=OFF \
        -DBUILD_segmentation=OFF \
        -DBUILD_stereo=OFF \
        -DBUILD_surface=OFF \
        -DBUILD_surface_on_nurbs=OFF \
        -DBUILD_tracking=OFF \
        -DBUILD_visualization=OFF \
        -DWITH_OPENNI=OFF \
        -DWITH_OPENNI2=OFF \
        -DWITH_QHULL=OFF \
        -DWITH_VTK=OFF
}

main() {
    need_cmd cmake
    need_cmd git
    need_cmd gcc
    need_cmd curl
    need_cmd make

    install_lz4
    install_eigen3
    install_boost
    install_flann
    install_giflib
    install_ffmpeg
    install_glew
    install_glm
    install_yaml_cpp
    install_tinyxml2
    install_tinyxml
    install_glog
    install_glfw
    install_assimp
    install_console_bridge
    install_urdfdom_headers
    install_urdfdom
    install_orocos_kdl
    install_qpoases
    install_nlopt
    install_pinocchio
    install_trac_ik
    install_pcl

    echo
    echo "Installed local dependencies under: $PREFIX"
    echo "Use with:"
    echo "  cmake -S . -B build -DROBOT_DEVEL_PATH=$PREFIX"
}

main "$@"
