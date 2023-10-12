#include <iostream>
#include "sophus/geometry.hpp"

int main() {
  // The following demonstrates the group multiplication of rotation matrices

  // Create rotation matrices from rotations around the x and y and z axes:
  const double kPi = Sophus::Constants<double>::pi();
  Sophus::SO3d R1 = Sophus::SO3d::rotX(kPi / 4);
  Sophus::SO3d R2 = Sophus::SO3d::rotY(kPi / 6);
  Sophus::SO3d R3 = Sophus::SO3d::rotZ(-kPi / 3);

  std::cout << "The rotation matrices are" << std::endl;
  std::cout << "R1:\n" << R1.matrix() << std::endl;
  std::cout << "R2:\n" << R2.matrix() << std::endl;
  std::cout << "R3:\n" << R3.matrix() << std::endl;
  std::cout << "Their product R1*R2*R3:\n"
            << (R1 * R2 * R3).matrix() << std::endl;
  std::cout << std::endl;

  // Rotation matrices can act on vectors
  Eigen::Vector3d x;
  x << 0.0, 0.0, 1.0;
  std::cout << "Rotation matrices can act on vectors" << std::endl;
  std::cout << "x\n" << x << std::endl;
  std::cout << "R2*x\n" << R2 * x << std::endl;
  std::cout << "R1*(R2*x)\n" << R1 * (R2 * x) << std::endl;
  std::cout << "(R1*R2)*x\n" << (R1 * R2) * x << std::endl;
  std::cout << std::endl;

  // SO(3) are internally represented as unit quaternions.
  std::cout << "R1 in matrix form:\n" << R1.matrix() << std::endl;
  std::cout << "R1 in unit quaternion form:\n"
            << R1.unit_quaternion().coeffs() << std::endl;
  // Note that the order of coefficiences of Eigen's quaternion class is
  // (imag0, imag1, imag2, real)
  std::cout << std::endl;

  Sophus::SO3d world_from_camera_rotation =
      Sophus::SO3d::rotX(kPi / 4);
  // Then the position of the camera in the world.
  Eigen::Vector3d camera_in_world(0.0, 0.0, 1.0);

  // The pose (position and orientation) of the camera in the world is
  // constructed by its orientation ``world_from_camera_rotation`` as well as
  // its position ``camera_in_world``.
  Sophus::SE3d world_anchored_camera_pose(
      world_from_camera_rotation, camera_in_world);

  // SE(3) naturally representation is a 4x4 matrix which can be accessed using
  // the .matrix() method:
  std::cout << "world_anchored_camera_pose:\n"
            << world_anchored_camera_pose.matrix()(3,3) << std::endl;

}