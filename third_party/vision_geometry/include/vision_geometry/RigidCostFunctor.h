#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace Eigen;
using namespace ceres;

class RigidCostFunctor {

public:
  RigidCostFunctor(const Matrix3d &intrinsics, const Vector2d &image_corners, const Vector3d &world_corner): intrinsics_(intrinsics) {
      X_world = world_corner[0];
      Y_world = world_corner[1];
      Z_world = world_corner[2];

      x_image = image_corners[0];
      y_image = image_corners[1];
  }

  template <typename T>
  bool operator()(const T* const rt, T* residuals) const {

    T quaternion[4];
    quaternion[0] = rt[0];
    quaternion[1] = rt[1];
    quaternion[2] = rt[2];
    quaternion[3] = rt[3];

    T translation[3];
    translation[0] = rt[4];
    translation[1] = rt[5];
    translation[2] = rt[6];

    Eigen::Matrix<T, 3, 1> world_corner;
    world_corner << T(X_world), T(Y_world), T(Z_world);

    Eigen::Matrix<T, 3, 3> intrinsics;
    intrinsics << T(intrinsics_(0, 0)), T(intrinsics_(0, 1)), T(intrinsics_(0, 2)),
                  T(intrinsics_(1, 0)), T(intrinsics_(1, 1)), T(intrinsics_(1, 2)),
                  T(intrinsics_(2, 0)), T(intrinsics_(2, 1)), T(intrinsics_(2, 2));

    Eigen::Quaternion<T> q = Map<const Eigen::Quaternion<T>>(quaternion);
    Eigen::Matrix<T, 3, 1> trans_vec = Map<const Eigen::Matrix<T, 3, 1>>(translation);

    Eigen::Matrix<T, 3, 1> res = intrinsics * (q * world_corner + trans_vec);
    res /= res[2];

    residuals[0] = res[0] - T(x_image);
    residuals[1] = res[1] - T(y_image);

    return true;
  }

private:
  const Matrix3d &intrinsics_;
  double X_world;
  double Y_world;
  double Z_world;
  double x_image;
  double y_image;

};
