#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>

#include <ceres/rotation.h>
#include <sophus/se3.hpp>

/*
struct PointToPointError_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_CeresAngleAxis(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_CeresAngleAxis, 3, 6>(new PointToPointError_CeresAngleAxis(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const {

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};
*/

struct PointToPointError_SophusSE3{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_SophusSE3(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_SophusSE3, 3, 7>(new PointToPointError_SophusSE3(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const cam1, T* residuals) const {

        //ceres::AngleAxisRotatePoint

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> p; p << T(p_src[0]), T(p_src[1]), T(p_src[2]);

        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3<T> q = Eigen::Map< const Sophus::SE3<T> >(cam1);

        // Rotate the point using Eigen rotations
        p = q.unit_quaternion() * p + q.translation();

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};

/*
struct TestSE3CostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TestSE3CostFunctor(Sophus::SE3d T_aw) : T_aw(T_aw) {}

  template <class T>
  bool operator()(T const* const sT_wa, T* sResiduals) const {
    Eigen::Map<Sophus::SE3<T> const> const T_wa(sT_wa);
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(sResiduals);

    // We are able to mix Sophus types with doubles and Jet types without
    // needing to cast to T.
    residuals = (T_aw * T_wa).log();
    // Reverse order of multiplication. This forces the compiler to verify that
    // (Jet, double) and (double, Jet) SE3 multiplication work correctly.
    residuals = (T_wa * T_aw).log();
    // Finally, ensure that Jet-to-Jet multiplication works.
    residuals = (T_wa * T_aw.cast<T>()).log();
    return true;
  }

  Sophus::SE3d T_aw;
};
*/