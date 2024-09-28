#pragma once

#include <vector>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <SpaceVecAlg/Conversions.h>
#include <RBDyn/parsers/urdf.h>
#include <Eigen/Geometry>

class rbdyn_wrapper
{
public:
  struct EefState
  {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
  };

  void init_rbdyn(const std::string &urdf_string, const std::string &end_effector)
  {
    // Convert URDF to RBDyn
    _rbdyn_urdf = rbd::parsers::from_urdf(urdf_string);

    _rbd_indices.clear();

    for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++)
    {
      if (_rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
        _rbd_indices.push_back(i);
    }

    for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++)
    {
      if (_rbdyn_urdf.mb.body(i).name() == end_effector)
      {
        _ef_index = i;
        return;
      }
    }
    throw std::runtime_error("Index for end effector link " + end_effector + " not found in URDF. Aborting.");
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);

    _update_urdf_state(_rbdyn_urdf.mbc, q, dq);

    // Compute jacobian
    rbd::Jacobian jac(_rbdyn_urdf.mb, _rbdyn_urdf.mb.body(_ef_index).name());

    rbd::forwardKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
    rbd::forwardVelocity(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

    return jac.jacobian(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);
  }

  EefState perform_fk(const Eigen::VectorXd &q)
  {
    _rbdyn_urdf.mbc.zero(_rbdyn_urdf.mb);

    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t rbd_index = _rbd_indices[i];
      double jt = q[i];
      // wrap in [-pi,pi]
      jt = _wrap_angle(jt);
      _rbdyn_urdf.mbc.q[rbd_index][0] = jt;
    }

    rbd::forwardKinematics(_rbdyn_urdf.mb, _rbdyn_urdf.mbc);

    sva::PTransformd tf = _rbdyn_urdf.mbc.bodyPosW[_ef_index];

    Eigen::Matrix4d eig_tf = sva::conversions::toHomogeneous(tf);
    Eigen::Vector3d trans = eig_tf.col(3).head(3);
    Eigen::Matrix3d rot_mat = eig_tf.block(0, 0, 3, 3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(rot_mat).normalized();

    return {trans, quat};
  }

  int n_joints() const
  {
    return _rbd_indices.size();
  }

  std::string root_link() const
  {
    return _rbdyn_urdf.mb.body(0).name();
  }

private:
  void _update_urdf_state(rbd::MultiBodyConfig &mbc, const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t rbd_index = _rbd_indices[i];

      if (q.size() > i)
        mbc.q[rbd_index][0] = q[i];
      if (dq.size() > i)
        mbc.alpha[rbd_index][0] = dq[i];
    }
  }

  double _wrap_angle(const double &angle) const
  {
    double wrapped;
    if ((angle <= M_PI) && (angle >= -M_PI))
    {
      wrapped = angle;
    }
    else if (angle < 0.0)
    {
      wrapped = std::fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
    }
    else
    {
      wrapped = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    }
    return wrapped;
  }

  rbd::parsers::ParserResult _rbdyn_urdf;
  std::vector<size_t> _rbd_indices;
  size_t _ef_index;
};
