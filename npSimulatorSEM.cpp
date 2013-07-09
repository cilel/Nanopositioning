/****************************************************************************
 *
 * This is a simulated platform in a SEM.
 *****************************************************************************/


/*!
  \file vpSimulatorCamera.cpp
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpDebug.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobotException.h>
//#include <visp/vpSimulatorCamera.h>
//#include "npSimulatorSEM.h"

/*!
  Default constructor that sets the transformation between
  world frame and camera frame to identity.

*/
npSimulatorSEM::npSimulatorSEM()
{
  init() ;
}

/*!
  Robot initialisation.

  Robot jacobian expressed in the end-effector frame \f$ {^e}{\bf J}_e \f$
  is set to identity (see get_eJe()).

*/
void npSimulatorSEM::init()
{
  nDof = 6;
  eJe.resize(6,6) ;
  eJe.setIdentity() ;
  eJeAvailable = true;
  fJeAvailable = false;
  areJointLimitsAvailable = false;
  qmin = NULL;
  qmax = NULL;

  setMaxTranslationVelocity(1.); // vx, vy and vz max set to 1 m/s
  setMaxRotationVelocity(vpMath::rad(90)); // wx, wy and wz max set to 90 deg/s
}


/*!
  Destructor.

*/
npSimulatorSEM::~npSimulatorSEM()
{
}

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation. Here this transformation is equal to identity
  since camera frame and end-effector frame are at the same location.

*/
void
npSimulatorSEM::get_cVe(vpVelocityTwistMatrix &cVe)
{
  vpVelocityTwistMatrix cVe_;
  cVe = cVe_;
}

/*!
  Get the robot jacobian expressed in the end-effector frame.
  For that simple robot the Jacobian is the identity.

  \param eJe : A 6 by 6 matrix representing the robot jacobian \f$ {^e}{\bf
  J}_e\f$ expressed in the end-effector frame. Yhis matrix is equal to identity.
*/
void
npSimulatorSEM::get_eJe(vpMatrix &eJe)
{
  eJe = this->eJe ;
}

/*!
  Get the robot position in the world frame.

*/
void
npSimulatorSEM::getPosition(vpHomogeneousMatrix &wMc) const
{
  wMc = this->wMc_ ;
}

/*
  Get the current position of the robot.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesien frame,
  - joint (articular) coordinates of each axes
  - in a reference or fixed cartesien frame attached to the robot base
  - in a mixt cartesien frame (translation in reference frame, and rotation in camera frame)

  \param position : Measured position of the robot:
  - in camera cartesien frame, a 6 dimension vector, set to 0.

  - in articular, a 6 dimension vector corresponding to the articular
  position of each dof, first the 3 translations, then the 3
  articular rotation positions represented by a vpRxyzVector.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector).
*/
void npSimulatorSEM::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  q.resize (6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME :
    q = 0;
    break;

  case vpRobot::ARTICULAR_FRAME :
  case vpRobot::REFERENCE_FRAME : {
    // Convert wMc_ to a position
    // From fMc extract the pose
    vpRotationMatrix wRc;
    this->wMc_.extract(wRc);
    vpRxyzVector rxyz;
    rxyz.buildFrom(wRc);

    for (unsigned int i=0; i < 3; i++) {
      q[i] = this->wMc_[i][3]; // translation x,y,z
      q[i+3] = rxyz[i]; // Euler rotation x,y,z
    }

    break;
    }
  case vpRobot::MIXT_FRAME :
    std::cout << "MIXT_FRAME is not implemented in vpSimulatorCamera::getPosition()" << std::endl;
  }
}

/*!
  Send to the controller a velocity.

  \param frame : Control frame type. Only articular (vpRobot::ARTICULAR_FRAME)
  and camera frame (vpRobot::CAMERA_FRAME) are implemented.

  \param v : Velocity to apply to the robot.

  - In the camera frame, this velocity is represented by a vector of dimension 6
  \f$ {\bf v} = [{\bf t}, {\bf \theta u }]^t \f$ where \f$ \bf t \f$ is a
  translation vector and \f$ {\bf \theta u} \f$ is a rotation vector (see
  vpThetaUVector): \f$ {\bf v} = [t_x, t_y, t_z, {\theta u}_x, {\theta u}_y,
  {\theta u}_z] \f$ (see vpTranslationVector and vpThetaUVector).

  - In articular, this velocity is represented by a 6 dimension vector \f$
  \dot{{\bf q}} = [{\bf t}, {\bf \theta u}]^t \f$ where \f$ \bf t \f$ is a
  translation vector and \f$ {\bf \theta u} \f$ is a rotation vector (see
  vpThetaUVector): \f$ \dot{{\bf q}} = [t_x, t_y, t_z, {\theta u}_x, {\theta
  u}_y, {\theta u}_z] \f$ (see vpTranslationVector and vpThetaUVector). The
  robot jacobian \f$ {^e}{\bf J}_e\f$ expressed in the end-effector frame is
  here set to identity.

  We use the exponential map (vpExponentialMap) to update the camera location.
  Sampling time can be set using setSamplingTime().

  \sa setSamplingTime()

*/
void
npSimulatorSEM::setVelocity(const vpRobot::vpControlFrameType frame,
                               const vpColVector &v)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ()) {
    setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
  }

  switch (frame)
  {
  case vpRobot::ARTICULAR_FRAME:
  case vpRobot::CAMERA_FRAME: {
      vpColVector v_max(6);

      for (unsigned int i=0; i<3; i++)
        v_max[i] = getMaxTranslationVelocity();
      for (unsigned int i=3; i<6; i++)
        v_max[i] = getMaxRotationVelocity();

      vpColVector v_sat = vpRobot::saturateVelocities(v, v_max, true);

      wMc_ = wMc_ * vpExponentialMap::direct(v_sat, delta_t_);
      setRobotFrame(frame);
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    vpERROR_TRACE ("Cannot set a velocity in the reference frame: "
                   "functionality not implemented");
    throw vpRobotException (vpRobotException::wrongStateError,
                            "Cannot set a velocity in the reference frame:"
                            "functionality not implemented");
    break ;
  case vpRobot::MIXT_FRAME:
    vpERROR_TRACE ("Cannot set a velocity in the mixt frame: "
                   "functionality not implemented");
    throw vpRobotException (vpRobotException::wrongStateError,
                            "Cannot set a velocity in the mixt frame:"
                            "functionality not implemented");

    break ;
  }
}

/*!
  Set the robot position in the world frame.

  \param wMc : Transformation from world frame to camera frame.
*/
void npSimulatorSEM::setPosition(const vpHomogeneousMatrix &wMc)
{
  if (vpRobot::STATE_POSITION_CONTROL != getRobotState ()) {
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  this->wMc_ = wMc;
}
