/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_torque_control_tsid_controller_H__
#define __sot_torque_control_tsid_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (tsid_controller_EXPORTS)
#    define SOTTSIDCONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTTSIDCONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTSIDCONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/common.hh>
#include <map>
#include "boost/assign.hpp"

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-output.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

        class TsidController;

        class TsidContactHelper
        {
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          TsidContactHelper(const std::string& frame_name,
                          TsidController& controller,
                          tsid::robots::RobotWrapper& robot,
                          tsid::InverseDynamicsFormulationAccForce& invDyn);

          const std::string m_frameName;
          int m_frameId;

          tsid::InverseDynamicsFormulationAccForce& m_invDyn;
          tsid::robots::RobotWrapper& m_robot;
          TsidController& m_controller;

          DECLARE_SIGNAL_IN(ref_pos,              dynamicgraph::Vector);
          DECLARE_SIGNAL_IN(ref_vel,              dynamicgraph::Vector);
          DECLARE_SIGNAL_IN(ref_acc,              dynamicgraph::Vector);
          DECLARE_SIGNAL_IN(ref_f,                dynamicgraph::Vector);
          DECLARE_SIGNAL_IN(f_min,                double);
          DECLARE_SIGNAL_IN(f_max,                double);

          DECLARE_SIGNAL_IN(mu,                   double);
          DECLARE_SIGNAL_IN(contact_normal,       dynamicgraph::Vector);

          DECLARE_SIGNAL_OUT(des_f,               dynamicgraph::Vector);
          DECLARE_SIGNAL_OUT(pos,                 dynamicgraph::Vector);
          DECLARE_SIGNAL_OUT(vel,                 dynamicgraph::Vector);
          DECLARE_SIGNAL_OUT(acc,                 dynamicgraph::Vector);
          DECLARE_SIGNAL_OUT(des_acc,             dynamicgraph::Vector);

        public:

          void init();
          const SignalArray<int>& signals();

          void before_solving(int iter, double time);
          void after_solving(int iter, const tsid::solvers::HQPOutput & sol);

        protected:
          void addContact(const double& transitionTime);
          void removeContact(const double& time, const double& transitionTime);

          tsid::contacts::ContactPoint* m_contact;
          tsid::tasks::TaskSE3Equality* m_taskMotion;

          tsid::trajectories::TrajectorySample m_sampleFoot;

          Eigen::Matrix<double,3,1> m_f_sol;

          bool m_init;

          enum ContactState
          {
            SUPPORT = 0,
            SUPPORT_TRANSITION = 1, // transition towards support
            FLYING = 2 // transition towards support
          };

          ContactState m_contactState;

          double            m_contactTransitionTime;  /// end time of the current contact transition (if any)

          SignalArray<int> m_signals;

          void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0);
        };

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTTSIDCONTROLLER_EXPORT TsidController
      : public::dynamicgraph::Entity
      {
        typedef TsidController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        TsidController( const std::string & name );

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(com_ref_pos,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(com_ref_vel,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(com_ref_acc,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_pos,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_vel,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_acc,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_pos,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_vel,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_acc,   dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(kp_base_orientation,        dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_base_orientation,        dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_com,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_com,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_feet,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_feet,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_posture,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_posture,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_pos,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_pos,                     dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(w_com,                      double);
        DECLARE_SIGNAL_IN(w_feet,                     double);
        DECLARE_SIGNAL_IN(w_posture,                  double);
        DECLARE_SIGNAL_IN(w_forces,                   double);
        DECLARE_SIGNAL_IN(weight_contact_forces,      dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(rotor_inertias,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(gear_ratios,                dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(q,                          dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(v,                          dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(tau_des,                   dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(dv_des,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(M,                         dynamicgraph::Matrix);
        DECLARE_SIGNAL_OUT(com,                       dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(com_vel,                   dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(com_acc,                   dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(com_acc_des,               dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(base_orientation,          dynamicgraph::Vector);

      public:

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void init(const double& dt);
        void updateComOffset();
        void addContact(const std::string& frame_name);
        void loadURDF(const std::string& urdf_filepath);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("["+name+"] "+msg, t, file, line);
        }

      protected:

        double            m_dt;               /// control loop time period
        double            m_t;
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_enabled;          /// True if controler is enabled
        bool              m_firstTime;        /// True at the first iteration of the controller
        bool              m_robot_loaded;

        int m_frame_id_rf;  /// frame id of right foot
        int m_frame_id_lf;  /// frame id of left foot

        /// tsid
        tsid::robots::RobotWrapper *               m_robot;
        tsid::solvers::SolverHQPBase *             m_hqpSolver;
        tsid::InverseDynamicsFormulationAccForce * m_invDyn;
        tsid::tasks::TaskComEquality *             m_taskCom;
        tsid::tasks::TaskJointPosture *            m_taskPosture;
        tsid::tasks::TaskJointPosture *            m_taskBlockedJoints;

        tsid::trajectories::TrajectorySample       m_sampleCom;
        tsid::trajectories::TrajectorySample       m_samplePosture;

        double m_w_com;
        double m_w_posture;

        tsid::math::Vector  m_dv;                  /// desired accelerations (urdf order)
        tsid::math::Vector  m_f;                   /// desired force coefficients (24d)
        tsid::math::Vector3 m_com_offset;          /// 3d CoM offset
        tsid::math::Vector  m_tau;
        tsid::math::Vector  m_q_urdf;
        tsid::math::Vector  m_v_urdf;

        typedef pinocchio::Data::Matrix6x Matrix6x;
        Matrix6x m_J_RF;
        Matrix6x m_J_LF;

        unsigned int m_timeLast;
        RobotUtil * m_robot_util;

        std::vector<TsidContactHelper*> m_contacts;

      }; // class TsidController
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_tsid_controller_H__
