/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *                 Julian Viereck, NYU, MPI Tübingen
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

#define EIGEN_RUNTIME_NO_MALLOC

#include <sot/torque_control/tsid-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>

#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/math/utils.hpp>

#include <boost/test/unit_test.hpp>

#define ZERO_FORCE_THRESHOLD 1e-3

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace tsid;
      using namespace tsid::trajectories;
      using namespace tsid::math;
      using namespace tsid::contacts;
      using namespace tsid::tasks;
      using namespace tsid::solvers;

      #define EntityClassName TsidContactHelper

      TsidContactHelper::
        TsidContactHelper(const std::string& frame_name,
                          TsidController& controller,
                          tsid::robots::RobotWrapper& robot,
                          tsid::InverseDynamicsFormulationAccForce& invDyn)
          : m_controller(controller)
          , m_invDyn(invDyn)
          , m_robot(robot)
          , m_frameName(frame_name)
          , m_frameId(robot.model().getFrameId(frame_name))
          , m_init(false)
          , CONSTRUCT_SIGNAL_IN(ref_pos, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(ref_vel, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(ref_acc, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(ref_f, dynamicgraph::Vector)
          , CONSTRUCT_SIGNAL_IN(f_min, double)
          , CONSTRUCT_SIGNAL_IN(f_max, double)

          , CONSTRUCT_SIGNAL_OUT(des_f, dynamicgraph::Vector, controller.m_tau_desSOUT)
          , CONSTRUCT_SIGNAL_OUT(des_acc, dynamicgraph::Vector, controller.m_tau_desSOUT)

          , CONSTRUCT_SIGNAL_OUT(pos,   dynamicgraph::Vector, controller.m_tau_desSOUT)
          , CONSTRUCT_SIGNAL_OUT(vel,   dynamicgraph::Vector, controller.m_tau_desSOUT)
          , CONSTRUCT_SIGNAL_OUT(acc,   dynamicgraph::Vector, controller.m_tau_desSOUT)

          , m_contactState(SUPPORT)
      {
        assert(m_robot.model()->existFrame(frame_name));
      }

      void TsidContactHelper::init()
      {
        m_contact = new ContactPoint("contact_" + m_frameName, m_robot, m_frameName,
                          m_controller.m_contact_normalSIN(0),
                          m_controller.m_muSIN(0),
                          m_f_minSIN(0),
                          m_f_maxSIN(0),
                          m_controller.m_w_forcesSIN(0));
        m_contact->Kp(m_controller.m_kp_constraintsSIN(0));
        m_contact->Kd(m_controller.m_kd_constraintsSIN(0));
        m_invDyn.addRigidContact(*m_contact);

        m_taskMotion = new TaskSE3Equality("task_" + m_frameName, m_robot, m_frameName);
        m_taskMotion->Kp(m_controller.m_kp_feetSIN(0));
        m_taskMotion->Kp(m_controller.m_kd_feetSIN(0));

        m_init = true;
      }

      const SignalArray<int>& TsidContactHelper::signals()
      {
        return m_ref_posSIN << m_ref_velSIN << m_ref_accSIN <<
            m_ref_fSIN << m_f_minSIN << m_f_maxSIN <<
            m_des_fSOUT << m_posSOUT << m_velSOUT << m_accSOUT << m_des_accSOUT;
      }

      void TsidContactHelper::addContact(const double& transitionTime)
      {
        if(m_contactState == FLYING)
        {
          SEND_MSG("Add contact " + m_contact->name() + " after transition time " + toString(transitionTime) + " s.", MSG_TYPE_INFO);
          m_invDyn.addRigidContact(*m_contact);
          m_invDyn.removeTask(m_taskMotion->name(), transitionTime);
          m_contactState = SUPPORT;

          pinocchio::SE3 ref = m_robot.position(m_invDyn.data(), m_robot.model().getJointId(m_frameName));
          m_contact->setReference(ref);
          SEND_MSG("Setting left foot reference to " + toString(ref), MSG_TYPE_DEBUG);
        }
      }

      void TsidContactHelper::removeContact(const double& time, const double& transitionTime)
      {
        if (m_contactState == FLYING) return;

        bool res = m_invDyn.removeRigidContact(m_contact->name(), transitionTime);
        if(!res)
        {
          SEND_MSG("Error while remove foot contact: " + m_contact->name(), MSG_TYPE_ERROR);
        }
        const double & w_feet = m_controller.m_w_feetSIN.accessCopy();
        m_invDyn.addMotionTask(*m_taskMotion, w_feet, 1);
        if(transitionTime > 0.) {
          m_contactState = SUPPORT_TRANSITION;
          m_contactTransitionTime = time + transitionTime;
        } else {
          m_contactState = FLYING;
        }
      }

      /**
       * @brief Consumes the input signals by updating the corresponding values on the
       * contact object.
       */
        //
      void TsidContactHelper::before_solving(int iter, double time)
      {
        m_contact->setRegularizationTaskWeightVector(m_controller.m_weight_contact_forcesSIN(iter));

        // use reference contact wrenches (if plugged) to determine contact phase
        if(m_ref_fSIN.isPlugged())
        {
          const Vector3 & f_ref  = m_ref_fSIN(iter);
          m_contact->setForceReference(f_ref);

          if (m_contactState == SUPPORT) {
            if (f_ref.norm() < ZERO_FORCE_THRESHOLD) {
              removeContact(time, 0.0);
            }
          } else if (m_contactState == FLYING && f_ref.norm() > ZERO_FORCE_THRESHOLD) {
            addContact(0.);
          }
        }

        if (m_contactTransitionTime >= time && m_contactState == SUPPORT_TRANSITION) {
          m_contactState = FLYING;
        }

        if (m_contactState != SUPPORT)
        {
          const Eigen::Matrix<double,3,1>& x_rf_ref = m_ref_posSIN(iter);
          const Vector6& dx_rf_ref =  m_ref_velSIN(iter);
          const Vector6& ddx_rf_ref = m_ref_accSIN(iter);
          m_sampleFoot.pos = x_rf_ref;
          m_sampleFoot.vel = dx_rf_ref;
          m_sampleFoot.acc = ddx_rf_ref;
          m_taskMotion->setReference(m_sampleFoot);
          m_taskMotion->Kp(m_controller.m_kp_feetSIN(iter));
          m_taskMotion->Kp(m_controller.m_kd_feetSIN(iter));
        }
      }

      void TsidContactHelper::after_solving(int iter, const HQPOutput & sol)
      {
        if (!m_invDyn.getContactForces(m_contact->name(), sol, m_f_sol)) {
          m_f_sol.setZero();
        }
      }

      DEFINE_SIGNAL_OUT_FUNCTION(des_f, dynamicgraph::Vector)
      {
        if (!m_init) {
          SEND_MSG("Not possible to compute signal before initialization", MSG_TYPE_ERROR);
          return s;
        }

        m_controller.m_tau_desSOUT(iter);
        s = m_f_sol;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(des_acc, dynamicgraph::Vector)
      {
        if (!m_init) {
          SEND_MSG("Not possible to compute signal before initialization", MSG_TYPE_ERROR);
          return s;
        }

        m_controller.m_tau_desSOUT(iter);
        if (m_contactState == SUPPORT) {
          s = m_contact->getMotionTask().getDesiredAcceleration();
        } else {
          s = m_taskMotion->getDesiredAcceleration();
        }
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pos, dynamicgraph::Vector)
      {
        if(!m_init)
        {
          SEND_MSG("Not possible to compute signal before initialization", MSG_TYPE_ERROR);
          return s;
        }

        m_controller.m_tau_desSOUT(iter);
        pinocchio::SE3 oMi;
        s.resize(12);
        m_robot.framePosition(m_invDyn.data(), m_frameId, oMi);
	      tsid::math::SE3ToVector(oMi, s);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(vel, dynamicgraph::Vector)
      {
        if(!m_init)
        {
          SEND_MSG("Not possible to compute signal before initialization", MSG_TYPE_ERROR);
          return s;
        }

        m_controller.m_tau_desSOUT(iter);

        pinocchio::Motion v;
        m_robot.frameVelocity(m_invDyn.data(), m_frameId, v);
        s = v.toVector();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(acc, dynamicgraph::Vector)
      {
        if(!m_init)
        {
          SEND_MSG("Not possible to compute signal before initialization", MSG_TYPE_ERROR);
          return s;
        }

        m_controller.m_tau_desSOUT(iter);

        pinocchio::Motion a;
        m_robot.frameAcceleration(m_invDyn.data(), m_frameId, a);
        s = a.toVector();
        return s;
      }

#define INPUT_SIGNALS m_com_ref_posSIN \
  << m_com_ref_velSIN \
  << m_com_ref_accSIN \
  << m_posture_ref_posSIN \
  << m_posture_ref_velSIN \
  << m_posture_ref_accSIN \
  << m_base_orientation_ref_posSIN \
  << m_base_orientation_ref_velSIN \
  << m_base_orientation_ref_accSIN \
  << m_kp_base_orientationSIN \
  << m_kd_base_orientationSIN \
  << m_kp_constraintsSIN \
  << m_kd_constraintsSIN \
  << m_kp_comSIN \
  << m_kd_comSIN \
  << m_kp_feetSIN \
  << m_kd_feetSIN \
  << m_kp_postureSIN \
  << m_kd_postureSIN \
  << m_kp_posSIN \
  << m_kd_posSIN \
  << m_w_comSIN \
  << m_w_feetSIN \
  << m_w_postureSIN \
  << m_w_forcesSIN \
  << m_weight_contact_forcesSIN \
  << m_rotor_inertiasSIN \
  << m_gear_ratiosSIN \
  << m_qSIN \
  << m_vSIN \

#define OUTPUT_SIGNALS        m_tau_desSOUT \
  << m_comSOUT \
  << m_com_velSOUT \
  << m_com_accSOUT \
  << m_com_acc_desSOUT \
  << m_base_orientationSOUT \
  << m_dv_desSOUT \
  << m_MSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      #define EntityClassName TsidController

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TsidController,
                                         "TsidController");

      /**
       * TsidController
       */
      TsidController::
        TsidController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(com_ref_pos,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_vel,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_acc,                 dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_pos,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_vel,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_acc,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_pos,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_vel,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_acc,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_base_orientation,         dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_base_orientation,         dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_constraints,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_constraints,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_com,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_com,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_contact,                  dynamicgraph::Vector)
    	      ,CONSTRUCT_SIGNAL_IN(kd_contact,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_feet,                     dynamicgraph::Vector)
    	      ,CONSTRUCT_SIGNAL_IN(kd_feet,                     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_posture,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_posture,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_pos,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_pos,                      dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_com,                       double)
            ,CONSTRUCT_SIGNAL_IN(w_feet,                      double)
            ,CONSTRUCT_SIGNAL_IN(w_posture,                   double)
            ,CONSTRUCT_SIGNAL_IN(w_forces,                    double)
            ,CONSTRUCT_SIGNAL_IN(mu,                          double)
            ,CONSTRUCT_SIGNAL_IN(f_min,                       double)
            ,CONSTRUCT_SIGNAL_IN(contact_normal,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(rotor_inertias,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(gear_ratios,                 dynamicgraph::Vector)

            ,CONSTRUCT_SIGNAL_OUT(dv_des,                     dg::Vector, m_tau_desSOUT)

            ,CONSTRUCT_SIGNAL_IN(q,                           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(v,                           dynamicgraph::Vector)

            ,CONSTRUCT_SIGNAL_IN(wrench_base,                 dynamicgraph::Vector)

            ,CONSTRUCT_SIGNAL_OUT(tau_des,                    dynamicgraph::Vector, INPUT_SIGNALS)

            ,CONSTRUCT_SIGNAL_OUT(M,                          dg::Matrix, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com,                        dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_vel,                    dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_acc,                    dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(com_acc_des,                dg::Vector, m_tau_desSOUT)
            ,CONSTRUCT_SIGNAL_OUT(base_orientation,           dg::Vector, m_tau_desSOUT)

            ,m_initSucceeded(false)
            ,m_robot_loaded(false)
            ,m_enabled(false)
            ,m_t(0.0)
            ,m_firstTime(true)
            ,m_timeLast(0)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        m_com_offset.setZero();

        addCommand("laodURDF",
                   makeCommandVoid1(*this, &TsidController::loadURDF,
                                    docCommandVoid1("Loads the URDF model.",
                                                    "Filepath to the URDF file (string)")));

        addCommand("init",
                   makeCommandVoid2(*this, &TsidController::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)")));

        addCommand("updateComOffset",
                   makeCommandVoid0(*this, &TsidController::updateComOffset,
                                    docCommandVoid0("Update the offset on the CoM based on the CoP measurement.")));

        addCommand("addContact",
                  makeCommandVoid1(*this, &TsidController::addContact,
                                   dynamicgraph::command::docCommandVoid1(
                                      "Add a contact point to the formulation.",
                                      "Name of frame for the contact")));
      }

      void TsidController::updateComOffset()
      {
        // const Vector3 & com = m_robot->com(m_invDyn->data());
        // m_com_offset = m_zmp - com;
        // m_com_offset(2) = 0.0;
        SEND_MSG("CoM offset updated: "+toString(m_com_offset), MSG_TYPE_INFO);
      }

      void TsidController::loadURDF(const std::string& urdf_filepath)
      {
        vector<string> package_dirs;
        m_robot = new robots::RobotWrapper(urdf_filepath, package_dirs,
           se3::JointModelFreeFlyer());

        m_invDyn = new InverseDynamicsFormulationAccForce("invdyn", *m_robot);

        assert(m_robot->nv()>=6);
        m_robot_loaded = true;
      }

      void TsidController::addContact(const std::string& frame_name)
      {
        (!m_robot_loaded) {
          SEND_MSG("Need to load the URDF model first.", MSG_TYPE_ERROR);
        }

        TsidContactHelper* ch = new TsidContactHelper(frame_name, this, m_robot, m_invDyn);
        m_contacts.push_back(ch);
        Entity::signalRegistration(ch.signlas());
      }

      void TsidController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);

        if (!m_robot_loaded) {
          return SEND_MSG("Init failed: Need to load robot URDF first.", MSG_TYPE_ERROR);
        }

        const Eigen::Vector3d& contactNormal = m_contact_normalSIN(0);;
        const Eigen::Vector3d& kp_com = m_kp_comSIN(0);
        const Eigen::Vector3d& kd_com = m_kd_comSIN(0);
        const Eigen::Vector6d& kp_feet = m_kp_feetSIN(0);
        const Eigen::Vector6d& kd_feet = m_kd_feetSIN(0);
        const VectorN& kp_posture = m_kp_postureSIN(0);
        const VectorN& kd_posture = m_kd_postureSIN(0);

        const int nj = m_robot->nv() - 6;

        assert(kp_posture.size()==nj);
        assert(kd_posture.size()==nj);
        assert(rotor_inertias.size()==nj);
        assert(gear_ratios.size()==nj);

        m_w_com = m_w_comSIN(0);
        const double & w_forces = m_w_forcesSIN(0);

        try
        {
          if (m_rotor_inertiasSIN.isPlugged() && m_gear_ratiosSIN.isPlugged()) {
            m_robot->rotor_inertias(m_rotor_inertiasSIN(0));
            m_robot->gear_ratios(m_gear_ratiosSIN(0));
          }

          m_taskCom = new TaskComEquality("task-com", *m_robot);
          m_taskCom->Kp(kp_com);
          m_taskCom->Kd(kd_com);
          m_invDyn->addMotionTask(*m_taskCom, m_w_com, 1);

          m_sampleCom = TrajectorySample(3);

          if (m_w_postureSIN.isPlugged()) {
            m_w_posture = m_w_postureSIN(0);
            m_taskPosture = new TaskJointPosture("task-posture", *m_robot);
            m_taskPosture->Kp(kp_posture);
            m_taskPosture->Kd(kd_posture);
            m_invDyn->addMotionTask(*m_taskPosture, m_w_posture, 1);

            m_samplePosture = TrajectorySample(m_robot->nv()-6);
          }

          for (auto &contact : m_contacts) {
            contact.init();
          }

          m_hqpSolver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                          "eiquadprog-fast");
          m_hqpSolver->resize(m_invDyn->nVar(), m_invDyn->nEq(), m_invDyn->nIn());
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          return SEND_MSG("Init failed.", MSG_TYPE_ERROR);
        }
        m_dt = dt;
        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(tau_des, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tau_des before initialization!");
          return s;
        }
        const int nj = m_robot->nv() - 6;
        if(s.size() != nj)
          s.resize(nj);

        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);


        getProfiler().start(PROFILE_READ_INPUT_SIGNALS_AND_PREPARE_INVDYN);

        // COM
        const Vector3& x_com_ref =   m_com_ref_posSIN(iter);
        const Vector3& dx_com_ref =  m_com_ref_velSIN(iter);
        const Vector3& ddx_com_ref = m_com_ref_accSIN(iter);
        const double & w_com = m_w_comSIN(iter);

        m_sampleCom.pos = x_com_ref - m_com_offset;
        m_sampleCom.vel = dx_com_ref;
        m_sampleCom.acc = ddx_com_ref;
        m_taskCom->setReference(m_sampleCom);
        m_taskCom->Kp(kp_com);
        m_taskCom->Kd(kd_com);
        if(m_w_com != w_com)
        {
          m_w_com = w_com;
          m_invDyn->updateTaskWeight(m_taskCom->name(), w_com);
        }

        // Posture
        if (m_w_postureSIN.isPlugged()) {
          const double & w_posture = m_w_postureSIN(iter);
          const VectorN& kp_posture = m_kp_postureSIN(iter);
          const VectorN& kd_posture = m_kd_postureSIN(iter);

          m_samplePosture.pos = m_posture_ref_posSIN(iter);
          m_samplePosture.vel = m_posture_ref_velSIN(iter);
          m_samplePosture.acc = m_posture_ref_accSIN(iter);

          assert(kp_posture.size() == nj);
          assert(kd_posture.size() == nj);
          assert(m_samplePosture.pos.size() == nj);
          assert(m_samplePosture.vel.size() == nj);
          assert(m_samplePosture.acc.size() == nj);

          m_taskPosture->setReference(m_samplePosture);
          m_taskPosture->Kp(kp_posture);
          m_taskPosture->Kd(kd_posture);
          if(m_w_posture != w_posture)
          {
            m_w_posture = w_posture;
            m_invDyn->updateTaskWeight(m_taskPosture->name(), w_posture);
          }
        }

        m_q = m_qSIN(iter);
        m_v = m_vSIN(iter);

        assert(m_q.size() == m_robot->nq());
        assert(m_v.size() == m_robot->nv());

        // Compute all terms, such that they are available when computing
        // frame positions during the `contact.before_solving(iter)`.
        m_robot->computeAllTerms(m_invDyn->data(), m_q, n_v);

        // Contacts
        for (auto &contact : m_contacts) {
          contact.before_solving(iter);
        }

        m_invDyn->computeProblemData(m_t, m_q, m_v);

        assert(m_q.size() == m_robot->nq());
        assert(m_v.size() == m_robot->nv());

        if(m_timeLast != iter-1)
        {
          SEND_MSG("Last time "+toString(m_timeLast)+" is not current time-1: "+toString(iter), MSG_TYPE_ERROR);
          if(m_timeLast == iter)
          {
            s = m_tau_sot;
            return s;
          }
        }
        m_timeLast = iter;

        const HQPData & hqpData = m_invDyn->computeProblemData(m_t, m_q_urdf, m_v_urdf);
        getProfiler().stop(PROFILE_READ_INPUT_SIGNALS_AND_PREPARE_INVDYN);

        getProfiler().start(PROFILE_HQP_SOLUTION);

        // TODO: Allocate solvers for different problem sizes and switch
        // between the solvers given the current problem.
        SolverHQPBase * solver = m_hqpSolver;
        const HQPOutput & sol = solver->solve(hqpData);
        getProfiler().stop(PROFILE_HQP_SOLUTION);

        if(sol.status!=HQP_STATUS_OPTIMAL)
        {
          SEND_ERROR_STREAM_MSG("HQP solver failed to find a solution: "+toString(sol.status));
          SEND_DEBUG_STREAM_MSG(tsid::solvers::HQPDataToString(hqpData, false));
          SEND_DEBUG_STREAM_MSG("q="+toString(q_sot.transpose(),1,5));
          SEND_DEBUG_STREAM_MSG("v="+toString(v_sot.transpose(),1,5));
          s.setZero();
          return s;
        }

        getStatistics().store("active inequalities", sol.activeSet.size());
        getStatistics().store("solver iterations", sol.iterations);
        if(ddx_com_ref.norm()>1e-3)
          getStatistics().store("com ff ratio", ddx_com_ref.norm()/m_taskCom->getConstraint().vector().norm());

        m_dv_urdf = m_invDyn->getAccelerations(sol);
        m_robot_util->velocity_urdf_to_sot(m_q_urdf, m_dv_urdf, m_dv_sot);
        Eigen::Matrix<double,12,1> tmp;



        if(m_invDyn->getContactForces(m_contactRF->name(), sol, tmp))
          m_f_RF = m_contactRF->getForceGeneratorMatrix() * tmp;
        if(m_invDyn->getContactForces(m_contactLF->name(), sol, tmp))
          m_f_LF = m_contactLF->getForceGeneratorMatrix() * tmp;
        m_robot_util->joints_urdf_to_sot(m_invDyn->getActuatorForces(sol), m_tau_sot);

        m_tau_sot += kp_pos.cwiseProduct(q_ref-q_sot.tail(m_robot_util->m_nbJoints)) +
                     kd_pos.cwiseProduct(dq_ref-v_sot.tail(m_robot_util->m_nbJoints));

        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
        m_t += m_dt;

        s = m_tau_sot;

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(M,dynamicgraph::Matrix)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal M before initialization!");
          return s;
        }
        if(s.cols()!=m_robot->nv() || s.rows()!=m_robot->nv())
          s.resize(m_robot->nv(), m_robot->nv());
        m_tau_desSOUT(iter);
        s = m_robot->mass(m_invDyn->data());
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dv_des,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dv_des before initialization!");
          return s;
        }
        if(s.size()!=m_robot->nv())
          s.resize(m_robot->nv());
        m_tau_desSOUT(iter);
        s = m_dv_sot;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(f_des_right_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_right_foot before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);
        m_tau_desSOUT(iter);
        if(m_contactState == LEFT_SUPPORT)
        {
          s.setZero();
          return s;
        }
        s = m_f_RF;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(f_des_left_foot,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal f_des_left_foot before initialization!");
          return s;
        }
        if(s.size()!=6)
          s.resize(6);
        m_tau_desSOUT(iter);
        if(m_contactState == RIGHT_SUPPORT)
        {
          s.setZero();
          return s;
        }
        s = m_f_LF;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_acc_des, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc_des before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        m_tau_desSOUT(iter);
        s = m_taskCom->getDesiredAcceleration();
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_acc, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_acc before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        m_tau_desSOUT(iter);
        s = m_taskCom->getAcceleration(m_dv_urdf);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        const Vector3 & com = m_robot->com(m_invDyn->data());
        s = com + m_com_offset;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(com_vel,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com_vel before initialization!");
          return s;
        }
        if(s.size()!=3)
          s.resize(3);
        const Vector3 & com_vel = m_robot->com_vel(m_invDyn->data());
        s = com_vel;
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(base_orientation,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal base_orientation before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void TsidController::display(std::ostream& os) const
      {
        os << "InverseDynamicsBalanceController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
          getStatistics().report_all(1, os);
          os<<"QP size: nVar "<<m_invDyn->nVar()<<" nEq "<<m_invDyn->nEq()<<" nIn "<<m_invDyn->nIn()<<"\n";
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph



    }
  }
}