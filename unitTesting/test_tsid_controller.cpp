#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <tsid/math/utils.hpp>
#include <tsid/robots/robot-wrapper.hpp>

#include "sot/torque_control/tsid-controller.hh"

using namespace tsid;
using namespace tsid::math;
using namespace tsid::robots;

using namespace std;
using namespace dynamicgraph::sot::torque_control;

const string package_dir = SOT_TORQUE_CONTROL_SOURCE_DIR"/unitTesting/models/quadruped/";
const string urdf_path = SOT_TORQUE_CONTROL_SOURCE_DIR"/unitTesting/models/quadruped/urdf/quadruped.urdf";

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_adding_contacts )
{
  TsidController controller("tsid_quadruped");
  controller.loadURDF(urdf_path);
  controller.addContact("FL_contact");

  // Check if a few signals were created.
  BOOST_CHECK(controller.hasSignal("FL_contact__f_max"));
  BOOST_CHECK(controller.hasSignal("FL_contact__f_min"));
  BOOST_CHECK(controller.hasSignal("FL_contact__vel"));

  // Other contact points do not exist yet.
  BOOST_CHECK(!controller.hasSignal("FR_contact__vel"));

  // Adding another contact.
  controller.addContact("FR_contact");

  // After adding the contact, the corresponding signal should now exist.
  BOOST_CHECK(controller.hasSignal("FR_contact__vel"));

  controller.displaySignalList(std::cout);
}

BOOST_AUTO_TEST_CASE ( test_basic_standing )
{
  vector<string> package_dirs;
  package_dirs.push_back(package_dir);
  RobotWrapper robot(urdf_path,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);

  Vector q = robot.model().neutralConfiguration;
  Vector v = Vector::Zero(robot.nv());
  const unsigned int nv = robot.nv();

  pinocchio::Data data(robot.model());

  // Create initial posture.
  q(0) = 0.1;
  q(2) = 0.5;
  q(6) = 1.;
  for (int i = 0; i < 4; i++) {
    q(7 + 2 * i) = -0.4;
    q(8 + 2 * i) = 0.8;
  }

  robot.computeAllTerms(data, q, v);

  // Place the robot onto the ground.
  pinocchio::SE3 fl_contact = robot.framePosition(data, robot.model().getFrameId("FL_contact"));
  q[2] -= fl_contact.translation()(2);

  std::cout << q.transpose() << std::endl;


  // Create constant signals.

}

BOOST_AUTO_TEST_SUITE_END ()
