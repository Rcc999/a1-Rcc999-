#include <locomotion/LeggedRobot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>

#include "TestResult.h"

using namespace tests;

static crl::LeggedRobot makeQuadruped() {
    crl::LeggedRobot robot(CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs", nullptr,
                           false);
    robot.addLimb("fl", robot.getRBByName("tibia_0"));
    robot.addLimb("hl", robot.getRBByName("tibia_1"));
    robot.addLimb("fr", robot.getRBByName("tibia_2"));
    robot.addLimb("hr", robot.getRBByName("tibia_3"));
    return robot;
}

TestResult test_fd_jacobian_base_dofs_nonzero() {
    TestResult res;

    auto robot = makeQuadruped();
    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    // Use a non-trivial base pose
    robot.setRootState(crl::P3D(0.2, 0.42, -0.3),
                       crl::Quaternion(0.98, 0.0, 0.2, 0.0));

    const auto *limb = robot.limbs[0];
    crl::Matrix J;
    gcrr.estimate_linear_jacobian(limb->ee->endEffectorOffset, limb->eeRB, J);

    // Base translation DOFs should affect end-effector position.
    for (int c = 0; c < 3; ++c) {
        double col_norm = J.col(c).norm();
        if (col_norm < 1e-6) {
            res.passed = false;
            res.error +=
                "FD Jacobian base translation columns are near zero. "
                "Check that FK/Jacobian depend on base q[0..2].\n";
            break;
        }
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_fd_jacobian_base_dofs_nonzero);
    return (allTestsOk ? 0 : 1);
}
