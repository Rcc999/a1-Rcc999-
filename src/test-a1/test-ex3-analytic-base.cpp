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

TestResult test_analytic_jacobian_base_matches_fd() {
    TestResult res;

    auto robot = makeQuadruped();
    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    // Non-trivial pose
    crl::dVector q;
    gcrr.getQ(q);
    q[0] = 0.3;  // x
    q[1] = 0.5;  // y
    q[2] = -0.4; // z
    q[3] = 0.1;  // roll
    q[4] = -0.2; // pitch
    q[5] = 0.3;  // yaw
    gcrr.setQ(q);
    gcrr.syncRobotStateWithGeneralizedCoordinates();

    const auto *limb = robot.limbs[0];

    crl::Matrix J_analytic;
    crl::Matrix J_fd;
    gcrr.compute_dpdq(limb->ee->endEffectorOffset, limb->eeRB, J_analytic);
    gcrr.estimate_linear_jacobian(limb->ee->endEffectorOffset, limb->eeRB, J_fd);

    double max_err = 0.0;
    for (int c = 0; c < 6; ++c) {
        max_err = std::max(max_err, (J_analytic.col(c) - J_fd.col(c)).norm());
    }

    if (max_err > 1e-4) {
        res.passed = false;
        res.error +=
            "Analytic Jacobian base columns do not match FD. "
            "Check handling of base DOFs (q[0..5]).\n";
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_analytic_jacobian_base_matches_fd);
    return (allTestsOk ? 0 : 1);
}
