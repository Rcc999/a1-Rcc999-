#include <locomotion/LeggedRobot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>

#include <vector>

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

TestResult test_fk_state_independent() {
    TestResult res;

    auto robot = makeQuadruped();
    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    // Set robot state to A (simulator state)
    robot.setRootState(crl::P3D(0.1, 0.42, -0.2), crl::Quaternion::Identity());

    // Prepare a different q (B) without syncing robot state
    crl::dVector q;
    gcrr.getQ(q);
    q[0] = 1.25;  // base x
    q[1] = 0.50;  // base y
    q[2] = -0.75; // base z
    q[5] = 0.4;   // base yaw
    gcrr.setQ(q);

    // FK using current robot state (should still depend on q, not robot state)
    std::vector<crl::P3D> p_before;
    for (auto limb : robot.limbs) {
        p_before.push_back(
            gcrr.getWorldCoordinates(limb->ee->endEffectorOffset, limb->eeRB));
    }

    // Now sync robot state to q and recompute
    gcrr.syncRobotStateWithGeneralizedCoordinates();

    for (size_t i = 0; i < robot.limbs.size(); ++i) {
        crl::P3D p_after = gcrr.getWorldCoordinates(
            robot.limbs[i]->ee->endEffectorOffset, robot.limbs[i]->eeRB);
        double diff = crl::V3D(p_after - p_before[i]).norm();
        if (diff > 1e-6) {
            res.passed = false;
            res.error +=
                "FK appears to depend on the simulator state. "
                "Make sure base pose is computed from q[0..5], not from "
                "robot->root->state.\n";
            break;
        }
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_fk_state_independent);
    return (allTestsOk ? 0 : 1);
}
