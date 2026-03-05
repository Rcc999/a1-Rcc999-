#include <locomotion/LeggedRobot.h>
#include <locomotion/LocomotionPlannerHelpers.h>
#include <type_traits>
#include <utility>

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

template <typename T, typename = void>
struct has_target_turning_speed : std::false_type {};

template <typename T>
struct has_target_turning_speed<
    T, std::void_t<decltype(std::declval<T&>().targetTurningSpeed)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_target_turnging_speed : std::false_type {};

template <typename T>
struct has_target_turnging_speed<
    T, std::void_t<decltype(std::declval<T&>().targetTurngingSpeed)>>
    : std::true_type {};

template <typename PlanT>
static void set_turning_speed_compat(PlanT& plan, double value) {
    if constexpr (has_target_turning_speed<PlanT>::value) {
        plan.targetTurningSpeed = value;
    } else if constexpr (has_target_turnging_speed<PlanT>::value) {
        plan.targetTurngingSpeed = value;
    }
}

static crl::V3D get_displacement(crl::bFrameReferenceMotionPlan &plan) {
    plan.generateTrajectory();
    const double t_eval = plan.tEnd - plan.dt;
    auto s0 = plan.getBFrameStateAt(plan.tStart);
    auto s1 = plan.getBFrameStateAt(t_eval);
    return crl::V3D(s1[0] - s0[0], s1[1] - s0[1], s1[2] - s0[2]);
}

TestResult test_trajectory_includes_dt() {
    TestResult res;

    auto robot = makeQuadruped();
    robot.setRootState(crl::P3D(0, 0.42, 0), crl::Quaternion::Identity());

    crl::bFrameReferenceMotionPlan plan1(&robot);
    plan1.targetForwardSpeed = 0.25;
    plan1.targetSidewaysSpeed = 0.1;
    set_turning_speed_compat(plan1, 0.2);
    plan1.targetbFrameHeight = 0.42;
    plan1.dt = 0.01;
    plan1.tStart = 0.0;
    plan1.tEnd = 1.0;

    crl::bFrameReferenceMotionPlan plan2 = plan1;
    plan2.dt = 0.02;

    crl::V3D d1 = get_displacement(plan1);
    crl::V3D d2 = get_displacement(plan2);

    double n1 = d1.norm();
    double n2 = d2.norm();
    if (n1 < 1e-8 || n2 < 1e-8) {
        res.passed = false;
        res.error += "Trajectory displacement is near zero. Check integration.\n";
        return res;
    }

    double ratio = n2 / n1;
    if (ratio < 0.8 || ratio > 1.25) {
        res.passed = false;
        res.error +=
            "Trajectory scaling looks off. You may be missing dt in integration. "
            "(Displacement ratio for dt=0.02 vs 0.01 was " +
            std::to_string(ratio) + ")\n";
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_trajectory_includes_dt);
    return (allTestsOk ? 0 : 1);
}
