#include <locomotion/LeggedRobot.h>
#include <locomotion/LocomotionPlannerHelpers.h>
#include <type_traits>
#include <utility>
#include <vector>
#include <cmath>

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

TestResult test_trajectory_base_integration_matches_speed_times_time() {
    TestResult res;

    auto robot = makeQuadruped();
    robot.setRootState(crl::P3D(0, 0.42, 0), crl::Quaternion::Identity());

    struct Case {
        double v_fwd;
        double v_side;
        double v_turn;
        double dt;
        double t_start;
        double t_end;
        double expected_disp_norm; // precomputed from reference implementation
        const char* name;
    };

    const std::vector<Case> cases = {
        {0.35, 0.12, 0.0, 0.01, 0.0, 1.0, 0.3663, "mixed-linear"},
        {0.20, 0.00, 0.0, 0.01, 0.0, 1.0, 0.1980, "forward-only"},
        {0.00, 0.20, 0.0, 0.01, 0.0, 1.0, 0.1980, "sideways-only"},
    };

    for (const auto& c : cases) {
        crl::bFrameReferenceMotionPlan plan(&robot);
        plan.targetForwardSpeed = c.v_fwd;
        plan.targetSidewaysSpeed = c.v_side;
        set_turning_speed_compat(plan, c.v_turn);
        plan.targetbFrameHeight = 0.42;
        plan.dt = c.dt;
        plan.tStart = c.t_start;
        plan.tEnd = c.t_end;
        plan.generateTrajectory();

        const auto s0 = plan.getBFrameStateAt(plan.tStart);
        const auto s1 = plan.getBFrameStateAt(plan.tEnd - plan.dt);
        const crl::V3D actual(s1[0] - s0[0], s1[1] - s0[1], s1[2] - s0[2]);

        const double err = std::abs(actual.norm() - c.expected_disp_norm);
        if (err > 3e-3) {
            res.passed = false;
            res.error +=
                std::string("Base trajectory integration mismatch in case '") +
                c.name + "'. (|norm(actual)-norm(expected)| = " +
                std::to_string(err) + ")\n";
        }
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_trajectory_base_integration_matches_speed_times_time);
    return (allTestsOk ? 0 : 1);
}
