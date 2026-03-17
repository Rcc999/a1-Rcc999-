#pragma once

#include <locomotion/LocomotionController.h>
#include <locomotion/LocomotionTrajectoryPlanner.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>

namespace crl {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    LeggedRobot *robot;
    IK_Solver *ikSolver = nullptr;
    std::vector<dVector> trajectory;

    // Terrain obstacles for Ex.4/5.
    // Students can add spheres here, e.g. {P3D(0, -11.5, 0), 12}.
    std::vector<std::pair<P3D, double>> spheres = {{P3D(0, -11.5, 0), 12}};

public:
    /**
     * constructor
     */
    KinematicTrackingController(LocomotionTrajectoryPlanner *planner)
        : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = new IK_Solver(robot);
        trajectory.clear();
    }

    /**
     * destructor
     */
    virtual ~KinematicTrackingController(void) { delete ikSolver; }

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt, bool recordTrajectory) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos =
            planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        targetPos.y += getGroundHeight(targetPos.x, targetPos.z);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(
            planner->getSimTime() + dt);

        robot->setRootState(targetPos, targetOrientation);

        // now we solve inverse kinematics for each limbs
        for (uint i = 0; i < robot->limbs.size(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(
                robot->limbs[i], planner->getSimTime() + dt);
            target.y += getGroundHeight(target.x, target.z);

            ikSolver->addEndEffectorTarget(
                robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
                target);
        }

        dVector q;
        ikSolver->solve(q);
        if (recordTrajectory) {
            trajectory.push_back(q);
        }
    }

    void advanceInTime(double dt) override { planner->advanceInTime(dt); }

    void resetRecordedTrajectory() { trajectory.clear(); }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }

    double getGroundHeight(double x, double z) {
        double h = 0.0;
        for (auto &sphere : spheres) {
            double dx = x - sphere.first.x;
            double dz = z - sphere.first.z;
            double dist_sqr = dx * dx + dz * dz;
            double r = sphere.second;
            if (dist_sqr <= r * r) {
                double height = sphere.first.y + sqrt(r * r - dist_sqr);
                h = std::max(h, height);
            }
        }
        return h;
    }

    void drawBump(gui::Shader *shader) {
        for (auto &sphere : spheres) {
            drawSphere(sphere.first, sphere.second, *shader, V3D(0, 0, 0));
        }
    }
};

}  // namespace crl
