package org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Swerb506.core.RobotHardware;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.controllers.PPHolonomicDriveController;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path.PathPlannerTrajectory;
import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.util.PIDConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

@Autonomous(name="Pathplanner Test")
public class Automus extends RobotHardware {
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController controller;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private Consumer<ChassisSpeeds> output;

    boolean hasRun = false;
    boolean finishedDriving = false;
    ElapsedTimer elapsedTimer = new ElapsedTimer();

    @Override
    public void init() {
        super.init();

        controller = new PPHolonomicDriveController(
                new PIDConstants(6.0, 0.0, 0.0),
                new PIDConstants(8.0, 0.0, 0.0),
                0.017,
                1.6,
                0.2507
        );

        poseSupplier = swerveDrive::getPose;
        speedsSupplier = swerveDrive::getRobotVelocity;
        output = swerveDrive::drive;

        ChassisSpeeds currentSpeeds = speedsSupplier.get();
        path = PathPlannerPath.fromPathFile("Forward Test");

        trajectory = new PathPlannerTrajectory(path, currentSpeeds);
        controller.reset(poseSupplier.get(), currentSpeeds);
        elapsedTimer.reset();
    }

    @Override
    public void loop() {
        super.loop();

        if(finishedDriving) {
            telemetry.addLine("Finished Driving");
            return;
        }

        if (!hasRun) {
            elapsedTimer.reset();
            hasRun = true;
        }

        double currentTime = elapsedTimer.seconds();
        PathPlannerTrajectory.State targetState = trajectory.sample(currentTime);
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
        output.accept(targetSpeeds);

        if (currentTime > trajectory.getTotalTimeSeconds()) {
            finishedDriving = true;
            output.accept(new ChassisSpeeds(0,0,0));
        }
    }
}