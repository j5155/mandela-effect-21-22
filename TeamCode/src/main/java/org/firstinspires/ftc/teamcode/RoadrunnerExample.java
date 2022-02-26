package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Example made by James

@Autonomous(name="RoadrunnerExample", group="Roadrunner")
@Disabled
public class RoadrunnerExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0); // Change this to where the robot starts!

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(10) // Goes forward the specified number of inches+
                .turn(Math.toRadians(90)) // This turns the specified number of radians, use Math.toRadians(degrees) to turn in degrees like this
                .waitSeconds(3) // Waits the specified number of seconds without breaking anything
                .splineTo(new Vector2d(10, 10), Math.toRadians(0)) // This automatically gets the robot to the specified position using a spline curve
                .addDisplacementMarker(() -> { // Simple displacement markers run after the previous action, i.e. this one would run after the robot had finished turning
                    // Run your action in here!
                    // Drop servo, start motor, whatever
                })
                .addDisplacementMarker(20, () -> { // If you specify a displacement, the marker will run after the robot has driven a specified number of inches overall
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                })
                .addTemporalMarker(2, () -> { // Temporal markers run the specified number of seconds into the path, it doesn't matter where you place them
                    // Run whatever in here too
                })
                .addSpatialMarker(new Vector2d(20, 20), () -> { // Spatial markers run when the robot is closest to a specified coordinate
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate

                    // Run your action in here!
                })
                .setReversed(true) // This makes all actions after it run in reverse
                .setReversed(false) // the exact opposite of setReversed(true)
                // There are a few other commands that we can't use because tank drive, such as .strafeLeft and .splineToLinearHeading

                .build();
                // Copy paste everything after the line beginning with TrajectorySequence here to Meep Meep for visualization!

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
