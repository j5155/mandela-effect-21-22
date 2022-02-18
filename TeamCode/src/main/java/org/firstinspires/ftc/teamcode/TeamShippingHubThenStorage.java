package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="TeamShippingHubThenStorage", group="Red")
@Disabled
public class TeamShippingHubThenStorage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -62, 0); // Change this to where the robot starts!

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(20) // Align with Carousel Wheel
                .waitSeconds(1)
                .addDisplacementMarker(() -> { // Todo: Carousel the Wheel.
                })
                .splineTo(new Vector2d(-12, -40), Math.toRadians(90))
                .waitSeconds(1)
                .addDisplacementMarker(() -> { // Ship the hub
                    // TODO: Put the stuff to ship the hub here
                })
                .setReversed(true)
                .splineTo(new Vector2d(-60, -34), Math.toRadians(180))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
