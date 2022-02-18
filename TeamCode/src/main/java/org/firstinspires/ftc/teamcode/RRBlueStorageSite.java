package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="RRBlueStorageSite", group="Blue")
public class RRBlueStorageSite extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -62, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(20) // Align with Carousel Wheel
                .waitSeconds(1)
                .addDisplacementMarker(() -> { // Spin Carousel Wheel
                    // TODO: Put carousel wheel spinner here
                })
                .splineTo(new Vector2d(-12, 40), Math.toRadians(270)) // Go To Blue Shipping Hub
                .waitSeconds(1)
                .addDisplacementMarker(() -> { // Ship the hub
                    // TODO: Put the stuff to ship the hub here
                })
                .setReversed(true)
                .splineTo(new Vector2d(50, 40), Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

