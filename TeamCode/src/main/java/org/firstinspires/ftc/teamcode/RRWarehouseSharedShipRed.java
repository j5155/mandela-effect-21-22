package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Example made by James

@Autonomous(name="RRWarehouseSharedShipRed", group="Roadrunner")
public class RRWarehouseSharedShipRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(15, -62, 90); // Change this to where the robot starts!

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(17) // Goes forward the specified number of inches
                .splineTo(new Vector2d(45, 44), Math.toRadians(180)) // This automatically gets the robot to the specified position using a spline curve
                .splineTo(new Vector2d(46, 17), Math.toRadians(270)) // This automatically gets the robot to the specified position using a spline curve
                .addDisplacementMarker(() -> {
                    Arm(ARM_SPEED, 6);// Ship the hub
// Simple displacement markers run after the previous action, i.e. this one would run after the robot had finished turning
                    // Run your action in here!
                    // Drop servo, start motor, whatever
                })
                .back(10)

                // There are a few other commands that we can't use because tank drive, such as .strafeLeft and .splineToLinearHeading

                .build();
        // Copy paste everything after the line beginning with TrajectorySequence here to Meep Meep for visualization!

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
    void Arm(double power, double howLong) {
        HardwareConfig robot   = new HardwareConfig();   // Use a Pushbot's hardware
        robot.init(hardwareMap);
        final ElapsedTime runtime = new ElapsedTime();
        robot.Arm.setPower(+power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()) < howLong) {
            telemetry.addData("Arm:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    static final double     ARM_SPEED  = 0.3;

}


