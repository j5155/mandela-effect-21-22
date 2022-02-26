package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="RRBlueStorageSite", group="Blue")
public class RRBlueStorageSite extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        HardwareConfig robot   = new HardwareConfig();   // Use a Pushbot's hardware

        Pose2d startPose = new Pose2d(-5, 61, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //.forward(20) // Align with Carousel Wheel
                .splineTo(new Vector2d(-47, 60), Math.toRadians(180))
                .addDisplacementMarker(() -> { carouselWheel(0.5, 8.4);// Spin Carousel Wheel
                })
                .splineTo(new Vector2d(-12, 40), Math.toRadians(270)) // Go To Blue Shipping Hub
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    Arm(ARM_SPEED, 6);// Ship the hub
                })
                .setReversed(true)
                .splineTo(new Vector2d(50, 40), Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

    private void carouselWheel(double power, double howLong) {
        HardwareConfig robot   = new HardwareConfig();   // Use a Pushbot's hardware
        robot.init(hardwareMap);
        final ElapsedTime runtime = new ElapsedTime();
        robot.CarouselWheel.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < howLong)) {
            telemetry.addData("carousel", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.CarouselWheel.setPower(0);
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
