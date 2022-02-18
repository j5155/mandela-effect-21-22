/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// Everything is switched in this code because the front and back of the robot were switched.

@Autonomous(name="Red Storage Site", group="Red Storage")
//@Disabled
public class RedStorageSite extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareConfig robot   = new HardwareConfig();   // Use a Pushbot's hardware
    private final ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.65;
    static final double     TURN_SPEED    = 0.4;
    static final double     ARM_SPEED  = 0.3;
    static final double     CLAW_TIME = 1.0;
    static final double     CLAW_SPEED = 0.2;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        turnLeft(TURN_SPEED, 0.3);//was 0.5
        allStop(100);
        straight(-FORWARD_SPEED, .5);
        allStop(100);
        // spin carousel for 5 secs
        carouselWheel(robot.CAROUSEL_SPEED, 5.0);
        allStop(100);
        // drive forwards for a sec
        straight(FORWARD_SPEED,1.4); //was 2.0
        allStop(200);
        //turn right to be straight
        turnRight(TURN_SPEED, .2);
        allStop(100);
        //line up with station
        straight(FORWARD_SPEED, .4);
        allStop(100);
        //put arm up
        Arm(ARM_SPEED, 3);
        allStop(100);
        //turn left to drop
        turnLeft(TURN_SPEED, .5);
        allStop(100);
        //drop particle
        Claw(CLAW_SPEED, 0.2);
        allStop(100);
        //turn right to face warehouse
        turnRight(TURN_SPEED, .5);
        allStop(100);
        // Step 3:  Drive forwards to warehouse
        straight(FORWARD_SPEED,3.0 ); // was 6

        // Step 4:  Stop and close the claw.
        allStop(500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    void straight(double power, double howLong){
        robot.LeftFront.setPower(power);
        robot.LeftBack.setPower(power);
        robot.RightBack.setPower(power);
        robot.RightFront.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < howLong)) {
            telemetry.addData("straight", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    void turnLeft(double power, double howLong){
        robot.LeftFront.setPower(-power);
        robot.LeftBack.setPower(-power);
        robot.RightBack.setPower(+power);
        robot.RightFront.setPower(+power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < howLong)) {
            telemetry.addData("turnLeft", runtime.seconds());
            telemetry.update();
        }
    }
    void allStop(int pause){
        robot.LeftFront.setPower(0);
        robot.LeftBack.setPower(0);
        robot.RightBack.setPower(0);
        robot.RightFront.setPower(0);
        sleep(pause);
    }
    void carouselWheel(double power, double howLong){
        robot.CarouselWheel.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < howLong)) {
            telemetry.addData("carousel", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.CarouselWheel.setPower(0);
    }
    void turnRight(double power, double howLong){
        robot.LeftFront.setPower(+power);
        robot.LeftBack.setPower(+power);
        robot.RightBack.setPower(-power);
        robot.RightFront.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < howLong)) {
            telemetry.addData("turnLeft", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    void Arm(double power, double howLong) {
        robot.Arm.setPower(+power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()) < howLong) {
            telemetry.addData("Arm:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    void Claw(double power, double howLong) {
        robot.Arm.setPower(0);
        robot.ClawLeft.setPower(CLAW_SPEED);
        robot.ClawRight.setPower(CLAW_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()) <CLAW_TIME) {
            telemetry.addData("Arm:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.ClawLeft.setPower(0);
        robot.ClawRight.setPower(0);
    }

}
