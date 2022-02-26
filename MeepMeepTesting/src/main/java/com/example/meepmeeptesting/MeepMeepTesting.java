package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
// TODO: Redo ALL the autonomous in RoadRunner, after the bot is tuned
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.73611498784813, 26.6123752687, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 62, Math.toRadians(270))) // COPY PASTE YOUR CODE AFTER THIS LINE, edit this line to the start pos of robot
                                .forward(18) // Goes forward the specified number of inches
                                //.turn(Math.toRadians(90)) // This turns the specified number of radians, use Math.toRadians(degrees) to turn in degrees like this
                                //.waitSeconds(3) // Waits the specified number of seconds without breaking anything
                                .splineTo(new Vector2d(45, 44), Math.toRadians(0)) // This automatically gets the robot to the specified position using a spline curve
                                .splineTo(new Vector2d(46, 17), Math.toRadians(270)) // This automatically gets the robot to the specified position using a spline curve
                                .back(6)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}