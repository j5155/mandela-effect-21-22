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
                .setConstraints(57.73611498784813, 26.6123752687, Math.toRadians(180), Math.toRadians(180), 18.82)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(270))) // COPY PASTE YOUR CODE AFTER THIS LINE, edit this line to the start pos of robot
                                //.back(17) // Goes forward the specified number of inches
                                .forward(15)
                                .turn(Math.toRadians(90))
                                .splineTo(new Vector2d(45, 44), Math.toRadians(270)) // This automatically gets the robot to the specified position using a spline curve
                                .splineTo(new Vector2d(45, 17), Math.toRadians(270)) // This automatically gets the robot to the specified position using a spline curve
                                .addDisplacementMarker(() -> {
                            //Arm(ARM_SPEED, 6);// Ship the hub

                             })
                                .setReversed(true)
                                .splineTo(new Vector2d(45, 44), Math.toRadians(90)) // This automatically gets the robot to the specified position using a spline curve
                                .addDisplacementMarker(() -> {
                                    //Arm(ARM_SPEED, 6);// Ship the hub

                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}