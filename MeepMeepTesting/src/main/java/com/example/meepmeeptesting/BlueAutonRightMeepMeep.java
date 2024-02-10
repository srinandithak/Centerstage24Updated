package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueAutonRightMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        int xStart = -41;
        int yStart = 62;
        double headingStart = Math.toRadians(90);
        Pose2d startPose = new Pose2d(xStart, yStart, headingStart);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                //dropPixel
                                .lineToConstantHeading(new Vector2d(xStart +7, yStart -28))
                                .waitSeconds(1)
                                //turn
                                .lineToLinearHeading(new Pose2d(xStart + 5,yStart - 28, headingStart + Math.toRadians(-90)))
                                //toBackdrop
                                .lineToConstantHeading(new Vector2d(xStart +89,yStart-28))
                                .waitSeconds(.5)
                                //reset
                                .lineToLinearHeading(new Pose2d(xStart +85,yStart-28, headingStart + Math.toRadians(-90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}