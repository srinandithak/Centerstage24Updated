package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class BlueAutonLeftMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        int xStart = 7;
        int yStart = 62;
        double headingStart = Math.toRadians(90);
        Pose2d startPose = new Pose2d(xStart, yStart, headingStart);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                //strafe
                                .lineToConstantHeading(new Vector2d(xStart +12, yStart -3))
                                //dropPixel
                                .lineToConstantHeading(new Vector2d(xStart +16,yStart -25))
                                .waitSeconds(1)
                                //toBackdrop
                                .lineToLinearHeading(new Pose2d(xStart +43,yStart-18, headingStart + Math.toRadians(-90)))
                                .waitSeconds(.5)
                                //reset
                                .lineToLinearHeading(new Pose2d(xStart +35,yStart-24, headingStart + Math.toRadians(-90)))
                                //shift
                                .lineToConstantHeading(new Vector2d(xStart +35, yStart-52))
                                //toPixels
                                .lineToConstantHeading(new Vector2d(xStart -68, yStart-50))
                                .waitSeconds(4)
                                //back
                                .lineToConstantHeading(new Vector2d(xStart +35, yStart-52))
                                //backDropAgain
                                .lineToLinearHeading(new Pose2d(xStart +41.5,yStart-26, headingStart + Math.toRadians(-90)))
                                .waitSeconds(1)
                                //park1
                                .lineToConstantHeading(new Vector2d(xStart + 20, yStart))
                                //park2
                                .lineToConstantHeading(new Vector2d(xStart + 50, yStart))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}