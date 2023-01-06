package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/*
TODO: Test trajectories
TODO: Get Servo & Motor Pos.
TODO: Add pre-loaded cone drop off
TODO: Separate movement trajectories
TODO: Add Vision
TODO: Add Sound
 */

/*
NOTE: Motor Levels
-Start Pos
-Ground Station =
-Low Pole =
-Mid Pole =
-High Pole = 1650


NOTE: Servo Levels
-Hooked = 0.04
-Unhooked = 0.2
 */

@Config
@Autonomous(group = "drive")
public class Amon_Gus extends LinearOpMode {

    private DcMotor arm;
    private Servo leftHook;
    boolean FirstTime = true;

    @Override
    public void runOpMode() throws InterruptedException {

        //Arm Init
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Hook Init
        leftHook = hardwareMap.servo.get("leftHook");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36,-64.5,Math.toRadians(90)));


        //Talk to Mr. Johnson about making the servo constatly grip
        leftHook.setPosition(0.04);



        //First Movement (start to first junction, place cone, arrive at warehouse)
        Trajectory splineToWare = drive.trajectoryBuilder(new Pose2d(-36,-64.5,Math.toRadians(90)))

                //Lift Arm
                .addDisplacementMarker(() -> {
                    leftHook.setPosition(0.04);
                    arm.setTargetPosition(250);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);
                    leftHook.setPosition(0.04);
                    sleep(500);
                })

                .splineToConstantHeading(new Vector2d(-13, -64.5), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-10, -24), Math.toRadians(90))

                .build();


          Trajectory placeCone = drive.trajectoryBuilder(new Pose2d(-10, -24), Math.toRadians(90))
                  .addDisplacementMarker(() -> {
                      sleep(500);
                      leftHook.setPosition(0.04);
                      arm.setTargetPosition(1650);
                      arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                      ((DcMotorEx) arm).setVelocity(250);
                      leftHook.setPosition(0.04);
                      drive.turn(Math.toRadians(-90));
                      sleep(1000);
                  })


                // Move to warehouse
                  .lineTo(new Vector2d(-9,-24))

                //Lower Arm, Place Cone, Raise Arm
                .addDisplacementMarker(() -> {
                    arm.setTargetPosition(400);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);
                    leftHook.setPosition(0.2);
                    arm.setTargetPosition(1650);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);
                    sleep(1000);

                })

                  .build();
                //Strafe to 2 tiles away from warehouse


        Trajectory strafeWarehouse = drive.trajectoryBuilder(new Pose2d(-9, -24), Math.toRadians(180))
                .strafeTo(new Vector2d(-9, -10))
//
                //Lower Arm
                .addDisplacementMarker(() -> {
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);
                    leftHook.setPosition(0.04);
                })

         .build();
//

//
////

        waitForStart();

        //Code to make traj run once
        while(opModeIsActive() == true){
            if ((opModeIsActive() && FirstTime == true)){
               //Spline to Warehouse
                drive.followTrajectory(splineToWare);

                drive.followTrajectory(placeCone);

                drive.followTrajectory(strafeWarehouse);

                FirstTime = false;
            }}
    }
}


