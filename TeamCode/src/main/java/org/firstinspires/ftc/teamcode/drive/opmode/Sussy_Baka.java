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
-High Pole =


NOTE: Servo Levels
-Hooked = 0.04
-Unhooked = 0.2
 */

@Config
@Autonomous(group = "drive")
public class Sussy_Baka extends LinearOpMode {

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

        drive.setPoseEstimate(new Pose2d(35,64.2,Math.toRadians(270)));

        //Make sure cone is hooked
        leftHook.setPosition(0.2);

        //Possibly make multiple traj to test ind. parts
        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d(35,64.2,Math.toRadians(270)))


//        Change Math.toRadians(90) if it spins too much
//                Move to Corner

                .lineTo(new Vector2d(58,64.2 ))


                //Sleep for stability
                .addDisplacementMarker(() -> {
                    sleep(100);
                })

                //Move to pole
                .lineTo(new Vector2d(58, 24))


//               Lift Arm
                .addDisplacementMarker(() -> {
                    //Claw and Arm movement
                    arm.setTargetPosition(500);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);

                })


                //Move to over pole
                .splineTo(new Vector2d(57,24), Math.toRadians(180))

                //Get Cone
                .addDisplacementMarker(() -> {
                    //Claw and Arm movement
                    arm.setTargetPosition(500);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) arm).setVelocity(250);
                    leftHook.setPosition(0.2);
                })
//
//                    //Move to pole
//                    .splineToLinearHeading(new Pose2d(-35,0), Math.toRadians(0))
//
//                    //Place Cone
//                    .addDisplacementMarker(() -> {
//                        //Claw and Arm movement
//                        arm.setTargetPosition(0);
//                        leftHook.setPosition(0.2);
//                        sleep(100);
//                        arm.setTargetPosition(1);
//                    })


                .build();


        waitForStart();

        //Code to make traj run once
        while(opModeIsActive() == true){
            if ((opModeIsActive() && FirstTime == true)){
                drive.followTrajectory(trajectoryForward);
                FirstTime = false;
            }}
    }
}


