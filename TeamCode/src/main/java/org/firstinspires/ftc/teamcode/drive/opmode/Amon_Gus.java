package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

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

import java.util.Vector;


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


        drive.turn(3.14/2);


                //Spline to Warehouse

//
//                drive.followTrajectory(pickAndPlace);

            }
        }



