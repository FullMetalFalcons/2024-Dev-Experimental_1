package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


@TeleOp
public class Experimental1TeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, armLift, armExtender;
//    Servo claw, wrist;




    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");
        armLift = (DcMotorEx) hardwareMap.dcMotor.get("arm_lift");
        armExtender = (DcMotorEx) hardwareMap.dcMotor.get("arm_extender");



        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

//        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        Hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();



        while(opModeIsActive()) {
            // Mecanum drive code
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -0.2 : 0.0;
                px = gamepad1.dpad_right ? 0.2 : px;
                py = gamepad1.dpad_down ? -0.2 : 0.0;
                py = gamepad1.dpad_up ? 0.2 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = -gamepad1.right_stick_x;
            }

            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            if (gamepad1.left_trigger > 0){
                armLift.setPower(gamepad1.left_trigger * -1);
            } else if (gamepad1.right_trigger > 0){
                armLift.setPower(gamepad1.right_trigger);
            } else {
                armLift.setPower(0);
            }

            if (gamepad1.right_bumper){
                armExtender.setPower(-1);
            } else if (gamepad1.left_bumper){
                armExtender.setPower(1);
            } else {
                armExtender.setPower(0);
            }




//
//            // Slide Code
//            if (gamepad1.left_bumper) {
//                // Arm Up
//                Slide.setPower(-1);
//            } else if (gamepad1.left_trigger > 0) {
//                // Arm Down
//                Slide.setPower(1);
//            } else {
//                // At Rest
//                Slide.setPower(0);
//            }
//
//            // Hanging Arm Code
//            if (gamepad2.left_bumper) {
//                // Actuator Out
//                Hanger.setPower(-1);
//            } else if (gamepad2.left_trigger > 0) {
//                // Actuator In
//                Hanger.setPower(1);
//            } else {
//                // At Rest
//                Hanger.setPower(0);
//            }
//
//            // Claw Code
//            if (gamepad1.y) {
//                // Open Position
//                Claw.setPosition(0.8);
//            } else {
//                // Closed Position
//                Claw.setPosition(0.87);
//            }
//
//
//
//            odo.update();
//
//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//
//            Pose2D vel = odo.getVelocity();
//            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", velocity);
//
//            telemetry.update();

        } // opModeActive loop ends
    }
} // end class