package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB;

    // Drive variables
    double powerLF; // Powers initialized here so that they can
    double powerLB; //   be referenced in a function below
    double powerRF;
    double powerRB;
    double desiredPower;

    // Access MecanumDrive and PinpointDriver for drive wheel/heading information
    //public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();

    public GoBildaPinpointDriver driver;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;
    double headingRadians;


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    public void runOpMode() {
        // Setup pinpoint driver
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
        double mmPerTick = 25.4 * 122.5/62288;
        driver.setEncoderResolution(1 / mmPerTick);
        driver.setOffsets(0, 0); // TODO:  Set actual offsets, in mm
        */

        // TODO: reverse encoder directions if needed
        // Forward and Left are both positive
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        // Define drive motors
        //The string should be the name on the Driver Hub
        // Set the strings at the top of the MecanumDrive file; they are shared between TeleOp and Autonomous
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        //This reverses the motor direction
        // This data is also set at the top of MecanumDrive, for the same reasons as above
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);

        //This resets the encoder values when the code is initialized
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The program will pause here until the Play icon is pressed on the Driver Station
        waitForStart();

        // opModeIsActive() returns "true" as long as the Stop button has not been pressed on the Driver Station
        while(opModeIsActive()) {

            // Field centric drive code
            // Get heading of the robot from pinpoint driver
            driver.update();
            headingRadians = -driver.getHeading();

            telemetry.addData("Robot heading", Math.toDegrees(headingRadians));

            // Set the desired powers based on joystick inputs (-1 to 1)
            double desiredForward = -gamepad1.left_stick_y;
            double desiredStrafe = gamepad1.left_stick_x;
            double powerAng = -gamepad1.right_stick_x;

            double powerForward = (desiredForward * Math.cos(headingRadians)) + (desiredStrafe * Math.sin(headingRadians));
            double powerStrafe = (desiredStrafe * Math.cos(headingRadians)) - (desiredForward * Math.sin(headingRadians));

            // Perform vector math to determine the desired powers for each wheel
            powerLF = powerStrafe + powerForward - powerAng;
            powerLB = -powerStrafe + powerForward - powerAng;
            powerRF = -powerStrafe + powerForward + powerAng;
            powerRB = powerStrafe + powerForward + powerAng;

            // Determine the greatest wheel power and set it to max
            double max = Math.max(1.0, Math.abs(powerLF));
            max = Math.max(max, Math.abs(powerRF));
            max = Math.max(max, Math.abs(powerLB));
            max = Math.max(max, Math.abs(powerRB));

            // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
            powerLF /= max;
            powerLB /= max;
            powerRF /= max;
            powerRB /= max;

            motorLF.setPower(powerLF);
            motorLB.setPower(powerLB);
            motorRF.setPower(powerRF);
            motorRB.setPower(powerRB);


            // If you want to print information to the Driver Station, use telemetry
            // addData() lets you give a string which is automatically followed by a ":" when printed
            //     the variable that you list after the comma will be displayed next to the label
            // update() only needs to be run once and will "push" all of the added data

            telemetry.update();


        } // opModeActive loop ends
    }

} // end class