package org.firstinspires.ftc.teamcode;

// TeleOp-related imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Pinpoint-related imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB;

    // Drive variables
    double powerLF; // Powers initialized here so that they can
    double powerLB; //   be referenced in a function below
    double powerRF;
    double powerRB;

    // Robot information storage
    double xPosition;
    double yPosition;
    Pose2D robotPos;

    // Field position variables
    /* Note that Meep Meep's field map has UP and RIGHT on the field being POSITIVE,
     *   but the Pinpoint instructions say to make UP and LEFT on the field POSITIVE
     *
     * I'm going to define locations using the Meep Meep coordinate system
     */
    // Where the robot starts on the field
    Pose2D startingPos = new Pose2D(DistanceUnit.INCH, 60.0, -12.0,
                                    AngleUnit.DEGREES, 90.0);
    // Where the goal is on the field
    Pose2D goalPos = new Pose2D(DistanceUnit.INCH, -60.0, 60.0,
                                    AngleUnit.DEGREES, 0.0);

    // Autonomous code
    Pose2D targetPos = new Pose2D(DistanceUnit.INCH, 0, -48,
            AngleUnit.DEGREES, 45);
    PIDLoop forwardPID = new PIDLoop(0.1,0,0.02);
    PIDLoop strafePID = new PIDLoop(0.1,0,0.02);


    // Access MecanumDrive and PinpointDriver for drive wheel/heading information
    //public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();

    public MeepMeepBasedDriver driver;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;
    double headingRadians;
    double headingFieldCentric;
    double desiredFreeHeading;

    // Custom class to store information about the robot's target heading
    public class TargetHeading {
        // Instance variables
        private double direction;
        private double errorDegrees;

        // Constructor
        public TargetHeading(double direction, double errorDegrees) {
            this.direction = direction;
            this.errorDegrees = errorDegrees;
        }

        // Accessor methods
        public double getDirection() {
            return direction;
        }
        public double getError() {
            return errorDegrees;
        }
    } // End class

//    @I2cDeviceType
//    @DeviceProperties(
//            name = "goBILDA® Pinpoint Meep Meep Based",
//            xmlTag = "goBILDAPinpointMeepMeep",
//            description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
//    )
//
//    // Custom class to convert between Pinpoint and Meep Meep's coordinate systems
//    public class MeepMeepBasedDriver extends GoBildaPinpointDriver {
//
//        // Constructor
//        public MeepMeepBasedDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
//            super(deviceClient, deviceClientIsOwned);
//        }
//
//        // Return a modified Pose2D in Meep Meep's coordinate system, assuming a Pose2D is passed in directly form Pinpoint
//        public Pose2D getMeepMeepPosition() {
//            Pose2D pinPos = getPosition();
//            return new Pose2D(
//                    DistanceUnit.INCH,
//                    -pinPos.getY(DistanceUnit.INCH), // This parameter is for Meep Meep's X (strafe) which is Pinpoint's -Y
//                    pinPos.getX(DistanceUnit.INCH), // This parameter is for Meep Meep's Y (forwards) which is Pinpoint's X
//                    AngleUnit.RADIANS,
//                    pinPos.getHeading(AngleUnit.RADIANS));
//        }
//    } // End class

    public TargetHeading targetHeading;


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    public void runOpMode() {
        // Setup pinpoint driver
        driver = hardwareMap.get(MeepMeepBasedDriver.class, "pinpoint");

        /*
        double mmPerTick = 25.4 * 122.5/62288;
        driver.setEncoderResolution(1 / mmPerTick);
        */

        // TODO: reverse encoder directions if needed
        // Forward and Left are both positive
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);
        driver.setOffsets(-90.0, -50.0);   // -X is to the right. -Y is to the back. Everything is in mm
        //driver.resetPosAndIMU(); // TODO: Don't reset if coming directly from Autonomous
        driver.setMeepMeepPosition(startingPos);

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
            robotPos = driver.getMeepMeepPosition();

            headingRadians = -robotPos.getHeading(AngleUnit.RADIANS);
            headingFieldCentric = headingRadians + startingPos.getHeading(AngleUnit.RADIANS);
            xPosition = robotPos.getX(DistanceUnit.INCH);
            yPosition = robotPos.getY(DistanceUnit.INCH);

            telemetry.addData("Robot heading", getCircularHeading(Math.toDegrees(headingRadians)));
            telemetry.addData("Relative heading", Math.toDegrees(headingFieldCentric));
            telemetry.addData("Desired heading", desiredFreeHeading);
            telemetry.addData("X", xPosition);
            telemetry.addData("Y", yPosition);

            // Set the desired powers based on joystick inputs (-1 to 1)
            double desiredForward = -gamepad1.left_stick_y;
            double desiredStrafe = gamepad1.left_stick_x;
            double powerAng;
            if (gamepad1.a) {
                // Set target heading based on the goal and its position compared to the robot
                desiredFreeHeading = ratioOfSidesToHeading(goalPos.getX(DistanceUnit.INCH) - xPosition,
                                                           goalPos.getY(DistanceUnit.INCH) - yPosition);
                targetHeading = determineRotationDirection(Math.toDegrees(headingRadians), desiredFreeHeading);
                powerAng = targetHeading.getDirection() * targetHeading.getError()/30.0;
            } else {
                powerAng = -gamepad1.right_stick_x;
            }

            double powerForward = (desiredForward * Math.cos(headingFieldCentric)) + (desiredStrafe * Math.sin(headingFieldCentric));
            double powerStrafe = (desiredStrafe * Math.cos(headingFieldCentric)) - (desiredForward * Math.sin(headingFieldCentric));

            if (gamepad1.y) {
                desiredStrafe = forwardPID.calculatePower(targetPos.getX(DistanceUnit.INCH) - xPosition);
                desiredForward = strafePID.calculatePower(targetPos.getY(DistanceUnit.INCH) - yPosition);

                powerForward = (desiredForward * Math.cos(headingRadians)) + (desiredStrafe * Math.sin(headingRadians));
                powerStrafe = (desiredStrafe * Math.cos(headingRadians)) - (desiredForward * Math.sin(headingRadians));
                targetHeading = determineRotationDirection(Math.toDegrees(headingRadians), targetPos.getHeading(AngleUnit.DEGREES));
                powerAng = targetHeading.getDirection() * targetHeading.getError()/30.0;
            }

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

    public double ratioOfSidesToHeading(double X, double Y) {
        double freeHeading = 0.0;
        // If Y is zero, then the angle is purely horizontal
        // Otherwise, the angle can be calculated with trig
        if (Y == 0.0) {
            // Determine which horizontal based on the sign on X
            freeHeading = (X > 0.0) ? 90.0 : -90.0;
        } else {
            // Calculate the heading
            freeHeading = Math.toDegrees(Math.atan(X/Y));
        }
        /*
                                 With inverse tangent, (+X, +Y) and (-X, -Y) are indistinguishable
     325        0        45
            I   |   II
          -X,+Y | +X,+Y               inverse tangent accounts for quadrants I and II
     270 -------|------- 90      but can't tell the different between those and quadrants III and IV
          -X,-Y | +X,-Y
           III  |   IV           so, if Y is negative (quad III or IV), then add 180 degrees to
     225       180      135        the calculated angle

        */
        if (Y < 0.0) {
            freeHeading += 180.0;
        } else if (X < 0.0) {
            // Add additional code for quad I to remove all negative values
            freeHeading += 360;
        }

        // Return the final calculated heading
        return freeHeading;
    }

    public TargetHeading determineRotationDirection(double current, double target) {
        double currentCircularHeading = getCircularHeading(current);
        double targetHeading = getCircularHeading(target);
        double clockwiseDegrees;
        double counterclockwiseDegrees;

        // Determine the larger of the two angles
        if (targetHeading > currentCircularHeading) {
            // Subtract the smaller current heading from the larger target heading
            //   to find the degrees needed to turn to get to the target going clockwise
            clockwiseDegrees = targetHeading - currentCircularHeading;
            // Find the alternative
            counterclockwiseDegrees = 360 - clockwiseDegrees;
        } else {
            // Subtract the smaller target heading from the larger current heading
            //   to find the degrees needed to turn to get to the target doing counterclockwise
            counterclockwiseDegrees = currentCircularHeading - targetHeading;
            // Find the alternative
            clockwiseDegrees = 360 - counterclockwiseDegrees;
        }
        // Determine the most efficient direction and return the proper multiplier
        if (clockwiseDegrees < counterclockwiseDegrees) {
            return new TargetHeading(-1.0, Math.abs(clockwiseDegrees));
        } else {
            return new TargetHeading(1.0, Math.abs(counterclockwiseDegrees));
        }
    }

    public double getCircularHeading(double actualHeading) {
        // Mod the heading to make it between 0 and 360
        double circularHeading = actualHeading % 360;

        // Java's Mod function can return negative numbers, so account for that possibility
        if (circularHeading < 0) circularHeading += 360;
        return circularHeading;
    }

} // end class