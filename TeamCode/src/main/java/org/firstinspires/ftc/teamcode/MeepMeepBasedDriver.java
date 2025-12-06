package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Meep Meep Based",
        xmlTag = "goBILDAPinpointMeepMeep",
        description ="goBILDA® Pinpoint Computer (Using Meep Meep Coordinate System)"
)

// Custom class to convert between Pinpoint and Meep Meep's coordinate systems
public class MeepMeepBasedDriver extends GoBildaPinpointDriver {

    // Constructor
    public MeepMeepBasedDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }

    // Return a modified Pose2D in Meep Meep's coordinate system, assuming a Pose2D is passed in directly form Pinpoint
    public Pose2D getMeepMeepPosition() {
        Pose2D pinPos = getPosition();
        return new Pose2D(
                DistanceUnit.INCH,
                -pinPos.getY(DistanceUnit.INCH), // This parameter is for Meep Meep's X (strafe) which is Pinpoint's -Y
                pinPos.getX(DistanceUnit.INCH), // This parameter is for Meep Meep's Y (forwards) which is Pinpoint's X
                AngleUnit.RADIANS,
                pinPos.getHeading(AngleUnit.RADIANS));
    }

    public void setMeepMeepPosition(Pose2D meepPos) {
        Pose2D pinPos = new Pose2D(
                DistanceUnit.MM,
                meepPos.getY(DistanceUnit.MM),
                -meepPos.getX(DistanceUnit.MM),
                AngleUnit.RADIANS,
                meepPos.getHeading(AngleUnit.RADIANS));
        setPosition(pinPos);
    }

} // End class