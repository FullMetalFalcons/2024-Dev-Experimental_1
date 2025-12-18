package org.firstinspires.ftc.teamcode;

public class PIDLoop {

    // Instance variables
    private double kP; // Proportional constant
    private double kI; // Integral constant
    private double kD; // Derivative constant

    private double lastError;
    private double sumError;

    private long lastLoopTime;
    private double elapsedTime;


    // Constructors
    public PIDLoop(double kP, double kI, double kD) {
        setGains(kP, kI, kD);
        lastLoopTime = 0;
        lastError = 0.0;
        sumError = 0.0;
        elapsedTime = 0.0;
    }
    public PIDLoop() {
        this(1,0,0);
    }


    // Methods
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }

    public double calculatePower(double error) {
        // Calculate the time since the last loop in nanoseconds
        elapsedTime = System.nanoTime() - lastLoopTime;
        elapsedTime /= 1000000000; // Divide by 1 billion (nine zeros) to convert to seconds

        // Set the derivative by finding the change in error from last loop
        double derivative = (error - lastError) / elapsedTime;

        // Add current error to sumError in order to keep a running sum
        sumError += error * elapsedTime;

        // Combine all factors to give the final power output
        double output = (kP * error) + (kI * sumError) + (kD * derivative);

        lastError = error;
        lastLoopTime = System.nanoTime();
        return output;
    }

}
