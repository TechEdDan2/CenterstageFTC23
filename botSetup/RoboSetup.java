package org.firstinspires.ftc.teamcode.botSetup;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Thread.sleep;

/**
 * This is the code for setting up my Centerstage robot which consists of
 *  a small mecanum chassis. This will be version of mecanum drive. The class
 *  will review other version using mathematical equations to achieve
 *  more elegant coding.
 *
 *  Hardware Setup
 *
 *  ---- Control Hub ---
 *  MOTORS
 *  Port 0 - backLeft
 *  Port 1 - frontLeft
 *  Port 2 - frontRight
 *  Port 3 - backRight
 *
 *  SERVOS
 *  Port 0 - bucketServo
 *  Port 1 - bucketClampServo
 *  Port 5 - intakeServo (continuous)
 *
 *  SENSORS
 *  I2C Expansion
 *  Port 0 - leftColor (Rev V3 Color Sensor)
 *  Port 1 - leftDist (REV 2M Distance Sensor)
 *  Port 2 - rightColor (Rev V3 Color Sensor)
 *  Port 3 - rightDist (REV 2M Distance Sensor)
 *
 * --- Expansion Hub ----
 * Port 0 - Motor armMotor
 * Port 3 - Motor TBD
 *
 * @author DNel2
 * @version v1 10/06/2023 current update 1/6/2024
 *
 */
public class RoboSetup {
    //Instance Variables for Mechanisms
    //////////////////////
    //      Motors      //
    //////////////////////
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor armMotor; //This will lift the virtual four bar lift
    // private DcMotor intakeMotor;
    //////////////////////
    //      Servos      //
    //////////////////////
    private CRServo intakeServo; //This will control the spinning intake
    private Servo bucketServo; //this will rotate the bucket up and down

    private Servo bucketClampServo;


    //////////////////////
    //      Sensors      //
    //////////////////////
    private DistanceSensor rightDist;
    private DistanceSensor leftDist;
//    //private TouchSensor myLiftTouch;

    /////////////////////////////
    //     Counter Value       //
    ////////////////////////////
    private double ticksPerRotation;
    private double ticksPerRotationFL;
    private double ticksPerRotationBL;
    private double ticksPerRotationFR;
    private double ticksPerRotationBR;

    private double ticksPerRotationAM;


    //goBilda Encoder count 537.7 PPR at the Output Shaft
    static final double GOBILDA_COUNTS_OUTPUT = 537.7;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (GOBILDA_COUNTS_OUTPUT * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    int newUpTarget;
    int newDownTarget;

    double newGeneralTarget;

    private ElapsedTime runtime = new ElapsedTime();

    //Add the param for init with HardwareMap
    public void init(HardwareMap hwMap) {
        //setting up the motors
        //Left Back Wheel Motor
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All left will be reversed for Math version of Mec Drive
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Wheel Motor
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All left will be reversed for Math version of Mec Drive
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right Back Wheel Motor
        backRight = hwMap.get(DcMotor.class, "backRight");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All Right will be reversed ... never
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right Front Wheel Motor
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All Right will be reversed ... never
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Arm Motor which is driving a virtual four bar lift
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Setting up the Sensors
        rightDist = hwMap.get(DistanceSensor.class, "rightDist");
        leftDist = hwMap.get(DistanceSensor.class, "leftDist");
        // myLiftTouch = hwMap.get(TouchSensor.class, "liftTouch");

        //Setting up the servos
        //The intake servo is a continuous rotation servo
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeServo.resetDeviceConfigurationForOpMode();
        // The bucket servo is a standard torque servo
        bucketServo = hwMap.get(Servo.class, "bucketServo");
        bucketServo.resetDeviceConfigurationForOpMode();
        //The bucket clamp servo is a standard torque servo
        bucketClampServo = hwMap.get(Servo.class,"clampServo");
        bucketClampServo.resetDeviceConfigurationForOpMode();

    } //END OF Init

    //Methods

    ///////////////////////////////////////////////////
    //                                               //
    // Hardcoded Mecanum Methods - Not Best Practice //
    //                                               //
    ///////////////////////////////////////////////////
    /**
     * This method will move the robot forward
     * @param power - double value to set the power of the movement
     */
    public void setForward(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * The setForwardToDist() method will use encoders to
     *   rotate the wheels for a specific amount of time
     *   to reach a passed target distance (mainly in autonomous)
     *
     * @param targetDist - this will be the target distance in INCHES
     *
     */
    public void setForwardToDist(double targetDist){
        //This is the value for moving to the correct position
        double countToMove = COUNTS_PER_INCH * targetDist;

        //Reset the timer
        runtime.reset();

        while(runtime.seconds() < 1){
            //This is a break for the Alliance Partner to have enough time to move
            //  out of the way if starting on Audience side... might remove later
        }

        // Determine new target position, and pass to motor controller
        newGeneralTarget = backRight.getCurrentPosition() + countToMove;
        backRight.setTargetPosition((int)newGeneralTarget);

        // Turn On RUN_TO_POSITION
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        setMecanumDrive(0.2, 0.0, 0.0);

        // keep looping while we are still active
        while (backRight.isBusy()) {
            //This is an empty loop to make sure that the arm has time to run
        }
        setMecanumDrive(0.0,0.0,0.0);
    }

    /**
     * This method will move the robot backward
     * @param power - double value to set the power of the movement
     */
    public void setBackward(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot to slide to the left
     * @param power - double value to set the power of the movement
     */
    public void setSlideLeft(double power){
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(power);
    }

    /**
     * The setSlideToDist() method will use encoders to
     *   rotate the wheels for a specific amount of time
     *   to reach a passed target distance (mainly in autonomous)
     *
     * @param targetDist - this will be the target distance in INCHES
     * @param slideDirection - this will be the direct to slide... left or right
     *
     */
    public void setSlideToDist(double targetDist, String slideDirection){
        //This is the value for moving to the correct position
        double countToMove = COUNTS_PER_INCH * targetDist;

        //Might need to make the encoder target value negative
        if(slideDirection.equalsIgnoreCase("left")){
            countToMove*=-1;
        }

        //Reset the timer
        runtime.reset();

        while(runtime.seconds() < 1){
            //This is a break for the Alliance Partner to have enough time to move
            //  out of the way if starting on Audience side... might remove later
        }

        // Determine new target position, and pass to motor controller
        newGeneralTarget = backRight.getCurrentPosition() + countToMove;
        backRight.setTargetPosition((int)newGeneralTarget);

        // Turn On RUN_TO_POSITION
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        if(slideDirection.equalsIgnoreCase("left")){
            setMecanumDrive(0.0, -0.2, 0.0);
        }else{
            setMecanumDrive(0.0, 0.2, 0.0);
        }


        // keep looping while we are still active
        while (backRight.isBusy()) {
            //This is an empty loop to make sure that the arm has time to run
        }
        setMecanumDrive(0.0,0.0,0.0);
    }

    /**
     * This method will move the robot to slide to the Right
     * @param power - double value to set the power of the movement
     */
    public void setSlideRight(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(-power);
    }

    /**
     * This method will move the robot to turn left
     * @param power - double value to set the power of the movement
     */
    public void setTurnLeft(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(-power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot to turn left
     * @param power - double value to set the power of the movement
     */
    public void setTurnRight(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(-power);
        frontRight.setPower(-power);
    }

    /**
     * This method will move the robot diagonally forward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagLtF(double power){
        backLeft.setPower(power);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot diagonally backward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagLtB(double power){
        backLeft.setPower(0);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(0);
    }

    /**
     * This method will move the robot diagonally forward to the right
     * @param power - double value to set the power of the movement
     */
    public void setDiagRtF(double power){
        backLeft.setPower(0);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(0);
    }

    /**
     * This method will move the robot diagonally backward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagRtB(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(-power);
    }


    //////////////////////////////////////////////////////////
    //                                                      //
    //   CLEANER VERSION OF THE PROGRAM FOR MECANUM         //
    //                                                      //
    //////////////////////////////////////////////////////////

    /**
     * This method will set all the values for the motors
     *  to drive the mecanum chassis. This method is inspired
     *  by code from gmZero programming tutorial
     *
     *  The left stick will control linear movement:
     *   forward, back, slide left, slide right, and
     *   diagonal movement.
     *  The right stick will control the rotational movement:
     *   spin left, spin right.
     *
     * @param y - the left stick y (double) value
     * @param x - the left stick x (double) value
     * @param rx - the right stick x (double) value
     */
    public void setMecanumDrive(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) /denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /////////////////////////////////////////////
    //                                          //
    //            Servo Methods                 //
    //                                          //
    /////////////////////////////////////////////

    /**
     *  This method will rotate the Continuous servo
     *    that is being used for the intake
     *
     * @param power
     */
    public void setIntatkeRotation(double power){
        intakeServo.setPower(power);
    }

    /**
     * This method will control the bucket servo (torque)
     *
     * @param servoPos - this variable will set the position of the servo
     *
     */
    public void setBucketServoPos(double servoPos){
        bucketServo.setPosition(servoPos);
    }

    /**
     * The setBucketClamp() method will open and close the clamp
     * and take a double pos value
     *
     * @param servoPos - this double value will set the clamp position
     */
    public void setBucketClampServo(double servoPos){
        bucketClampServo.setPosition(servoPos);
    }

    //////////////////////////////////////////////
    //                                          //
    //      Arm Methods                         //
    //                                          //
    /////////////////////////////////////////////

    /**
     * This method will move both the motor for the
     *   arm and the servo to the position to be up.
     */
    public void setArmToDelivery(){
        //This is the value for moving to the correct position
        int countToMove = 300;

        //Moves the bucket up so it can pass by the intake
        setBucketServoPos(0.1);
        //Ensures the clamp is down
        setBucketClampServo(0.3);
        // Determine new target position, and pass to motor controller
        newUpTarget = armMotor.getCurrentPosition() + countToMove;
        armMotor.setTargetPosition(newUpTarget);

        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        armMotor.setPower(0.5);

        // keep looping while we are still active
        while (armMotor.isBusy()) {
            //This is an empty loop to make sure that the arm has time to run
        }
        // Stop all motion;
        //armMotor.setPower(0);

        //Moves the bucket up so it can pass by the intake
        setBucketServoPos(0.5);
    }

    /**
     * The setArmToCollect() method will return the
     *   collection arm with bucket back to the
     *   low position ready to collect more Pixels.
     */
    public void setArmToCollect(){
        //This is the value for moving to the correct position
        int countToMove = -315;

        //Moves the bucket up so it can pass by the intake
        setBucketServoPos(0.1);

        //Reset the timer
        runtime.reset();

        while(runtime.seconds() < 1){
            //This is a break for the servo to have enough time to move
        }

        // Determine new target position, and pass to motor controller
        newUpTarget = armMotor.getCurrentPosition() + countToMove;
        armMotor.setTargetPosition(newUpTarget);

        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        armMotor.setPower(0.1);

        // keep looping while we are still active
        while (armMotor.isBusy()) {
            //This is an empty loop to make sure that the arm has time to run
        }


        //Moves the bucket to a "ready" position
        setBucketServoPos(0.05);
        //Open up the clamp ready for Pixels
        setBucketClampServo(0.65);



    }

    //////////////////////////////////////////////
    //                                          //
    //      Sensor Methods                      //
    //                                          //
    /////////////////////////////////////////////

    /**
     * This method will return the sensor reading
     * of the distance sensor
     *
     * @return Double value for the distance read from the sensor
     *
     */
    public double getCurrentDistanceRight(DistanceUnit du){
        return rightDist.getDistance(du);
    }

    /**
     * This method will return the sensor reading
     * of the distance sensor that is located at the back
     * of the robot
     *
     * @return Double value for the distance read from the sensor
     *
     */
    public double getCurrentDistanceLeft(DistanceUnit du){
        return leftDist.getDistance(du);
    }


    /**
     * This method will return the state of the
     *  touch sensor located at the top of the lift
     *  mechanism
     *
     * @return - boolean value true pressed false unpressed
     */
//    public boolean getLiftTouchSensorState(){
//        boolean isLiftPressed = false;
//        isLiftPressed = myLiftTouch.isPressed();
//        return isLiftPressed;
//    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationFR() {
        ticksPerRotationFR = frontRight.getMotorType().getTicksPerRev();
        return frontRight.getCurrentPosition() / ticksPerRotationFR;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationBR() {
        ticksPerRotationBR = backRight.getMotorType().getTicksPerRev();
        return backRight.getCurrentPosition() / ticksPerRotationBR;
    }

    /**
     * This method will return the motor encoder rotations
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationFL() {
        ticksPerRotationFL = frontLeft.getMotorType().getTicksPerRev();
        return frontLeft.getCurrentPosition() / ticksPerRotationFL;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationBL() {
        ticksPerRotationBL = backRight.getMotorType().getTicksPerRev();
        return backLeft.getCurrentPosition() / ticksPerRotationBL;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationArmLift() {
        ticksPerRotationAM = backRight.getMotorType().getTicksPerRev();
        return armMotor.getCurrentPosition() / ticksPerRotationAM;
    }

}//END OF CLASS