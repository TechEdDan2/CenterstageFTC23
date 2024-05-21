package org.firstinspires.ftc.teamcode.myOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.botSetup.RoboSetup;

/**
 * This is a Iterative OpMode driver program for a small
 *  mecanum chassis; this robot will have an (servo) intake, a
 *  virtual four bar lift, powered by a motor, a standard servo
 *  to control the angle of the bucket (pixel holder)
 *  and servos interacting with (clamping) the Pixel.
 *
 * @author DNel2
 * @version 11/28/2021 v1.0 updated 1/06/2024 v3.0
 */
@TeleOp(name = "Basic Mec Drive", group = "Iterative OpMode")
//@Disabled
public class BasicDriverTeleOp extends OpMode {
    //Create an instance of the RoboSetup
    RoboSetup myMecanumBot = new RoboSetup();

    Boolean isArmUp = false;
    Boolean isArmDown = true;
    String armState = "The Arm is Down";

    @Override
    public void init(){
        myMecanumBot.init(hardwareMap);
        telemetry.addData("Welcome to Team NelsonBot", "Program Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        //Variables
        double motorSpeedX = 0.0;
        double motorSpeedY = 0.0;
        double motorSpeedRx = 0.0;

//        telemetry.addData("Dist Right (CM): ", myMecanumBot.getCurrentDistanceRight(DistanceUnit.CM));
//        telemetry.addData("Dist Right (IN): ", myMecanumBot.getCurrentDistanceRight(DistanceUnit.INCH));
//        telemetry.addData("Dist Back (CM): ", myMecanumBot.getCurrentDistanceBack(DistanceUnit.CM));
//        telemetry.addData("Dist Back (IN): ", myMecanumBot.getCurrentDistanceBack(DistanceUnit.INCH));
//        telemetry.update();

        //------------------------------//
        // Main Chassis Driving Control //
        //------------------------------//
        if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2 ||
                gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2) {
            //Make the y value negative because forward on the game controller
            // creates negative values
            motorSpeedY = -gamepad1.left_stick_y;
            motorSpeedX = gamepad1.left_stick_x;
            motorSpeedRx = gamepad1.right_stick_x;
            myMecanumBot.setMecanumDrive(motorSpeedY, motorSpeedX, motorSpeedRx);
            telemetry.addData("Game pad Driving:", gamepad1.left_stick_y);
            telemetry.update();
        } else if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2) {
            motorSpeedY = -gamepad1.left_stick_y;
            motorSpeedX = gamepad1.left_stick_x;
            motorSpeedRx = gamepad1.right_stick_x;
            myMecanumBot.setMecanumDrive(motorSpeedY, motorSpeedX, motorSpeedRx);
            telemetry.addData("Game pad Right Stick X:", gamepad1.right_stick_x);
            telemetry.update();
        } else {
            myMecanumBot.setForward(0.0);
            myMecanumBot.setDiagLtB(0.0);
            myMecanumBot.setDiagLtF(0.0);
            myMecanumBot.setSlideLeft(0.0);
            myMecanumBot.setSlideRight(0.0);
        }

        //--------------------------//
        // Buttons for Manipulators //
        //--------------------------//

        //Press X to spin servo for IN or A for OUT
        if (gamepad1.a) {
            myMecanumBot.setIntatkeRotation(-0.75);
            telemetry.addData("Servo Rotating OUT", "Yes");
            telemetry.update();
        } else if (gamepad1.x){
            myMecanumBot.setIntatkeRotation(0.75);
            telemetry.addData("Servo Rotating IN", "Yes");
            telemetry.update();
        }else{
            myMecanumBot.setIntatkeRotation(0);
        }

        //Servo for Bucket, pixel collected
        // 0.5 is a Good delivery
        // 0.1 is a Good up pos for once the pixel is collected
        // 0.2 is Good for a neutral, "ready" to receive the pixel
        // 0.28 is Good for Collecting
        if(gamepad1.dpad_up){//good up pos
            myMecanumBot.setBucketServoPos(0.1);
        } else if(gamepad1.dpad_down){
            myMecanumBot.setBucketServoPos(0.5);
        }else if(gamepad1.dpad_left){
            myMecanumBot.setBucketServoPos(0.2);
        }else if(gamepad1.dpad_right){
            myMecanumBot.setBucketServoPos(0.28);
        }

        //Position Info:
        //  0.3 close (down)
        //  0.65 open (up)
        if(gamepad1.left_bumper){
            myMecanumBot.setBucketClampServo(0.3);
        }else if(gamepad1.right_bumper){
            myMecanumBot.setBucketClampServo(0.65);
        }

        //Full Arm Movement

        //Arm Position Down, ready for Pixel
        // This will require the motor to return to a set position
        //  and for the bucket servo to be rotated to tilt the bucket
        //  down.

        //Arm Position Up (Y), ready for delivery, Arm Pos Down (B)
        if(gamepad1.y){
            if(!isArmUp){
                myMecanumBot.setArmToDelivery();
                telemetry.addData("Arm Status", "Arm is Moving up");
                telemetry.update();
                isArmUp = true;
                isArmDown = false;
            }else{
                telemetry.addData("Arm Status", "The Arm is already up...stop pushing Y!");
                telemetry.update();
            }
        } else if (gamepad1.b) {
            if(!isArmDown){
                myMecanumBot.setArmToCollect();
                telemetry.addData("Arm Status", "The Arm is Moving Down");
                telemetry.update();
                isArmDown = true;
                isArmUp = false;
            }else {
                telemetry.addData("Arm Status", "The Arm is Down...stop pressing B!");
                telemetry.update();
            }
        }

        telemetry.addData("Right Dist: ", myMecanumBot.getCurrentDistanceRight(DistanceUnit.INCH));
        telemetry.addData("Left Dist: ",myMecanumBot.getCurrentDistanceLeft(DistanceUnit.INCH));
        telemetry.update();

    }//END OF LOOP

}//END OF CLASS