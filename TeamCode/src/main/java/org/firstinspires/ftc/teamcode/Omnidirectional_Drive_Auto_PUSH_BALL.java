/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Omnidirectional Drive Auto PUSH BALL", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive_Auto_PUSH_BALL extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftFlipper = null;
    Servo rightFlipper = null;
    Servo leftArm = null;
    Servo rightArm = null;
    Servo holder1 = null;

    DcMotor shootMotor = null;

    ColorSensor leftColor;  // Hardware Device Object
    ColorSensor rightColor;
    TouchSensor leftTouch;
    TouchSensor rightTouch;
    OpticalDistanceSensor lineReader;

    Impulse i = new Impulse();

    @Override
    public void runOpMode() throws InterruptedException {

        //variable setup
        double buffer = 0.25;               //how far the joystick must move before moving the motors
        String direction = "stop";          //the direction the robot will be heading
        double motorSpeed = 0.5;           //the power the motors will be set to
        int maxSpeed = 2800;
        double spinSpeed = 0.25;            //the power the motors will be set to while spinning
        double ticksPerRev = 1120;          //the amount of encoder ticks per revolution of the wheel

        int shootTimer = 0;                 //how long the gun has been shooting for
        double shootSpeed = 1;              //how fast the gun shoots at
        int shootMax = 50;                  //the maximum position of the launcher
        int shootMin = 10;                  //the minimum position of the launcher

        boolean colorSensorLEDOn = false;    //if the color sensor LED is on or not
        boolean buttonPressed1 = false;     //if a,b,x, or y is pressed on gamepad1

        double leftFlipperBack = 0.25;      //position of the left flipper when retracted
        double leftFlipperForward = 0.0;    //position of the left flipper when extended
        boolean leftFlipperOut = false;     //if the left flipper is out or not

        double rightFlipperBack = 0.75;     //position of the right flipper when retracted
        double rightFlipperForward = 1.0;   //position of the right flipper when extended
        boolean rightFlipperOut = false;    //if the right flipper is out or not

        double leftArmUp = 0.80;            //position of the left arm when up
        double leftArmDown = 0.2;           //position of the left arm when down
        boolean leftArmOut = false;         //if the left arm is out or not

        double holderUp = 1;                //position of the holder when up
        double holder1Down = 0.5;            //position of the holder when down
        boolean holder1Out = false;          //if the holder is down or not



        //auto variables
        String leftColorSensed = "none";
        String rightColorSensed = "none";
        String SIDE_COLOR = "blue";
        String colorToPress = "none";
        boolean wallReached = false;
        boolean side1Moved = false;
        boolean line1Detected = false;
        boolean beacon1Pressed = false;
        boolean side2Moved = false;
        boolean line2Detected = false;
        boolean beacon2Pressed = false;

        int moveSideTimer = 0;
        int moveSideTime = 6000;

        int pressTimer = 0;
        int pressTime = 20000;

        double lineLight = 1;                 //the value at which the line gives off light

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");

        leftArm = hardwareMap.servo.get("left arm");
        holder1 = hardwareMap.servo.get("holder1");

        shootMotor = hardwareMap.dcMotor.get("shoot");

        shootMotor.setDirection(DcMotor.Direction.REVERSE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Crow crow = new Crow(telemetry, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftFlipper, rightFlipper, shootMotor, colorSensor);
        //Impulse i = new Impulse();
        //i.setCrow(crow);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMaxSpeed(maxSpeed);
        frontRightMotor.setMaxSpeed(maxSpeed);
        backLeftMotor.setMaxSpeed(maxSpeed);
        backRightMotor.setMaxSpeed(maxSpeed);



        //sensor setup
        leftColor = hardwareMap.colorSensor.get("left color");
        rightColor = hardwareMap.colorSensor.get("right color");
        rightColor.setI2cAddress(I2cAddr.create8bit(0x4c)); //this makes it so that we can have 2 color sensors
        leftTouch = hardwareMap.touchSensor.get("left touch");
        rightTouch = hardwareMap.touchSensor.get("right touch");
        lineReader = hardwareMap.opticalDistanceSensor.get("ods");

        leftColor.enableLed(colorSensorLEDOn);  //turn on the left color sensor light when init
        rightColor.enableLed(colorSensorLEDOn); //turn off the right color sensor light when init
        telemetry.addData("Status", "Initialized");//tell that everything is started
        telemetry.update();

        //setting init servo pos
        leftArm.setPosition(leftArmUp);
        holder1.setPosition(holder1Down);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :", direction);
            telemetry.addData("Motor Speed: ", motorSpeed);
            //telemetry.addData("Spin Speed: ", spinSpeed);
            telemetry.addData("L color - ", "Red: " + leftColor.red() + "Green: " + leftColor.green() + "Blue: " + leftColor.blue() + leftColorSensed);
            telemetry.addData("R color - ", "Red: " + rightColor.red() + "Green: " + rightColor.green() + "Blue: " + rightColor.blue() + rightColorSensed);
            telemetry.addData("Color to press: ",  colorToPress);
            telemetry.addData("Flippers - ", "Right " + rightFlipperOut + " Left " + leftFlipperOut);
            //telemetry.addData("ColorSensor LED: ", colorSensorLEDOn);
            //telemetry.addData("Touch - ", "left: " + leftTouch.isPressed() + " right: " + rightTouch.isPressed());
            telemetry.addData("ODS: ", lineReader.getRawLightDetected());
            telemetry.addData("Encoders: ", "FL " + frontLeftMotor.getCurrentPosition() + " FR " + frontRightMotor.getCurrentPosition() + " BL " + backLeftMotor.getCurrentPosition() + " BR " + backRightMotor.getCurrentPosition());
            //telemetry.addData("Shooting ", "Timer: " + shootTimer + " Pos: " + shootMotor.getCurrentPosition());
            //telemetry.addData("Triggers: ", "L2: " + gamepad2.left_trigger + " R2: " + gamepad2.right_trigger);
            //telemetry.addData("Arms: ", "Left: " + leftArmOut);
            //telemetry.addData("Holder1: ", holder1Out);
            telemetry.update();


            //autonomous logik

            if(pressTimer<pressTime && !buttonPressed1){
                direction = "stop";
                pressTimer++;
            } else if(moveSideTimer<moveSideTime && !side1Moved){
                buttonPressed1 = true;
                direction = "forwards";
                moveSideTimer++;
            } else {
                side1Moved = true;
                colorToPress = "none";
                //beacon2Pressed = true;
                direction = "stop";
                motorSpeed = 0.25;
            }


            if (leftColor.red()> leftColor.blue()){
                leftColorSensed = "red";
            } else if (leftColor.blue() > leftColor.red()){
                leftColorSensed = "blue";
            } else {
                leftColorSensed = "none";
            }

            if (rightColor.red()> rightColor.blue()){
                rightColorSensed = "red";
            } else if (rightColor.blue() > rightColor.red()){
                rightColorSensed = "blue";
            } else {
                rightColorSensed = "none";
            }

            /*
            if (gamepad2.x){
                colorToPress = "blue";
            } else if (gamepad2.b){
                colorToPress = "red";
            } else {
                colorToPress = "none";
            }
            */

            if(leftColorSensed.equals("none") || colorToPress.equals("none")){
                leftFlipperOut = false;
            } else if (leftColorSensed.equals(colorToPress)){
                leftFlipperOut = true;
            }

            if(rightColorSensed.equals("none") || colorToPress.equals("none")){
                rightFlipperOut = false;
            } else if (rightColorSensed.equals(colorToPress)){
                rightFlipperOut = true;
            }



            if(leftFlipperOut){
                leftFlipper.setPosition(leftFlipperForward);
            } else {
                leftFlipper.setPosition(leftFlipperBack);
            }

            if(rightFlipperOut){
                rightFlipper.setPosition(rightFlipperForward);
            } else {
                rightFlipper.setPosition(rightFlipperBack);
            }











            ///------------------Movement code below------------------\\\


            switch (direction) {//set the motors at different speeds based off of the directions
                case "left forwards":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(0);

                    break;

                case "left backwards":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "left":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "right forwards":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "right backwards":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(0);

                    break;

                case "right":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "forwards":

                    frontLeftMotor.setPower(motorSpeed);
                    frontRightMotor.setPower(motorSpeed);
                    backLeftMotor.setPower(motorSpeed);
                    backRightMotor.setPower(motorSpeed);

                    break;

                case "backwards":

                    frontLeftMotor.setPower(-motorSpeed);
                    frontRightMotor.setPower(-motorSpeed);
                    backLeftMotor.setPower(-motorSpeed);
                    backRightMotor.setPower(-motorSpeed);

                    break;

                case "stop":

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    break;

                case "clockwise":

                    frontLeftMotor.setPower(spinSpeed);
                    frontRightMotor.setPower(-spinSpeed);
                    backLeftMotor.setPower(spinSpeed);
                    backRightMotor.setPower(-spinSpeed);

                    break;

                case "counter clockwise":

                    frontLeftMotor.setPower(-spinSpeed);
                    frontRightMotor.setPower(spinSpeed);
                    backLeftMotor.setPower(-spinSpeed);
                    backRightMotor.setPower(spinSpeed);

                    break;

                default:

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    break;

            }


            ///---------------End of Movement code--------------\\\

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        //this code will be executed when the robot stops
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }


}



