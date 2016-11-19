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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Omnidirectional Drive Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftFlipper = null;
    Servo rightFlipper = null;

    DcMotor shootMotor = null;

    ColorSensor colorSensor;  // Hardware Device Object
    TouchSensor leftTouch;
    TouchSensor rightTouch;
    OpticalDistanceSensor lineReader;

    Impulse i = new Impulse();

    @Override
    public void runOpMode() throws InterruptedException {

        //variable setup
        String direction = "stop";          //the direction the robot will be heading
        double motorSpeed = 0.25;           //the power the motors will be set to
        double spinSpeed = 0.25;            //the power the motors will be set to while spinning

        boolean shooting = false;           //if the robot is in the process of shooting
        int shootTimer = 0;                 //how long the gun has been shooting for
        double shootSpeed = 0.25;           //how fast the gun shoots at
        int pullBackTime = 100;             //how long the gun pulls back for

        boolean colorSensorLEDOn = true;    //if the color sensor LED is on or not
        boolean buttonPressed1 = false;     //if a,b,x, or y is pressed on gamepad1

        double leftFlipperBack = 0.25;      //position of the left flipper when retracted
        double leftFlipperForward = 0.0;    //position of the left flipper when extended
        boolean leftFlipperOut = false;     //if the left flipper is out or not

        double rightFlipperBack = 0.75;     //position of the right flipper when retracted
        double rightFlipperForward = 1.0;   //position of the right flipper when extended
        boolean rightFlipperOut = false;    //if the right flipper is out or not

        String colorSensed = "none";

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");
        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");

        shootMotor = hardwareMap.dcMotor.get("shoot");

        Crow crow = new Crow(telemetry, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftFlipper, rightFlipper, shootMotor, colorSensor);
        Impulse i = new Impulse();
        i.setCrow(crow);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);

        //sensor setup
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        leftTouch = hardwareMap.touchSensor.get("left touch");
        rightTouch = hardwareMap.touchSensor.get("right touch");
        lineReader = hardwareMap.opticalDistanceSensor.get("ods");

        colorSensor.enableLed(colorSensorLEDOn);//turn on the color sensor light when init
        telemetry.addData("Status", "Initialized");//tell that everything is started
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :", direction);
            telemetry.addData("Motor Speed: ", motorSpeed);
            telemetry.addData("Spin Speed: ", spinSpeed);
            telemetry.addData("Colors - ", "Red: " + colorSensor.red() + "Green: " + colorSensor.green() + "Blue: " + colorSensor.blue());
            telemetry.addData("Color Sensed: ", colorSensed);
            telemetry.addData("Flippers - ", "Right " + rightFlipperOut + " Left " + leftFlipperOut);
            telemetry.addData("ColorSensor LED: ", colorSensorLEDOn);
            telemetry.addData("Touch - ", "left: " + leftTouch + " right: " + rightTouch);
            telemetry.addData("ODS: ", lineReader.getRawLightDetected());
            telemetry.update();


            //autonomous logik
            /*
            if (getRuntime() <  3){
                direction = "forwards";
            } else if (getRuntime() < 5){
                direction = "clockwise";
            } else if (getRuntime() < 7){
                direction = "counter clockwise";
            } else {
                direction = "stop";
            }
            */

            if (colorSensor.red()> colorSensor.blue()){
                colorSensed = "red";
            } else if (colorSensor.blue() > colorSensor.red()){
                colorSensed = "blue";
            }

            if(gamepad2.x){
                if (colorSensed == "blue"){
                    leftFlipperOut = true;
                    rightFlipperOut = false;
                } else if (colorSensed == "red"){
                    leftFlipperOut = false;
                    rightFlipperOut = true;
                } else {
                    leftFlipperOut = false;
                    rightFlipperOut = false;
                }
            } else

            if(gamepad2.b){
                if (colorSensed == "red"){
                    leftFlipperOut = true;
                    rightFlipperOut = false;
                } else if (colorSensed == "blue"){
                    leftFlipperOut = false;
                    rightFlipperOut = true;
                } else {
                    leftFlipperOut = false;
                    rightFlipperOut = false;
                }
            } else {
                leftFlipperOut = false;
                rightFlipperOut = false;
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



