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

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Omnidirectional Drive Auto RED CENTER", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class Omnidirectional_Drive_Auto_RED_CENTER extends LinearOpMode {

    MediaPlayer spidersclaw;
    MediaPlayer kahoot1;
    MediaPlayer kahoot2;
    MediaPlayer kahoot3;


    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    Servo leftFlipper = null;
    Servo rightFlipper = null;
    Servo loader = null;
    DcMotor shootMotor = null;
    ColorSensor leftColor;  // Hardware Device Object
    ColorSensor rightColor;
    TouchSensor leftTouch;
    TouchSensor rightTouch;
    OpticalDistanceSensor lineReader;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        MediaPlayer spidersclaw = MediaPlayer.create(hardwareMap.appContext,R.raw.spidersclaw);
        MediaPlayer kahoot1 = MediaPlayer.create(hardwareMap.appContext,R.raw.kahoot1);
        MediaPlayer kahoot2 = MediaPlayer.create(hardwareMap.appContext,R.raw.kahoot2);
        MediaPlayer kahoot3 = MediaPlayer.create(hardwareMap.appContext,R.raw.kahoot3);
        double songSelection  = 0;
        int songCount = 4;

        //variable setup
        double buffer = 0.25;               //how far the joystick must move before moving the motors
        String direction = "stop";          //the direction the robot will be heading
        double motorSpeed = 0.25;           //the power the motors will be set to
        int maxSpeed = 2800;
        double spinSpeed = 0.25;            //the power the motors will be set to while spinning
        double ticksPerRev = 1120;          //the amount of encoder ticks per revolution of the wheel

        int shootTimer = 0;                 //how long the gun has been shooting for
        double shootSpeed = 1;              //how fast the gun shoots at
        //int shootMax = 4500;                //how far the gun shoots
        int shootMax = 1000;
        int shootMin = 0;
        int loadTimer = 0;
        int loadTime = 4000;

        boolean colorSensorLEDOn = false;    //if the color sensor LED is on or not
        boolean buttonPressed1 = false;     //if a,b,x, or y is pressed on gamepad1

        double leftFlipperBack = 0.25;      //position of the left flipper when retracted
        double leftFlipperForward = 0.0;    //position of the left flipper when extended
        boolean leftFlipperOut = false;     //if the left flipper is out or not

        double rightFlipperBack = 0.75;     //position of the right flipper when retracted
        double rightFlipperForward = 1.0;   //position of the right flipper when extended
        boolean rightFlipperOut = false;    //if the right flipper is out or not

        double loaderDown = 0.5;            //position of the loader when retracted
        double loaderUp = 0.0;              //position of the loader when hitting the ball
        boolean loaderOut = false;          //if the loader is out or not


        //auto variables
        String leftColorSensed = "none";
        String rightColorSensed = "none";

        String SIDE_COLOR = "red"; // <======================================================================
        String PARK_LOCATION = "center"; //options are center, ramp, none
        String colorToPress = "none";

        boolean wallReached = false;
        boolean side1Moved = false;
        boolean line1Detected = false;
        boolean beacon1Pressed = false;
        boolean shooting = true;
        boolean side2Moved = false;
        boolean line2Detected = false;
        boolean beacon2Pressed = false;
        boolean parked = false;
        boolean backedUp = false;
        boolean turned = false;
        boolean isTeamColor = false;


        int moveSideTimer = 0;
        int moveSideTime = 1000;

        int pushCycle = 0;
        int maxPushes = 3;
        int pushTimer = 0;
        int pushTime = 750;

        int parkTimer = 0;
        int parkCenterTime = 6000;
        int parkRampTime = 8000;

        int backUpTimer = 0;
        int backUpTime = 1000;

        int turnTimer = 0;
        int turnTime = 2000;

        double lineLight = 1;                 //the value at which the line gives off light

        //motor setup
        frontLeftMotor = hardwareMap.dcMotor.get("front left");
        frontRightMotor = hardwareMap.dcMotor.get("front right");
        backLeftMotor = hardwareMap.dcMotor.get("back left");
        backRightMotor = hardwareMap.dcMotor.get("back right");

        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");
        loader = hardwareMap.servo.get("loader");

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

        //intialize servo positions
        leftFlipper.setPosition(leftFlipperBack);
        rightFlipper.setPosition(rightFlipperBack);
        loader.setPosition(loaderDown);

        waitForStart();
        runtime.reset();

        songSelection = (int)(Math.random()*4.0 + 0.5);
        if(songSelection == 1){
            spidersclaw.start();
        } else if (songSelection == 2){
            kahoot1.start();
        } else if (songSelection == 3){
            kahoot2.start();
        } else if (songSelection == 4){
            kahoot3.start();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction :", direction);
            telemetry.addData("Song Selection: ", songSelection);
            telemetry.addData("Goals: " , "Side-" + SIDE_COLOR + " Target-" + PARK_LOCATION);
            //telemetry.addData("Motor Speed: ", motorSpeed);
            //telemetry.addData("Spin Speed: ", spinSpeed);
            telemetry.addData("L color - ", "Red: " + leftColor.red() + "Green: " + leftColor.green() + "Blue: " + leftColor.blue() + leftColorSensed);
            telemetry.addData("R color - ", "Red: " + rightColor.red() + "Green: " + rightColor.green() + "Blue: " + rightColor.blue() + rightColorSensed);
            telemetry.addData("Color to press: ", colorToPress);
            telemetry.addData("Flippers - ", "Right " + rightFlipperOut + " Left " + leftFlipperOut);
            //telemetry.addData("ColorSensor LED: ", colorSensorLEDOn);
            //telemetry.addData("Touch - ", "left: " + leftTouch.isPressed() + " right: " + rightTouch.isPressed());
            telemetry.addData("ODS: ", lineReader.getRawLightDetected());
            telemetry.addData("Encoders: ", "FL " + frontLeftMotor.getCurrentPosition() + " FR " + frontRightMotor.getCurrentPosition() + " BL " + backLeftMotor.getCurrentPosition() + " BR " + backRightMotor.getCurrentPosition());
            telemetry.addData("Shooting ", "Timer: " + shootTimer + " Pos: " + shootMotor.getCurrentPosition());
            //telemetry.addData("Triggers: ", "L2: " + gamepad2.left_trigger + " R2: " + gamepad2.right_trigger);
            telemetry.update();




            ///-----------------------------autonomous logik---------------------\\\
            if ((!leftTouch.isPressed() && !rightTouch.isPressed()) && !wallReached) {//moves diagonally untill a button is pressed

                motorSpeed = 0.8;

                if (SIDE_COLOR.equals("blue")) {
                    direction = "left forwards";
                } else {
                    direction = "right forwards";
                }
                /*
            } else if(!leftTouch.isPressed() || !rightTouch.isPressed()){//always will move towards wall if a button is pressed
                motorSpeed = 0.1;
                direction = "right";
                */
            } else if (moveSideTimer < moveSideTime && !side1Moved) {//moves away from beacon for a set amount of frames

                wallReached = true;
                motorSpeed = 0.5;
                moveSideTimer++;

                if (SIDE_COLOR.equals("blue")) {
                    direction = "right";
                } else {
                    direction = "left";
                }
            } else if (lineReader.getRawLightDetected() < lineLight && !line1Detected) {//moves towards beacons untill it sees the line

                motorSpeed = 0.3;
                side1Moved = true;

                if (SIDE_COLOR.equals("blue")) {
                    direction = "left";
                } else {
                    direction = "right";
                }
            } else if (!isTeamColor && !beacon1Pressed && pushTimer < pushTime && pushCycle % 2 == 0 && pushCycle < 2 * maxPushes) {//presses beacon color that it is supposed to  and moves towards wall
                side1Moved = true;
                line1Detected = true;
                moveSideTimer = 0;
                if (!leftTouch.isPressed() || !rightTouch.isPressed()) {//always will move towards wall if a button is pressed
                    motorSpeed = 0.1;
                    direction = "forwards";
                    colorToPress = "none";
                } else {
                    colorToPress = SIDE_COLOR;
                    direction = "stop";
                    motorSpeed = 0.3;
                    shootTimer = 1;
                    pushTimer++;
                }
            } else if (!isTeamColor && !beacon1Pressed && pushTimer < pushTime && pushCycle % 2 == 1 && pushCycle < 2 * maxPushes) {//going backwards
                line1Detected = true;
                moveSideTimer = 0;
                if (!leftTouch.isPressed() || !rightTouch.isPressed()) {//always will move towards wall if a button is pressed
                    motorSpeed = 0.1;
                    direction = "forwards";
                    colorToPress = "none";
                } else {
                    colorToPress = "none";
                    direction = "stop";
                    shootTimer = 1;
                    pushTimer++;
                }
            } else if (!isTeamColor && !beacon1Pressed && pushTimer >= pushTime) {
                pushTimer = 0;
                pushCycle++;
            } else if ((isTeamColor || pushCycle >= 2 * maxPushes) && !beacon1Pressed) {
                beacon1Pressed = true;
                shooting = true;
            } else if (shooting) {

                direction = "stop";
                colorToPress = "none";

                pushTimer = 0;
                pushCycle = 0;
                //auto shooting

                /*
                if (shootTimer == 1){
                    shootMotor.setPower(shootSpeed);
                    if (shootMotor.getCurrentPosition() > shootMax) {
                        shootTimer = 2;
                    }
                } else {
                    shootMotor.setPower(0);
                    shooting = false;
                    shootTimer = 0;
                }
                */


                if (shootTimer == 1) {
                    shootMotor.setPower(shootSpeed);
                    if (shootMotor.getCurrentPosition() > shootMax) {
                        shootTimer = 2;
                    }
                } else if (shootTimer == 2) {
                    loaderOut = true;
                    shootMotor.setPower(0);
                    if (loadTimer > loadTime) {
                        shootTimer = 3;
                    } else {
                        loadTimer++;
                    }
                } else if (shootTimer == 3) {
                    shootMotor.setPower(shootSpeed);
                    if (shootMotor.getCurrentPosition() > 1.5 * shootMax + ticksPerRev) {
                        shootTimer = 4;
                    }
                } else {
                    shootMotor.setPower(0);
                    shooting = false;
                    loadTimer = 0;
                    loaderOut = false;
                }

            } else if (moveSideTimer < moveSideTime && !side2Moved) {//moves towards other beacon for a set time to prevent line reader from activating too early
                colorToPress = "none";
                moveSideTimer++;
                motorSpeed = 0.75;
                if (SIDE_COLOR.equals("blue")) {
                    direction = "left";
                } else {
                    direction = "right";
                }
            } else if (lineReader.getRawLightDetected() < lineLight && !line2Detected) {//moves towards second beacon until line is read
                side2Moved = true;
                motorSpeed = 0.3;
                if (SIDE_COLOR.equals("blue")) {
                    direction = "left";
                } else {
                    direction = "right";
                }
            } else if (!isTeamColor && !beacon2Pressed && pushTimer < pushTime && pushCycle % 2 == 0 && pushCycle < 2 * maxPushes) {//presses beacon color that it is supposed to  and moves towards wall
                line2Detected = true;
                moveSideTimer = 0;
                if (!leftTouch.isPressed() || !rightTouch.isPressed()) {//always will move towards wall if a button is pressed
                    motorSpeed = 0.1;
                    direction = "forwards";
                    colorToPress = "none";
                } else {
                    colorToPress = SIDE_COLOR;
                    direction = "stop";
                    shootTimer = 1;
                    pushTimer++;
                }
            } else if (!isTeamColor && !beacon2Pressed && pushTimer < pushTime && pushCycle % 2 == 1 && pushCycle < 2 * maxPushes) {//going backwards
                line1Detected = true;
                moveSideTimer = 0;
                if (!leftTouch.isPressed() || !rightTouch.isPressed()) {//always will move towards wall if a button is pressed
                    motorSpeed = 0.1;
                    direction = "forwards";
                    colorToPress = "none";
                } else {
                    colorToPress = "none";
                    direction = "stop";
                    shootTimer = 1;
                    pushTimer++;
                }
            } else if (!isTeamColor && !beacon2Pressed && pushTimer >= pushTime) {
                pushTimer = 0;
                pushCycle++;
            } else if ((isTeamColor || pushCycle >= 2 * maxPushes) && !beacon2Pressed) {
                beacon2Pressed = true;
                colorToPress = "none";
                moveSideTimer = 0;
            } else if (parkTimer < parkCenterTime && !parked && PARK_LOCATION.equals("center")) {
                beacon2Pressed = true;
                motorSpeed = 0.8;
                parkTimer++;
                if (SIDE_COLOR.equals("blue")) {
                    direction = "right backwards";
                } else {
                    direction = "left backwards";
                }
            } else if (backUpTimer < backUpTime && !backedUp && PARK_LOCATION.equals("ramp")) {
                beacon2Pressed = true;
                motorSpeed = 0.8;
                backUpTimer++;
                direction = "backwards";
            } else if (turnTimer < turnTime && !turned && PARK_LOCATION.equals("ramp")) {
                backedUp = true;
                turnTimer++;
                motorSpeed = 0.25;
                if (SIDE_COLOR.equals("blue")) {
                    direction = "clockwise";
                } else {
                    direction = "counter clockwise";
                }
            } else if (parkTimer < parkRampTime && !parked && PARK_LOCATION.equals("ramp")) {
                turned = true;
                motorSpeed = 0.8;
                parkTimer++;
                if (SIDE_COLOR.equals("blue")) {
                    direction = "right forwards";
                } else {
                    direction = "left forwards";
                }

            } else { //stops
                beacon2Pressed = true;
                parked = true;
                pushTimer = 0;
                colorToPress = "none";
                direction = "stop";
                motorSpeed = 0.25;
            }
            ///------------------ end of autonomus logik --------------------------\\\


            if (leftColor.red() > leftColor.blue()) {
                leftColorSensed = "red";
            } else if (leftColor.blue() > leftColor.red()) {
                leftColorSensed = "blue";
            } else {
                leftColorSensed = "none";
            }

            if (rightColor.red() > rightColor.blue()) {
                rightColorSensed = "red";
            } else if (rightColor.blue() > rightColor.red()) {
                rightColorSensed = "blue";
            } else {
                rightColorSensed = "none";
            }


            if (leftColorSensed.equals("none") || colorToPress.equals("none")) {
                leftFlipperOut = false;
            } else if (leftColorSensed.equals(colorToPress)) {
                leftFlipperOut = true;
            }

            if (rightColorSensed.equals("none") || colorToPress.equals("none")) {
                rightFlipperOut = false;
            } else if (rightColorSensed.equals(colorToPress)) {
                rightFlipperOut = true;
            }

            if (!rightColorSensed.equals("none") && !leftColorSensed.equals("none") && !colorToPress.equals("none")) {
                if (rightColorSensed.equals(colorToPress) && leftColorSensed.equals(colorToPress)) {
                    leftFlipperOut = false;
                    rightFlipperOut = false;
                    isTeamColor = true;
                }
            } else {
                isTeamColor = false;
            }


            if (leftFlipperOut) {
                leftFlipper.setPosition(leftFlipperForward);
            } else {
                leftFlipper.setPosition(leftFlipperBack);
            }

            if (rightFlipperOut) {
                rightFlipper.setPosition(rightFlipperForward);
            } else {
                rightFlipper.setPosition(rightFlipperBack);
            }

            if (loaderOut) {
                loader.setPosition(loaderUp);
            } else {
                loader.setPosition(loaderDown);
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