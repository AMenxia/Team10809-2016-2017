package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by alex on 10/19/16.
 * <p>
 * The API of methods used by Team10809.
 */

public class Impulse {

    Crow crow;

    public void hardwareCheck(HardwareDevice hardwareDevice) {
        crow.telemetry.addData("Name: ", hardwareDevice.getDeviceName());
        crow.telemetry.addData("Connection Info: ", hardwareDevice.getConnectionInfo());
    }

    public void stop() {
        crow.frontLeftMotor.setPower(0);
        crow.frontRightMotor.setPower(0);
        crow.backLeftMotor.setPower(0);
        crow.backRightMotor.setPower(0);
    }

    public void setCrow(Crow crow) {
        this.crow = crow;
    }

}