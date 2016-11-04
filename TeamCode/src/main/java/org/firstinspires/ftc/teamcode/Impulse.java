package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by alex on 10/19/16.
 * <p>
 * The API of methods used by Team10809.
 */

public class Impulse {

    Telemetry telemetry;

    public void hardwareCheck(HardwareDevice hardwareDevice) throws TelemetryException {
        try {
            telemetry.addData("Name: ", hardwareDevice.getDeviceName());
            telemetry.addData("Connection Info: ", hardwareDevice.getConnectionInfo());
        } catch (NullPointerException n) {
            throw new TelemetryException();
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}

class TelemetryException extends Exception {
    public TelemetryException() {
        throw new NullPointerException("Telemetry was never defined in Impulse. Use the setTelemetry() method before using Impulse.");
    }

}