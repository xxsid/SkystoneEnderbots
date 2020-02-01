package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp(name="Potentiometer test", group="Iterative Opmode")

public class Potentiometer_Test extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private AnalogInput potentiometer = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
    potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Voltage:", potentiometer.getVoltage());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

