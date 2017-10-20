package org.usfirst.frc.team3238.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot implements Vision.VisionListener {

    private Vision visionSystem;

    private Joystick joy;

    @Override
    public void robotInit() {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        visionSystem = new Vision(camera, this);
        visionSystem.start();

        joy = new Joystick(0);
    }

    @Override
    public void robotPeriodic() {
        if (joy.getRawButton(11)) visionSystem.stopProcessing();
        else visionSystem.startProcessing();
    }

    @Override
    public void onFrameReady(Vision.VisionOutput output) {
        DriverStation.reportWarning("Vision frame complete:\n\tDistance: " + output.distance + "\n\tAngle: "
                + output.angle, false);
    }
}
