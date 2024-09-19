package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Vision Processor Test")
public class VisionProcessorTest extends LinearOpMode {
    private PropDetectorProcessor propProcessor;
    private VisionPortal visionPortal;

    boolean red = true;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propProcessor = new PropDetectorProcessor(telemetry, red);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propProcessor);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard.startCameraStream(propProcessor, 0);

        telemetry.addData("Location", propProcessor.getLocation());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", propProcessor.getLocation());
            telemetry.update();
        }

        visionPortal.stopStreaming();
    }
}
