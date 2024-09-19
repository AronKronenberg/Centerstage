package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropDetectorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Prop Detector")
public class PropDetector extends LinearOpMode {

    public static boolean detectRed = false;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    VisionPortal visionPortal;
    PropDetectorProcessor propDetector;

    PropDetectorProcessor.Location spikeMarkLocation = PropDetectorProcessor.Location.NOT_FOUND;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propDetector = new PropDetectorProcessor(telemetry, detectRed);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        dashboard.startCameraStream(propDetector, 0);

        waitForStart();

        while (opModeIsActive()) {
            PropDetectorProcessor.red = detectRed;

            spikeMarkLocation = propDetector.getLocation();

            telemetry.addData("Prop Location", spikeMarkLocation);
            telemetry.update();
        }
    }
}
