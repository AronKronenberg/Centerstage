package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropDetectorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Prop Detector Sample")
public class PropDetectorSample extends LinearOpMode {
    private PropDetectorProcessor propDetector;
    private VisionPortal visionPortal;

    public static PropDetectorProcessor.Location spikeMarkLocation = PropDetectorProcessor.Location.NOT_FOUND;

    boolean detectRed = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize prop detecting processor and vision portal
        // Tell propDetector if we are red are blue so it can look for the right color
        propDetector = new PropDetectorProcessor(telemetry, detectRed);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        waitForStart();

        while (opModeIsActive()) {
            // Get the prop location detected by the prop detector processor
            spikeMarkLocation = propDetector.getLocation();
            telemetry.addData("Prop Location", spikeMarkLocation);
            telemetry.update();
        }
    }
}
