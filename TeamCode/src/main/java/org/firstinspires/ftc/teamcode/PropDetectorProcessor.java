package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropDetectorProcessor implements VisionProcessor, CameraStreamSource {

    Telemetry telemetry;

    public static double percentThreshold = 0.2;

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    Mat mat = new Mat();

    Mat red1 = new Mat();
    Mat red2 = new Mat();

    @Override
    public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    public static boolean red;

    Location location = Location.NOT_FOUND;

    Scalar lowHSVBlue = new Scalar(90, 80, 30);
    Scalar highHSVBlue = new Scalar(128, 255, 255);

    Scalar lowHSVRed1 = new Scalar(0, 80, 50);
    Scalar highHSVRed1 = new Scalar(10, 255, 255);
    Scalar lowHSVRed2 = new Scalar(160, 80, 50);
    Scalar highHSVRed2 = new Scalar(180, 255, 255);

    // These colors are for the rectangular boxes
    Scalar colorEmpty = new Scalar(0, 0, 0);
    Scalar colorProp = new Scalar(0, 0, 255);

    static final Rect LEFT_ROI = new Rect(
            new Point(10, 300),
            new Point(190, 415)
    );
    static final Rect CENTER_ROI = new Rect(
            new Point(110, 165),
            new Point(220, 210)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(490, 180),
            new Point(600, 225)
    );

    public PropDetectorProcessor(Telemetry telemetry, boolean r) {
        this.telemetry = telemetry;
        red = r;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) {
            location = Location.NOT_FOUND;
            return Location.NOT_FOUND;
        }

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        if (red) {
            Core.inRange(mat, lowHSVRed1, highHSVRed1, red1);
            Core.inRange(mat, lowHSVRed2, highHSVRed2, red2);

            Core.bitwise_or(red1, red2, mat);
        }
        else {
            Core.inRange(mat, lowHSVBlue, highHSVBlue, mat);
        }

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftRaw = Core.sumElems(left).val[0];
        double centerRaw = Core.sumElems(center).val[0];
        double rightRaw = Core.sumElems(right).val[0];

        double leftValue = leftRaw / LEFT_ROI.area() / 255;
        double centerValue = centerRaw / CENTER_ROI.area() / 255;
        double rightValue = rightRaw / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        Location currLocation = Location.NOT_FOUND;
        if (leftValue > centerValue && leftValue > rightValue && leftValue > percentThreshold) {
            currLocation =  Location.LEFT;
        } else if (centerValue > leftValue && centerValue > rightValue && centerValue > percentThreshold) {
            currLocation = Location.CENTER;
        } else if (rightValue > leftValue && rightValue > centerValue && rightValue > percentThreshold) {
            currLocation = Location.RIGHT;
        } else {
            currLocation = Location.NOT_FOUND;
        }

        telemetry.addData("Found prop at", currLocation);
        telemetry.addData("Detected left", leftValue);
        telemetry.addData("Detected center", centerValue);
        telemetry.addData("Detected right", rightValue);
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(frame, LEFT_ROI, currLocation == Location.LEFT ? colorProp : colorEmpty);
        Imgproc.rectangle(frame, CENTER_ROI, currLocation == Location.CENTER ? colorProp : colorEmpty);
        Imgproc.rectangle(frame, RIGHT_ROI, currLocation == Location.RIGHT ? colorProp : colorEmpty);

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

//        Imgproc.rectangle(mat, LEFT_ROI, currLocation == Location.LEFT ? colorProp : colorEmpty);
//        Imgproc.rectangle(mat, CENTER_ROI, currLocation == Location.CENTER ? colorProp : colorEmpty);
//        Imgproc.rectangle(mat, RIGHT_ROI, currLocation == Location.RIGHT ? colorProp : colorEmpty);
//
//        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
//        Utils.matToBitmap(mat, b);
//        lastFrame.set(b);

        location = currLocation;
        return currLocation;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint propPaint = new Paint();
//        propPaint.setColor(Color.BLUE);
//        propPaint.setStyle(Paint.Style.STROKE);
//        propPaint.setStrokeWidth(scaleCanvasDensity * 4);
//
//        Paint notPropPaint = new Paint(propPaint);
//        notPropPaint.setColor(Color.GRAY);
//
//        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(LEFT_ROI, scaleBmpPxToCanvasPx);
//        android.graphics.Rect drawRectangleCenter = makeGraphicsRect(CENTER_ROI, scaleBmpPxToCanvasPx);
//        android.graphics.Rect drawRectangleRight = makeGraphicsRect(RIGHT_ROI, scaleBmpPxToCanvasPx);
//
//        location = (Location) userContext;
//        switch (location) {
//            case LEFT:
//                canvas.drawRect(drawRectangleLeft, propPaint);
//                canvas.drawRect(drawRectangleCenter, notPropPaint);
//                canvas.drawRect(drawRectangleRight, notPropPaint);
//                break;
//            case CENTER:
//                canvas.drawRect(drawRectangleLeft, notPropPaint);
//                canvas.drawRect(drawRectangleCenter, propPaint);
//                canvas.drawRect(drawRectangleRight, notPropPaint);
//                break;
//            case RIGHT:
//                canvas.drawRect(drawRectangleLeft, notPropPaint);
//                canvas.drawRect(drawRectangleCenter, notPropPaint);
//                canvas.drawRect(drawRectangleRight, propPaint);
//                break;
//            case NOT_FOUND:
//                canvas.drawRect(drawRectangleLeft, notPropPaint);
//                canvas.drawRect(drawRectangleCenter, notPropPaint);
//                canvas.drawRect(drawRectangleRight, notPropPaint);
//                break;
//        }
    }

    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void close() {
        mat.release();
        red1.release();
        red2.release();
    }

    public Location getLocation() {
        return location;
    }
}
