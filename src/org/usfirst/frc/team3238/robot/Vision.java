package org.usfirst.frc.team3238.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Vision extends Thread {

    private static final double CAMERA_FIELD_OF_VIEW = 75;
    private static final int FRAME_WIDTH = 480;
    private static final int FRAME_HEIGHT = 320;

    private static final int CAMERA_EXPOSURE = 1;

    private static final int COLOR_MIN_H = 78;
    private static final int COLOR_MIN_S = 172;
    private static final int COLOR_MIN_V = 44;
    private static final int COLOR_MAX_H = 92;
    private static final int COLOR_MAX_S = 255;
    private static final int COLOR_MAX_V = 255;

    private static final double CONTOUR_MAX_AREA = 500;
    private static final double CONTOUR_MIN_AREA = 100;

    private static final double CONTOUR_MIN_HEIGHT = 100;
    private static final double CONTOUR_MAX_HEIGHT = 200;
    private static final double CONTOUR_MIN_WIDTH = 20;
    private static final double CONTOUR_MAX_WIDTH = 100;

    private static final double CONTOUR_MIN_RATIO = 0.1;
    private static final double CONTOUR_MAX_RATIO = 1;

    private static final double DISTANCE_TO_HEIGHT_CONVERSION = 0.5;

    static class VisionOutput {
        double angle;
        double distance;

        VisionOutput(double distance, double angle) {
            this.angle = angle;
            this.distance = distance;
        }
    }

    public interface VisionListener {
        void onFrameReady(VisionOutput output);
    }

    private CvSink sink;
    private VisionListener listener;

    private Mat sourceFrame;
    private Mat yCrCbFrame;
    private Mat yFrame;
    private Mat mask;
    private List<Mat> channels;
    private List<MatOfPoint> contours;
    private List<MatOfPoint> filteredContours;

    private boolean isProcessing = false;

    Vision(UsbCamera source, VisionListener listener) {
        super();
        this.listener = listener;

        source.setExposureManual(CAMERA_EXPOSURE);

        sink = CameraServer.getInstance().getVideo(source);
        sourceFrame = new Mat();
        yCrCbFrame = new Mat();
        yFrame = new Mat();
        mask = new Mat();

        channels = new ArrayList<>(3);
        contours = new ArrayList<>();
        filteredContours = new ArrayList<>();
    }

    void startProcessing() {
        isProcessing = true;
    }

    void stopProcessing() {
        isProcessing = false;
    }

    @Override
    public void run() {
        while (!Thread.interrupted()) {
            if (isProcessing) {
                double distance;
                double angle;

                double timestamp = Timer.getFPGATimestamp();

                try {
                    sink.grabFrame(sourceFrame);

                    Imgproc.resize(sourceFrame, sourceFrame, new Size(FRAME_WIDTH, FRAME_HEIGHT));

//                    Imgproc.cvtColor(sourceFrame, yCrCbFrame, Imgproc.COLOR_RGB2YCrCb);
//                    Core.split(yCrCbFrame, channels);
//                    Imgproc.equalizeHist(channels.get(0), yFrame);
//                    channels.set(0, yFrame);
//                    Core.merge(channels, yCrCbFrame);
//
//
//                    Imgproc.cvtColor(yCrCbFrame, sourceFrame, Imgproc.COLOR_YCrCb2RGB);
                    Imgproc.cvtColor(sourceFrame, sourceFrame, Imgproc.COLOR_RGB2HSV);

                    Core.inRange(sourceFrame, new Scalar(COLOR_MIN_H, COLOR_MIN_S, COLOR_MIN_V),
                            new Scalar(COLOR_MAX_H, COLOR_MAX_S, COLOR_MAX_V), mask);

                    contours.clear();
                    filteredContours.clear();
                    Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (MatOfPoint contour : contours) {
                        double contourArea = Imgproc.contourArea(contour);

                        if (contourArea < CONTOUR_MIN_AREA || contourArea > CONTOUR_MAX_AREA) continue;

                        Rect bound = Imgproc.boundingRect(contour);

                        if (bound.height < CONTOUR_MIN_HEIGHT || bound.height > CONTOUR_MAX_HEIGHT) continue;
                        if (bound.width < CONTOUR_MIN_WIDTH || bound.width > CONTOUR_MAX_WIDTH) continue;

                        double ratio = bound.width / bound.height;

                        if (ratio < CONTOUR_MIN_RATIO || ratio > CONTOUR_MAX_RATIO) continue;

                        filteredContours.add(contour);
                    }

                    filteredContours.sort((o1, o2) -> {
                        double area1 = Imgproc.contourArea(o1);
                        double area2 = Imgproc.contourArea(o2);

                        return Double.compare(area1, area2);

                    });

                    Rect bounds1 = Imgproc.boundingRect(filteredContours.get(0));
                    Rect bounds2 = Imgproc.boundingRect(filteredContours.get(1));

                    double centerX1 = bounds1.x + (0.5 * bounds1.width);
                    //double centerY1 = bounds1.y + (0.5 * bounds1.height);
                    double centerX2 = bounds2.x + (0.5 * bounds2.width);
                    //double centerY2 = bounds2.y + (0.5 * bounds2.height);

                    double centerXOffset = (centerX1 - centerX2) / 2;
                    //double centerYOffset = (centerY1 - centerY2) / 2;

                    double centerX = Math.min(centerX1, centerX2) + centerXOffset;
                    //double centerY = Math.min(centerY1, centerY2) + centerYOffset;

                    double height = (bounds1.height + bounds2.height) / 2;

                    angle = ((centerX - (FRAME_WIDTH / 2)) / (FRAME_WIDTH / 2)) * CAMERA_FIELD_OF_VIEW;
                    distance = height * DISTANCE_TO_HEIGHT_CONVERSION;

                    listener.onFrameReady(new VisionOutput(distance, angle));

                    double time = Timer.getFPGATimestamp() - timestamp;
                    DriverStation.reportWarning("Vision FPS: " + (1 / time), false);

                } catch (Exception e) {
                    DriverStation.reportError(e.getMessage(), true);
                }
            }
        }
    }
}

