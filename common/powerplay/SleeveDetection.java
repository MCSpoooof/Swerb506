package org.firstinspires.ftc.teamcode.Swerb506.common.powerplay;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions
    private Point sleeve_pointA;
    private Point sleeve_pointB;

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;
    private boolean isLeft;

    public SleeveDetection() {
        isLeft = (Side.LEFT == Globals.SIDE);
        regenerate();
    }

    @Override
    public Mat processFrame(Mat input) {
        regenerate();
        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );

        } else if (sumColors.val[1] == minColor) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        } else {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        }

        areaMat.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
    
    public void regenerate() {
        sleeve_pointA = (isLeft) ? new Point(LEFTSIDE_REGION_X, LEFTSIDE_REGION_Y) : new Point(RIGHTSIDE_REGION_X, RIGHTSIDE_REGION_Y);
        sleeve_pointB = (isLeft) ? new Point(LEFTSIDE_REGION_X + REGION_WIDTH, LEFTSIDE_REGION_Y + REGION_HEIGHT) : new Point(RIGHTSIDE_REGION_X + REGION_WIDTH, RIGHTSIDE_REGION_Y + REGION_HEIGHT);
    }
}
