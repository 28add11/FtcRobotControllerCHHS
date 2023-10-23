/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import android.util.Size;
import org.opencv.imgproc.Imgproc;


import java.lang.Math;
import java.util.List;
import java.util.concurrent.TimeUnit;
import android.graphics.Canvas;


@TeleOp
public class CenterstageAuto extends LinearOpMode
{

    // Computer vision stuff
    AprilTagProcessor aprilTags;
    //VisionPortal.Builder VisionPortalBuilder;
    VisionPortal visionPortal;


    // Declare OpMode members for each of the 2 motors.
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double ROTATION_SPEED = 0.6;
    // Servo stuff

    // Define class members

    //Servo stuff end

    // Method that simplifies instruction for movement, math required to determine power is done here
    // Is a copy of math done from a template, and is in use in our main program

    // Turn is emulating the right joystick X value, drive emulating left Y value

    // If we want we could theoretically record someone's movements for auto,
    // but realistically that would be pointless this season
    public void setMotorInstruction(double turn, double drive) {

        double leftPower;
        double rightPower;

        // POV Mode uses left joystick to go forward & back, and right joystick to rotate.
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Left", "%4.2f", leftPower);
        telemetry.addData("Right", "%4.2f", rightPower);
        //telemetry.update();
    }

    public void driveDistance(double distance, double speed){
        // Reset the encoder
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int count = (distance/(Math.PI*0.1))*732 //Distance in meters

        leftMotor.setTargetPosition(count/2);
        rightMotor.setTargetPosition(count/2);

        // Switch to RUN_TO_POSITION mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //SPEED
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //this makes the motors continue to run until they reach the correct encoder value.
    }

    // Non color sensor stuff
    double timeX;
    double timeY;
    float externalJunctionPointX;
    float externalJunctionPointY;

    @Override
    public void runOpMode()
    {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor  = hardwareMap.get(DcMotor.class, "rightMotor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Computer vision initialization

        // Create the AprilTag processor and assign it to a variable.
        aprilTags = AprilTagProcessor.easyCreateWithDefaults();

        // Create a new VisionPortal.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(new autoPipeline())
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            /*
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.update();
            */

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }

    }


    class autoPipeline implements VisionProcessor
    {

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration)
        {

        }
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
        {

        }
        @Override
        public Mat processFrame(Mat input, long captureTimeNanos)
        {
            return input;
        }
    }

}
