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

//import android.graphics.Point; caused error in later imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;


@TeleOp
public class PoleDetectionOpMode extends LinearOpMode
{
    OpenCvWebcam webcam;


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


    // Non color sensor stuff
    boolean cameraError = false;
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

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();


        // Drive forward
        setMotorInstruction(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Stop
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {

        }


        if(colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) //red is location 1
        {
            scenario = 0;
        }

        if(colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) //green is location 2
        {
            scenario = 1;
        }
        
        if(colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) //blue is location 3
        {
            scenario = 2;
        }


        // In order to prevent the cones from getting stuck in the wheels, this code
        // pushes the cone forward and moves back to its original spot
        setMotorInstruction(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Run it back
        setMotorInstruction(0, FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }

        if (!cameraError)
        {
            // Rotate to let camera see
            telemetry.addData("Step", "Beginning rotation");
            telemetry.update();
            setMotorInstruction(0, 0, -ROTATION_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.15)) {

            }
            telemetry.addData("Step", "Rotation end");
            telemetry.update();
            if (true == true)//!DetectPoles.detectError()) commented out, was giving me an error.
            {
                telemetry.addData("Step", "Driving forward");
                telemetry.update();
                // Clever little bit of code to avoid ifs. Gets sign of where we want the point - the actual point, then moves in the appropreate direction
                float signX = Math.signum(175 - externalJunctionPointX); //formerly 320
                setMotorInstruction(0, FORWARD_SPEED * signX * 0.5, 0);
                runtime.reset();
                while (opModeIsActive() && !(externalJunctionPointX >= 165 && externalJunctionPointX <= 185)) // loop until detectPoles.getTargetPointX is between 310 and 330
                {
                    telemetry.addData("Funni number 1", externalJunctionPointX);
                    telemetry.update();
                }
                timeX = runtime.seconds();




                setSlidePos(1.0);

                setMotorInstruction(0, 0, 0);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.5))
                {

                }

                setSlidePos(0.0);

            }
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class DetectPoles extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        //Note: I have literally no clue what various qualities mats should have, I feel like you might want to make them a volatile but the docs do it this way so ¯\_(ツ)_/¯

        Mat poles = new Mat(); //Final output mat to be displayed on screen, THIS IS NOT DATA FOR THE OPMODE, OPMODE DATA WILL BE GATHERED IN THE PIPELINE!
        Mat HSVsource = new Mat();
        Mat hierarchy = new Mat(); //It needs this for some reason
        Mat output = new Mat();
        Mat RGBsource = new Mat();
        Mat median = new Mat();

        // Declaring variables to interface with the opmode

        Point junctionPoint = new Point(0, 0);
        boolean noneDetected;

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */


            /* Basic documentation can be found at docs.opencv.org, however they are a bit hard to understand. If any help is needed reach out to 28+11#9929 On discord.
             * 
             * If anyone is maintaining this repo in the future when I have left Crescent please sub in your own discord tag.
             *
             */

            output = input.clone();

            Imgproc.medianBlur(input, median, 3); //apply a median filter to reduce noise

            //UNTESTED AND THE DOCS WERE BAD!!!!
            Imgproc.cvtColor(median, RGBsource, Imgproc.COLOR_RGBA2RGB); //Convert RGBA colorspace of input into RGB
            Imgproc.cvtColor(RGBsource, HSVsource, Imgproc.COLOR_RGB2HSV);

            /* Change the Scalars to modify parameters. In HSV colorspace. First Scalar is min value, second is max 
             * Values are gotten from an online color picker, (H/360) * 255 for the hue to put in the code, ([S, V]/100) * 100 to get saturation or value for the code
            */
            Core.inRange(HSVsource, new Scalar(20, 105, 105), new Scalar(43, 255, 255), poles); //Looks at every pixel of HSVsource, sees if it is between the two scalars, 255 if it is, 0 if it isnt

            java.util.List<MatOfPoint> contours = new java.util.ArrayList<MatOfPoint>();

            Imgproc.findContours(poles, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); //finds contours, meaning edges. should work in practice

            if (contours == null || contours.isEmpty()) {

                noneDetected = true;
                return output; //both of these situations are bad, so just flag a non fatal error, and leave

            } else {

                noneDetected = false; // else case for clarity

            }


            Imgproc.drawContours(output, contours, -1, new Scalar(255, 0, 0), 2);

            //I got help with the following code from LHACK4142#7686 on discord, many thanks to him!


            //finds the biggest pole, ie the closest, since it will take up the largest portion of the FOV
            MatOfPoint biggestContour = contours.get(0); // this is because using the default constructor sets biggestContour up in a bad way

            for (MatOfPoint curContour : contours) {
                if (Imgproc.contourArea(curContour) > Imgproc.contourArea(biggestContour)) {
                    biggestContour = curContour;
                }
            }


            //finds centroid of contour
            Moments moments = Imgproc.moments(biggestContour);
            junctionPoint = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());


            externalJunctionPointX = (float)(moments.get_m10() / moments.get_m00());
            externalJunctionPointY = (float)(moments.get_m01() / moments.get_m00());

            // 1/area gets the distance to the contour

            Imgproc.circle(output, junctionPoint, 8, new Scalar(0, 255, 0), -1);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */




            return output;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }


        public void getTargetPointX()
        {
            //return junctionPoint.X();
        }

        public void getTargetPointY()
        {
            //return junctionPoint.Y();
        }

        public boolean detectError()
        {
            return noneDetected;
        }

    }
}