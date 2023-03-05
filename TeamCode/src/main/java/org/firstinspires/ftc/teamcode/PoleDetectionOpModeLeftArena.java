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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;


/* WORK ON THIS LATER, IT'S TIME TO SLEEP! 

public class colorSensorDrive extends LinearOpMode {


    

    


    @Override
    public void runOpMode() {



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way





        // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();


        // Park RED
        if(colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) //red is location 1
        {
            scenario = 0;

            



            //drive left
            setMotorInstruction(-FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        }

        // Park GREEN
        if(colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) //green is location 2
        {
            scenario = 1;

            // Do nothing lmao
        }

        // Park BLUE
        if(colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) //blue is location 3
        {
            scenario = 2;

            // In order to prevent the cones from getting stuck in the wheels, this code
            // pushes the cone forward and moves back to its original spot
            setMotorInstruction(0, -FORWARD_SPEED, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Run it back
            setMotorInstruction(0, FORWARD_SPEED, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }



            // Drive right
            setMotorInstruction(FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        }

        telemetry.addData("Scenario", scenario);

        telemetry.update();

        // Step 2:  Stop
        setMotorInstruction(0, 0, 0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
*/




@Autonomous
public class PoleDetectionOpModeLeftArena extends LinearOpMode
{
    OpenCvWebcam webcam;


    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rearDrive = null;
    private DcMotor slideMotor = null;
    private Servo clawServo;
    //private CRServo slideServoA;
    //private CRServo slideServoB;
    //private CRServo slideServoC;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double ROTATION_SPEED = 0.6;
    // Servo stuff
    static final double INCREMENT_CLAW  =         0.06;     // amount to slew claw servo each cycle
    static final double INCREMENT_SLIDE =         0.10;     // amount to slew slide servo
    static final double MAX_POS         =          1.0;     // Maximum rotational position (tested: 1.0 = 270 degrees)
    static final double MIN_POS         =          0.0;     // Minimum rotational position
    static final double MAX_CLAW        =   45.0/270.0;     // Maximum rotational position for the claw

    // Define class members

    double clawPos = (MIN_POS);  // Start at 0
    double slidePower = (MIN_POS); // Start at 0
    boolean clawActivated = false;
    boolean buttonAPressed = false;
    //Servo stuff end

    // Method that simplifies instruction for movement, math required to determine power is done here
    // Is a copy of math done from a template, and is in use in our main program

    // movementY is forward-back movement (negative backwards positive forwards),
    // movementX is left-right movement (negative left positive right).
    public void setMotorInstruction(double movementY, double movementX, double rotation) //based off of https://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot
    {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        //double axial = movementY;
        //double lateral =  movementX;
        //double yaw =  rotation;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.

        double FrontLeft  = movementY + movementX * (Math.sqrt(2)/2) + rotation;
        double FrontRight = movementY - movementX * (Math.sqrt(2)/2) - rotation;
        //double FrontLeft  = -1/2*movementX - Math.sqrt(3)/2*movementY + rotation;
        //double FrontRight = -1/2*movementX + Math.sqrt(3)/2*movementY + rotation;
        double Rear   = movementX - rotation;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(FrontLeft), Math.abs(FrontRight));
        max = Math.max(max, Math.abs(Rear));


        if (max > 1.0) {
            FrontLeft  /= max;
            FrontRight /= max;
            Rear   /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(FrontLeft);
        rightFrontDrive.setPower(FrontRight);
        rearDrive.setPower(Rear);

    }


    // Function to control slides. Pass -1 to lower, 0 to halt, 1 to raise
    public void setSlidePos(double power) {
        //.setPower(power);
        //slideServoB.setPower(power);
        //slideServoC.setPower(power);
        slideMotor.setPower(power);
    }


    public void moveClaw(boolean clawActivated) {
        /* {
            if (clawActivated) {
                // Keep stepping up until we hit the max value.
                clawPos += INCREMENT_CLAW;
                if (clawPos > MAX_CLAW ) {
                    clawPos = MAX_CLAW;
                }
            } else {
                // Keep stepping down until we hit the min value.
                clawPos -= INCREMENT_CLAW;
                if (clawPos < MIN_POS ) {
                    clawPos = MIN_POS;
                }

            }
        

        } while (clawPos != MAX_CLAW || clawPos != MIN_POS);*/

        if (clawActivated) {
            // Keep stepping up until we hit the max value.
            clawPos += INCREMENT_CLAW;
            if (clawPos > MAX_CLAW ) {
                clawPos = MAX_CLAW;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            clawPos -= INCREMENT_CLAW;
            if (clawPos < MIN_POS ) {
                clawPos = MIN_POS;
            }

        }
        clawServo.setPosition(clawPos);
        sleep(70);
    }

    // Color sensor side of things setup
    ColorSensor colorSensor;    // Hardware Device Object

    int scenario = 0; //0 is parking location 1, 1 is parking location 2, 2 is parking location three

    // Non color sensor stuff
    boolean cameraError = false;
    double timeXpos;
    double timeXneg;

    double timeY;
    float externalJunctionPointX;
    float externalJunctionPointY;
    float contourarea;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new DetectPoles());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */

                // Note: the YUV thing is a lie. That is the camera input format, but from the software side, 
                //it inputs RGBA
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                cameraError = true;
            }
        });


        //COLORSENSOR SETUP

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rearDrive  = hardwareMap.get(DcMotor.class, "rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        //slideServoA = hardwareMap.get(CRServo.class, "slide_servo_a");
        //slideServoB = hardwareMap.get(CRServo.class, "slide_servo_b");
        //slideServoC = hardwareMap.get(CRServo.class, "slide_servo_c");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        //slideServoA.setDirection(DcMotorSimple.Direction.FORWARD);
        //slideServoB.setDirection(DcMotorSimple.Direction.FORWARD);
        //slideServoC.setDirection(DcMotorSimple.Direction.FORWARD);



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
            moveClaw(true);
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

        // Drive right
        setMotorInstruction(FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {

        }

        float signX = 0;

        if (!cameraError)
        {
            // Rotate to let camera see
            telemetry.addData("Step", "Beginning rotation");
            telemetry.update();
            setMotorInstruction(0, 0, -ROTATION_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.15/2)) {

            }
            /*// Back up
            setMotorInstruction(-FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }*/

            double oldtimexpos = 0;
            double oldtimexneg = 0;
            double oldtimey = 0;
            double signXlastframe = 0;

            telemetry.addData("Step", "Rotation end");
            telemetry.update();
            if (true == true)//!DetectPoles.detectError()) commented out, was giving me an error.
            {
                telemetry.addData("Step", "Driving forward");
                telemetry.update();

                while(opModeIsActive() && (contourarea < 15000))
                {
                    if(!(externalJunctionPointX >= 170 && externalJunctionPointX <= 180))
                    {
                        // Clever little bit of code to avoid ifs. Gets sign of where we want the point - the actual point, then moves in the appropreate direction
                        signX = Math.signum(175 - externalJunctionPointX); //formerly 320
                        signXlastframe = signX;
                        setMotorInstruction(0, FORWARD_SPEED * signX * 0.25, 0);
                        runtime.reset();
                        while (opModeIsActive() && !(externalJunctionPointX >= 170 && externalJunctionPointX <= 180) && signXlastframe == signX) // loop until detectPoles.getTargetPointX is between 310 and 330
                        {
                            signX = Math.signum(175 - externalJunctionPointX); //formerly 320
                            telemetry.addData("Funni number 1", externalJunctionPointX);
                            telemetry.update();
                        }

                        if(signX > 0)
                        {
                            timeXpos = runtime.seconds() + oldtimexpos;
                            runtime.reset();

                            oldtimexpos = timeXpos;
                        }
                        else
                        {
                            timeXneg = runtime.seconds() + oldtimexneg;
                            runtime.reset();

                            oldtimexneg = timeXneg;
                        }

                    }
                    else
                    {
                        setMotorInstruction(FORWARD_SPEED * 0.25, 0, 0);


                        while (opModeIsActive() && (contourarea < 15000))
                        {
                            telemetry.addData("info", "going to pole");
                            telemetry.update();
                        }

                        timeY = runtime.seconds() + oldtimey;
                        runtime.reset();

                        oldtimey = timeY;
                    }
                }


                //setMotorInstruction(0, 0, 0);


                setSlidePos(1.0);
                setMotorInstruction(0, 0, 0);



                while (opModeIsActive() && (contourarea > 15000/2))
                {
                    telemetry.addData("info", "raising slide while stopped");
                    telemetry.update();
                }

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 4))
                {
                    telemetry.addData("info", "raising slide while stopped");
                    telemetry.update();
                }

                setSlidePos(0);

                setMotorInstruction(FORWARD_SPEED, 0, 0);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.2))
                {
                    telemetry.addData("info", "moving forward");
                    telemetry.update();
                }

                setMotorInstruction(0, 0, 0);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.5))
                {
                    telemetry.addData("info", "dropping cone");
                    telemetry.update();
                    moveClaw(false);

                }



                setMotorInstruction(-FORWARD_SPEED, 0, 0);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.2))
                {
                    telemetry.addData("info", "moving backward");
                    telemetry.update();
                }

                setSlidePos(-1);
                setMotorInstruction(0, 0, 0);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 7/2))
                {
                    telemetry.addData("info", "lowering slide while stopped");
                    telemetry.update();
                }

                setSlidePos(0.0);


            }
        }

        setMotorInstruction(-FORWARD_SPEED * 0.25, 0, 0); //reverse forward driving
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeY)) {

        }

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }

        setMotorInstruction(0, -FORWARD_SPEED * 0.25, 0); //reverse X axis driving pos
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeXpos)) {

        }

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }

        setMotorInstruction(0, FORWARD_SPEED * 0.25, 0); //reverse X axis driving neg
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeXneg)) {

        }

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }

        /*// Undo backup
        setMotorInstruction(FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Rotate to original pos
        telemetry.addData("Step", "Beginning rotation");
        telemetry.update();
        setMotorInstruction(0, 0, ROTATION_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.15/2)) {

        }
        telemetry.addData("Step", "Rotation end");
        telemetry.update();

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }

        // drive left
        setMotorInstruction(-FORWARD_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {

        }

        // Stop to minimize impact of inertia
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {

        }


        // Park RED
        if(scenario == 0)
        {
            //drive left
            setMotorInstruction(-FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        }

        // Park GREEN
        if(scenario == 1) //green is location 2
        {
            // Do nothing lmao
        }

        // Park BLUE
        if(scenario == 2) //blue is location 3
        {
            // Drive right
            setMotorInstruction(FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
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
            contourarea = (float)Imgproc.contourArea(biggestContour);

            //telemetry.addData("area", contourarea);
            //telemetry.update();

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