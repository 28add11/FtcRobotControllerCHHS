/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 * The code also assumes an omni-wheel drivetrain
 *
 *   The desired path in this example is:
 *   - Drive forward to detect cone
 *   - Determine color
 *   - Push cone away (forward and back)
 *   - If RED, park left
 *   - if BLUE, park right
 *
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="ColorSensor Autodrive", group="Robot")
//@Disabled
public class colorSensorDrive extends LinearOpMode {


    ColorSensor colorSensor;    // Hardware Device Object
//    static float meterspersecond = 1;




    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rearDrive = null;
    private DcMotor slideMotor = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
//    static final double     TURN_SPEED    = 0.5;

    int scenario = 0; //0 is parking location 1, 1 is parking location 2, 2 is parking location three

    // Method that simplifies instruction for movement, math required to determine power is done here
    // Is a copy of math done from a template, and is in use in our main program

    // movementY is forward-back movement (negative backwards positive forwards),
    // movementX is left-right movement (negative left positive right).
   /* public void setMotorInstruction(double movementY, double movementX, double rotation) {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = movementY;
        double lateral =  movementX;
        double yaw =  rotation;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }*/

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

    @Override
    public void runOpMode() {

        //COLORSENSOR SETUP

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");




        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rearDrive  = hardwareMap.get(DcMotor.class, "rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


        // Drive forward
        setMotorInstruction(0, -FORWARD_SPEED, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.15)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Stop
        setMotorInstruction(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {

        }


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

            // In order to prevent the cones from getting stuck in the wheels, this code
            // pushes the cone forward and moves back to its original spot
            setMotorInstruction(0, -FORWARD_SPEED, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Run it back
            setMotorInstruction(0, FORWARD_SPEED, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }



            //drive left
            setMotorInstruction(-FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.86)) { //formerly 1.3
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
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Run it back
            setMotorInstruction(0, FORWARD_SPEED, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop to minimize impact of inertia
            setMotorInstruction(0, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.72)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }



            // Drive right
            setMotorInstruction(FORWARD_SPEED, 0, 0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.86)) { //formerly 1.3
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
