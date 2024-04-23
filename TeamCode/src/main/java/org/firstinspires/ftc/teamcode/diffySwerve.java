package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MAIN", group="Linear Opmode")
public class diffySwerve extends LinearOpMode{

/* Copyright (c) 2021 FIRST. All rights reserved.
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

/**
 * Main driving opmode for the 2023-24 season.
 * This year our robot is non-Holonomic, meaning it can only move forward or back.
 * Feel free to add info about the whole functionality here later
 */

    // Declare OpMode members for each of the 2 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveMotor = null;
    private DcMotor turnMotor = null;

    private int currentTicks;

    static final int    CYCLE_MS        =           50;     // period of each cycle

    // Define class members

    //Servo stuff end

    public void diffSwerve(double turn, double drive){

        double drivePower;

        // POV Mode uses left joystick to go forward & back, and right joystick to rotate.
        drivePower    = Range.clip(drive, -1.0, 1.0);
        //32 to 64 gear ratio

        // Calculate where to move the motor to
        int desiredPos = ;

        // Send calculated power to wheels
        driveMotor.setPower(drivePower);

        telemetry.addData("Drive", "%4.2f", drivePower);
        telemetry.addData("Turn", "%4.2f", turn);

    }

    //movementY is forward-back movement (negative backwards positive forwards), movementX is left-right movement (negative left positive right).

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        driveMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        turnMotor  = hardwareMap.get(DcMotor.class, "rightMotor");

        // Reset the motor encoder so that it reads zero ticks
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        currentTicks = turnMotor.getCurrentPosition();


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        turnMotor.setDirection(DcMotor.Direction.FORWARD);
        driveMotor.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y    =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x     =  gamepad1.right_stick_x;

            double turn = Math.toDegrees(Math.atan2(x, y));

            double drive = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            // Motor Control
            diffSwerve(drive, turn);

            sleep(CYCLE_MS);
            idle();

            // SERVO STUFF END

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
