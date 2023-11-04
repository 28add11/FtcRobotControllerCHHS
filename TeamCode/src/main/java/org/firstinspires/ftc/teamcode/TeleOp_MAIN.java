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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main driving opmode for the 2023-24 season.
 * This year our robot is non-Holonomic, meaning it can only move forward or back.
 * Feel free to add info about the whole functionality here later
 */

@TeleOp(name="MAIN", group="Linear Opmode")
//@Disabled
public class TeleOp_MAIN extends LinearOpMode {

    // Declare OpMode members for each of the 2 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;

    private DcMotor speen = null;

    // Servo stuff
    private Servo planeLauncher = null;
    static final double MIN_LAUNCHER = 0;
    static final double MAX_LAUNCHER       =   90.0/270.0;     // Maximum rotational position for the launcher servo
    double incrumentLauncher = 0.1;

    double launcherPOS = (MIN_LAUNCHER);

    static final int    CYCLE_MS        =           50;     // period of each cycle
    // Define class members

    //Servo stuff end

    //movementY is forward-back movement (negative backwards positive forwards), movementX is left-right movement (negative left positive right).
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

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor  = hardwareMap.get(DcMotor.class, "rightMotor");
        speen = hardwareMap.get(DcMotor.class, "spinner");
        planeLauncher = hardwareMap.get(Servo.class, "launcher");

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
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        speen.setDirection((DcMotorSimple.Direction.REVERSE));

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

            double drive    =  gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double turn     =  gamepad1.right_stick_x;

            // SERVO STUFF

            if (gamepad1.b) {
                // Keep stepping up until we hit the max value.
                launcherPOS += incrumentLauncher;
                if (launcherPOS > MAX_LAUNCHER ) {
                    launcherPOS = MAX_LAUNCHER;
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                launcherPOS -= incrumentLauncher;
                if (launcherPOS < MIN_LAUNCHER ) {
                    launcherPOS = MIN_LAUNCHER;
                }
            }


            // Motor Control
            setMotorInstruction(drive, -turn);

            if (gamepad1.a) {
                speen.setPower(1);
            } else {
                speen.setPower(0);
            }

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
