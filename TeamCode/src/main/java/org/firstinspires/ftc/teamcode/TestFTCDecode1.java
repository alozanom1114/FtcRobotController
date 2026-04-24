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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class TestFTCDecode1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    GoBildaPinpointDriver odo;
    double oldTime = 0;

    private boolean yButtonPreviouslyPressed;
    private boolean flywheelOn;
    private DcMotorEx flyWheel;
    private double BANG_BANG_TARGET_VELOCITY;

    private double angleValue = 0.85; //0.85-0.01 (bottom-top)
    private double additionalAngle = 0.00; //player 1/2 controlled angle

    private Servo angle;

    private double turretTargetTicks;
    private double turretTargetDeg;

    private double basketX;
    private double basketY;
    private double resetX;
    private double resetY;
    private DcMotorEx flyWheelDirection;
    private double smoothedTurretDeg;
    private Servo blocker;

    private DcMotor intakeMotor;

    public static final double OFFSET_X = 5.5;
    public static final double OFFSET_Y = -137;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftfront_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightback_drive");

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
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(5.5, -137, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Odo status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());

        //flywheel declarations
        flyWheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //angle servo declarations
        angle = hardwareMap.get(Servo.class, "angle");

        //TO-DO: Make this take a simple button to switch sides before the match starts.
        if (true) { //red
            basketX=3300.00;
            resetX=3396.00;
        } else { //blue
            basketX=300.00;
            resetX=225.00;
        }
        basketY=3300.00;
        resetY=220.00;

        flyWheelDirection = hardwareMap.get(DcMotorEx.class, "flywheelDirection");
        flyWheelDirection.setDirection(DcMotor.Direction.FORWARD);
        flyWheelDirection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelDirection.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blocker = hardwareMap.get(Servo.class, "blocker");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);






        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset(); //This is from the LinarOpMode
        resetRuntime(); //Used for the odometry unit time-keeping

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            odo.update();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate


            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (gamepad1.left_bumper) {
                frontLeftPower*=0.4;
                frontRightPower*=0.4;
                backLeftPower*=0.4;
                backRightPower*=0.4;
                telemetry.addData("Slow driving mode: ", true);
            } else {
                telemetry.addData("Fast driving mode: ", true);
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            if (gamepad1.y && !yButtonPreviouslyPressed) {
                flywheelOn = !flywheelOn;
            }
            yButtonPreviouslyPressed = gamepad1.y;

            if (flywheelOn) {
                flyWheel.setVelocity(BANG_BANG_TARGET_VELOCITY);
            } else {
                flyWheel.setVelocity(0);
            }

            telemetry.addData("Nominal flywheel RPM: ", (BANG_BANG_TARGET_VELOCITY / 28) * 60);
            telemetry.addData("#1 Actual flywheel RPM: ", ((flyWheel.getVelocity()) / 28) * 60);
            telemetry.addData("#1 voltage: ", flyWheel.getPower());
            //TO-DO: Add a modifier onto the flywheel speed (increments of 0.002 I think)
            //telemetry.addData("modifier value: ", modifier);

            telemetry.addData("angleValue: ", angleValue);
            telemetry.addData("Additional angle added: ", additionalAngle);
            telemetry.addData("Final Servo Angle: ", angle.getPosition());

            double currentX = 3621-pos.getY(DistanceUnit.MM);
            double currentY = pos.getX(DistanceUnit.MM);
            double currentH = pos.getHeading(AngleUnit.DEGREES);

            if (gamepad1.back) {
                turretTargetTicks=0;
            } else {
                turretTargetDeg = getTurretTargetAngle(currentX, currentY, currentH, basketX, basketY); // from your aiming function
                turretTargetTicks = degreesToTicks(turretTargetDeg);
            }

            //turretTargetTicks=0;

            flyWheelDirection.setPower(-1);
            flyWheelDirection.setTargetPosition((int) turretTargetTicks);
            flyWheelDirection.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //TO-DO: Add a modifier to be changed by the player
            BANG_BANG_TARGET_VELOCITY = getTurretSpeed(currentX, currentY, basketX, basketY);

            angleValue = getServoAngle(currentX, currentY, basketX, basketY);

            telemetry.addData("robot angle: ", currentH);
            telemetry.addData("robot x: ", currentX);
            telemetry.addData("robot y: ", currentY);
            telemetry.addData("Turret correction angle: ", degreesToTicks(turretTargetDeg));
            telemetry.addData("servo angle: ", getServoAngle(currentX, currentY, basketX, basketY));
            telemetry.addData("shooter speed: ", getTurretSpeed(currentX, currentY, basketX, basketY)); //TO-DO: Add modifier
            //telemetry.addData("modifier: ", modifier);//TO-DO: Add modifier

            if (gamepad1.a) {
                // -------------------------
                // A PRESSED → SHOOT LOGIC
                // -------------------------

                // Open blocker
                blocker.setPosition(0.0);

                sleep(300);

                // Intake ALWAYS spins while A is pressed
                intakeMotor.setPower(1.0);

                sleep (100);

                /* TO-DO: Integrate the distance sensors into the shooting code
                if((bottomDistance <= 5 || frontDistance <= 5)) {
                    // More than one ball → little helper stays down
                    littleServo.setPosition(0.0);
                } else if (topDistance <= 5) {
                    // Only one ball → use little helper
                    littleServo.setPosition(0.5);
                    sleep(100);
                } else {
                    // No balls detected → use little helper
                    littleServo.setPosition(0.5);
                    sleep(100);
                }

                enteredElse = false;
                waitingForBlocker = false;

                 */

            } else {
                // -------------------------
                // A NOT PRESSED → NORMAL LOGIC
                // -------------------------

                /*
                // Detect FIRST FRAME entering this else
                if (!enteredElse) {
                    littleServo.setPosition(0.50);
                    servoStartTime = System.currentTimeMillis();
                    waitingForBlocker = true;
                    enteredElse = true;
                }

                // Non-blocking delay
                if (waitingForBlocker &&
                        System.currentTimeMillis() - servoStartTime >= 500) {

                    blocker.setPosition(0.25);
                    waitingForBlocker = false;
                }

                 */

                blocker.setPosition(0.25); //Servo line from the commented out things above

                // Intake control
                if (gamepad1.left_trigger != 0 && !gamepad1.right_bumper) {
                    intakeMotor.setPower(gamepad1.left_trigger);
                } else if (gamepad1.left_trigger != 0 && gamepad1.right_bumper) {
                    intakeMotor.setPower(-gamepad1.left_trigger*0.75);
                } else {
                    intakeMotor.setPower(0);
                }
            }

            /*
            telemetry.addData("bottomDistance: ", bottomDistance);
            telemetry.addData("sideDistance1: ", sideDistance1);
            telemetry.addData("front distance: ", frontDistance);
            telemetry.addData("top distance: ", topDistance);

             */

            if (gamepad1.dpad_right) {
                angleValue += 0.01;
                //additionalAngle+=0.01;
                sleep(5);
            } else if (gamepad1.dpad_left) {
                angleValue -= 0.01;
                //additionalAngle-=0.01;
                sleep(5);
            }

            if (gamepad1.b) {
                odo.setOffsets(OFFSET_X, OFFSET_Y, DistanceUnit.MM);
                odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
                odo.setHeading(0, AngleUnit.DEGREES);
                odo.setPosX(resetY, DistanceUnit.MM);
                odo.setPosY(resetX, DistanceUnit.MM);
            }

            if (gamepad1.x) {
                BANG_BANG_TARGET_VELOCITY=-BANG_BANG_TARGET_VELOCITY;
            } else {
                BANG_BANG_TARGET_VELOCITY=BANG_BANG_TARGET_VELOCITY;
            }

            //angle.setPosition(angleValue + additionalAngle);
            odo.update();

            telemetry.addData("DeviceStatus", odo.getDeviceStatus());
            telemetry.addData("Enc X raw", odo.getEncoderX());   // raw counts
            telemetry.addData("Enc Y raw", odo.getEncoderY());
            telemetry.addData("Pos X (mm)", String.format(Locale.US, "%.3f", odo.getPosition().getX(DistanceUnit.MM)));
            telemetry.addData("Pos Y (mm)", String.format(Locale.US, "%.3f", odo.getPosition().getY(DistanceUnit.MM)));
            telemetry.addData("Vel X (mm/s)", String.format(Locale.US, "%.3f", odo.getVelX(DistanceUnit.MM)));
            telemetry.addData("Vel Y (mm/s)", String.format(Locale.US, "%.3f", odo.getVelY(DistanceUnit.MM)));
            telemetry.addData("Heading (deg)", String.format(Locale.US, "%.3f", odo.getPosition().getHeading(AngleUnit.DEGREES)));

            if (gamepad1.right_trigger>=0.5) {
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            telemetry.update();
        }
    }

    public static double getTurretTargetAngle(double robotX, double robotY, double robotHeadingDeg,
                                              double basketX, double basketY) {

        //final double TURRET_MIN_DEG = -125.0;
        //final double TURRET_MAX_DEG = 225.0;

        double deltaX = basketX - robotX;
        double deltaY = basketY - robotY;

        // Angle to basket in field coords (0° = forward, CW positive)
        double targetAngleDeg = -Math.toDegrees(Math.atan2(deltaX, deltaY));

        // Convert to robot-relative
        double relativeAngle = targetAngleDeg - robotHeadingDeg;

        // Normalize to [-180, 180)
        relativeAngle = ((relativeAngle + 540) % 360) - 180;

        // CCW limit = +135°, CW limit = -225° (continuous)
        if (relativeAngle > 135.0) {
            // Move around the CW side instead of stopping at 135
            relativeAngle -= 360.0;  // e.g., 150 -> -210
        } else if (relativeAngle < -225.0) {
            // Wrap around the CCW side if below CW limit
            relativeAngle += 360.0;  // e.g., -230 -> 130
        }

        return relativeAngle;
    }

    public static double getTurretSpeed(double robotX, double robotY,
                                        double basketX, double basketY) {

        double dx_m = (basketX - robotX) / 1000.0;   // horizontal planar distance, m
        double dy_m = (basketY - robotY) / 1000.0;   // if you need 2D distance, see below
        double R = Math.sqrt(dx_m*dx_m + dy_m*dy_m);   // radial distance in meters
        double h_a = 1.190, h_s = 0.381, x_1 = R + 0.235, m_s = 0.8085, m_i = -1.0/m_s, a_g = -9.80665;

        double theta_i = Math.atan((h_a - h_s - (x_1 * m_i)/(2))/((x_1)/(2)));
        double u = Math.sqrt((a_g * x_1)/(Math.pow(Math.cos(theta_i), 2)*(m_i - Math.tan(theta_i))));

        double ppr = 28.0, mew = 1.67, d_s = 0.1, theta_r = 1.2, pi = 3.14159265; // mew = 0.318

        double v_t = (u * ppr)/(mew * d_s);
        //double theta_a = (1.316 - theta_r * theta_i)/(2 * pi);

        return -v_t;
    }

    public static double getServoAngle (double robotX, double robotY, double basketX, double basketY) {
        double dx_m = (basketX - robotX) / 1000.0;   // horizontal planar distance, m
        double dy_m = (basketY - robotY) / 1000.0;   // if you need 2D distance, see below
        double R = Math.sqrt(dx_m*dx_m + dy_m*dy_m);   // radial distance in meters
        double h_a = 1.190, h_s = 0.381, x_1 = R + 0.235, m_s = 0.8085, m_i = -1.0/m_s, a_g = -9.80665;

        double theta_i = Math.atan((h_a - h_s - (x_1 * m_i)/(2))/((x_1)/(2)));
        // convert chosen angle (radians) to physical degrees relative to horizon
        double chosenAngleDeg = Math.toDegrees(theta_i); // e.g. 20.5 deg

        double servoA = 0.85;    // servo position that yields angleA
        double angleA = 78.0;    // degrees (relative to x-axis) at servoA
        double servoB = 0.01;    // servo position that yields angleB
        double angleB = 53.0;    // degrees (relative to x-axis) at servoB

        // Inverse linear map
        double servoPos = 0.85
                + (((theta_i * (180 / Math.PI)) - 78)
                * (0.01 - 0.85)
                * (1.0 / (53 - 78)));

        double minCalib = Math.min(servoA, servoB);
        double maxCalib = Math.max(servoA, servoB);
        servoPos = Math.max(minCalib, Math.min(maxCalib, servoPos));

        // Also enforce the global servo range [0.0, 1.0]
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        // Optional trim
        double trim = 0.0;            // change to small value like 0.01 if needed
        servoPos = Math.max(0.0, Math.min(1.0, servoPos + trim));

        return servoPos;
    }

    public static final double TICKS_PER_REV = 384.5;
    public static final double GEAR_RATIO = 5.263; // motor:turret
    public static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    public static double degreesToTicks(double degrees) {
        // Convert to encoder ticks
        return degrees * TICKS_PER_DEGREE;
    }

    public static double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }
}
