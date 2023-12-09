/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Holonomic;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Easy", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetectionEasy extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /* Declare OpMode members. */
    HardwareMap_Holonomic robot = new HardwareMap_Holonomic(); // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    boolean pixelDetected;

    @Override
    public void runOpMode() {

        int state = 0;
        if (state == 0) {
            //init robot
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.leftFront.getCurrentPosition(),
                    robot.rightFront.getCurrentPosition());
            telemetry.update();
            waitForStart();
            state = 1;
        }
        //move to center spike line
        //if (state == 1) {
            //telemetry.addData("State", "1");
            //telemetry.update();
            //move forward 4 squares
            //encoderDrive(1, 8, 8, 8, 8, 30);
            //pixelDetected = tensorFlow();
            //if (pixelDetected) {
                //do whatever needed after detection
            //} else {
                //state = 2;
            //}
        //}
        //move to center spike line
        //if (state == 2) {
            //telemetry.addData("State", "2");
            //telemetry.update();
            //move forward 4 squares
            //encoderDrive(1, 8, 8, 8, 8, 30);
            //pixelDetected = tensorFlow();
            //if (pixelDetected) {
                //do whatever needed after
            //} else {
                //state = 3;
            //}
        //}
        //stopping robot
        //if (state == 3) {
            //telemetry.addData("State", "4");
            //telemetry.update();
            //stopMotors();
            //Move forward six feet.
            //pixelDetected = tensorFlow();
            //if (pixelDetected) {
                //do whatever needed
            //} else {
                //state = 4;
            //}
        //}
    }

    public boolean tensorFlow() {
        initTfod();
        int detected = 0;
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                detected = telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        return (detected > 0);
    }

    // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
        tfod.setZoom(2.0);
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        int pixelsDetected = 0;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if ((recognition.getConfidence() * 100) >= 75) {
               pixelsDetected++;
            }
        }   // end for() loop
        return pixelsDetected;
    }   // end method telemetryTfod()

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
    }

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void strafeLeft(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3/360.0;
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int leftBackTarget = robot.leftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightBackTarget = robot.rightBack.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);
        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                &&(robot.leftFront.getCurrentPosition() > leftFrontTarget && robot.rightFront.getCurrentPosition() < rightFrontTarget && robot.leftBack.getCurrentPosition() < leftBackTarget && robot.rightBack.getCurrentPosition() > rightBackTarget)) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            robot.leftFront.setPower(-(power + (error * k)));
            robot.rightFront.setPower((power + (error * k)));
            robot.leftBack.setPower((power - (error * k)));
            robot.rightBack.setPower(-(power - (error * k)));
            telemetry.addData("error: ",error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void strafeRight(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3 / 360.0;
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int rightFrontTarget = robot.rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int leftBackTarget = robot.leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int rightBackTarget = robot.rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);
        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                && (robot.leftFront.getCurrentPosition() < leftFrontTarget && robot.rightFront.getCurrentPosition() > rightFrontTarget && robot.leftBack.getCurrentPosition() > leftBackTarget && robot.rightBack.getCurrentPosition() < rightBackTarget)) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            robot.leftFront.setPower((power - (error * k)));
            robot.rightFront.setPower(-(power - (error * k)));
            robot.leftBack.setPower(-(power + (error * k)));
            robot.rightBack.setPower((power + (error * k)));
            telemetry.addData("error: ", error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ", robot.leftFront.getCurrentPosition());


            telemetry.update();
        }
        stopMotors();
    }

}   // end class
