package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptGamepadTouchpad;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;

// name this OpMode and determine a group
@TeleOp (name="CompetitionBot", group="Teleop")
public class CompetitionBot extends OpMode {

    /* Declare OpMode members. */

    HardwareMap_CompetitionBot robot       = new HardwareMap_CompetitionBot();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeLift = new ElapsedTime();

    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        //robot.slideSystem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello WAGS Driver!!");    //
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /* Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        double rStickX;
        double rStickY;
        double lStickX;
        double targetAngle;
        double mag1;
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;

        //gamepad 1 driving
        rStickX = gamepad1.right_stick_x;
        rStickY = gamepad1.right_stick_y; // this used to be negative
        lStickX = gamepad1.left_stick_x;

        targetAngle = (Math.atan2(rStickY,rStickX));

        rotationPower = -lStickX;
        mag1 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle + Math.PI / 4));
        mag2 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle - Math.PI / 4));

        maxPower = Math.max(Math.abs(mag1) +  Math.abs(rotationPower) , Math.abs(mag2) +  Math.abs(rotationPower)) + 0.15;
        scaleDown = 1.0;

        if (maxPower > 1)
            scaleDown = 1.0 / maxPower;

        robot.leftFront.setPower((mag2 + rotationPower) * scaleDown);
        robot.rightFront.setPower((mag1 - rotationPower) * scaleDown);
        robot.leftBack.setPower((mag1 + rotationPower) * scaleDown);
        robot.rightBack.setPower((mag2 - rotationPower) * scaleDown);

        //original --> - / + / - / +

        //end of gamepad driving 1
        //beginning of Natalie bad code
        double rStickX2;
        double rStickY2;
        double lStickX2;
        double targetAngle2;
        double mag12;
        double mag22;
        double rotationPower2;
        double maxPower2;
        double scaleDown2;

        //gamepad 1 driving
        rStickX2 = gamepad2.right_stick_x;
        rStickY2 = gamepad2.right_stick_y; // this used to be negative
        lStickX2 = gamepad2.left_stick_x;

        targetAngle2 = (Math.atan2(rStickY2,rStickX2));

        rotationPower2 = -lStickX2;
        mag12 = Math.sqrt(Math.pow(rStickX2,2) + Math.pow(rStickY2,2)) * (Math.sin(targetAngle2 + Math.PI / 4));
        mag22 = Math.sqrt(Math.pow(rStickX2,2) + Math.pow(rStickY2,2)) * (Math.sin(targetAngle2 - Math.PI / 4));

        maxPower2 = Math.max(Math.abs(mag12) +  Math.abs(rotationPower2) , Math.abs(mag22) +  Math.abs(rotationPower2)) + 0.15;
        scaleDown2 = 1.0;

        if (maxPower2 > 1)
            scaleDown2 = 1.0 / maxPower2;

        robot.arm1.setPower((mag22 + rotationPower2) * scaleDown2);
        //robot.rightFront.setPower((mag1 - rotationPower) * scaleDown);
        //robot.leftBack.setPower((mag1 + rotationPower) * scaleDown);
        //robot.rightBack.setPower((mag2 - rotationPower) * scaleDown);

        boolean isButtonB2 = gamepad2.b; //moving shooter wheels (they move simultaneously in opposite directions)
        boolean isButtonA2 = gamepad2.a; //to move arm
        boolean isButtonX2 = gamepad2.x; //down lift
        boolean isButtonY2 = gamepad2.y; //intake sucking in

        boolean isButtonLB2 = gamepad2.left_bumper;
        boolean isButtonRB2 = gamepad2.right_bumper;

        boolean isButtonDU2 = gamepad2.dpad_up;
        boolean isButtonDR2 = gamepad2.dpad_right;
        boolean isButtonDL2 = gamepad2.dpad_left;
        boolean isButtonDD2 = gamepad2.dpad_down;

        float isButtonLT2 = gamepad2.left_trigger;
        float isButtonRT2 = gamepad2.right_trigger;

        float isButtonLT1 = gamepad1.left_trigger;
        float isButtonRT1 = gamepad1.right_trigger;

        boolean isButtonDU1 = gamepad1.dpad_up;
        boolean isButtonDR1 = gamepad1.dpad_right;
        boolean isButtonDL1 = gamepad1.dpad_left;
        boolean isButtonDD1 = gamepad1.dpad_down;

        //programming buttons for gamepad 2 bumpers
        final int down = 0;
        final int level1 = -4530;

        //Arm --> lift up
        if (isButtonA2) {
            robot.arm1.setPower(-0.5); // we changed speed from 1 to 0.5 to see if it stops jerkiness
            robot.arm2.setPower(-0.5);
            telemetry.addData("Button","A2");
            //telemetry.addData("Arm 1", String.format("%7d", robot.arm1.getCurrentPosition()));
            //telemetry.addData("Arm 2", String.format("%7d", robot.arm2.getCurrentPosition()));
            telemetry.update();
        } else {
            robot.arm1.setPower(0);
            robot.arm2.setPower(0);
            telemetry.addData("Button", "None");
        }

        //Arm --> go down
        if (isButtonX2) {
            robot.arm1.setPower(0.5);
            robot.arm2.setPower(0.5);
            telemetry.addData("Button","X2");
            //telemetry.addData("Arm 1", String.format("%7d", robot.arm1.getCurrentPosition()));
            //telemetry.addData("Arm 2", String.format("%7d", robot.arm2.getCurrentPosition()));
            telemetry.update();
        } else {
            robot.arm1.setPower(0);
            robot.arm2.setPower(0);
            telemetry.addData("Button", "None");
        }

        /*
        //lift to high junction --> button Y = high junction
        if (isButtonY2) {
            robot.slideSystem.setPower(0);
            liftUp(3.80, 1);
            telemetry.addData("Button","Y2");
        } else {
            telemetry.addData("Button","None");
            robot.slideSystem.setPower(0);
        }
         */

        //Wheels for paper airplane shooter
        if (isButtonB2) {
            robot.shooter.setPower(1); // have to use setPower command if using continuous servos (motors now)
            //robot.leftWheel.setPower(1);
            telemetry.addData("Button","B2");
        } else {
            robot.shooter.setPower(0);
            //robot.leftWheel.setPower(0);
            telemetry.addData("Button", "None");
        }

        //Intake --> clamping
        if (isButtonY2) {
            robot.Intake1.setPosition(180);
            robot.Intake2.setPosition(0);
            telemetry.addData("Button","Y2");
        } else {
            robot.Intake1.setPosition(0);
            robot.Intake2.setPosition(180);
            telemetry.addData("Button", "None");
        }

        //Pivot --> up
        if (isButtonLT2 > 0) {
            robot.Pivot.setPower(0.5);
            robot.Pivot.setTargetPosition((int) gamepad2.left_trigger);
            //robot.Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Button","LT2");
            telemetry.update();
        } else {
            robot.Pivot.setPower(0);
        }

        //Pivot --> down
        if (isButtonRT2 > 0) {
            robot.Pivot.setPower(-0.5);
            robot.Pivot.setTargetPosition((int) gamepad2.right_trigger);
            //robot.Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Button","RT2");
            telemetry.update();
        } else {
            robot.Pivot.setPower(0);
        }

        //raise the arm to top position
        //if () {
            //robot.arm1.setPower(-1);
            //robot.arm2.setPower(1);
            //robot.arm1.setPower(1);
            //robot.arm2.setPower(-1);
            //robot.arm1.setTargetPosition(2000);
            //robot.arm2.setTargetPosition(-2000);
            //liftUpPosition(-2000, 1);
            //telemetry.addData("Button","A2");
        //} //else {
            //robot.arm1.setPower(0);
            //robot.arm2.setPower(0);

        //Pivot --> Up
        //if (isButtonDU2) {
            //robot.Pivot.setPower(0.5);
            //telemetry.addData("Button","DU2");
        //} else {
            //robot.Pivot.setPower(0);
            //telemetry.addData("Button", "None");
        //}

        //Pivot --> Down
        //if (isButtonDD2) {
            //robot.Pivot.setPower(-0.5);
            //telemetry.addData("Button","DD2");
        //} else {
            //robot.Pivot.setPower(0);
            //telemetry.addData("Button", "None");
        //}

    }

    //liftUp
    public void liftUp (double liftTime, double liftSpeed) {
        runtimeLift.reset();
        while (runtimeLift.seconds() < liftTime) {
            robot.arm1.setPower(liftSpeed);
            robot.arm2.setPower(liftSpeed);
        }
        robot.arm1.setPower(0);
        robot.arm2.setPower(0);
    }

    public void liftUpPosition(int position, double liftSpeed) {

        robot.arm1.setTargetPosition(Math.abs(position));
        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm1.setPower(Math.abs(liftSpeed));
        robot.arm2.setTargetPosition(position);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setPower(Math.abs(liftSpeed));

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
        telemetry.addData("Say", "Good Job Team! We have STOPPED!!");
    }
}