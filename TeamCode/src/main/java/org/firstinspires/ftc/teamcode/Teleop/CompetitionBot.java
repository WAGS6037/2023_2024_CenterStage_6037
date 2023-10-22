package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        boolean isButtonB2 = gamepad2.b; //moving shooter wheels (they move simultaneously in opposite directions)
        boolean isButtonA2 = gamepad2.a; //to move arm
        boolean isButtonX2 = gamepad2.x; //dowwn lift
        boolean isButtonY2 = gamepad2.y;

        boolean isButtonLB2 = gamepad2.left_bumper;
        boolean isButtonRB2 = gamepad2.right_bumper;

        boolean isButtonDU2 = gamepad2.dpad_up;
        boolean isButtonDR2 = gamepad2.dpad_right;
        boolean isButtonDL2 = gamepad2.dpad_left;
        boolean isButtonDD2 = gamepad2.dpad_down;

        boolean isButtonDU1 = gamepad1.dpad_up;
        boolean isButtonDR1 = gamepad1.dpad_right;
        boolean isButtonDL1 = gamepad1.dpad_left;
        boolean isButtonDD1 = gamepad1.dpad_down;

        //programming buttons for gamepad 2 bumpers
        final int down = 0;
        final int level1 = -4530;

        /*
        if (isButtonRB2) {
            //robot.slideSystem.setPower(1);
            //liftUpPosition(level1, 1.0);
            telemetry.addData("Button","RB");
        } else if (isButtonLB2) {
            //liftUpPosition(down, 1.0);
            telemetry.addData("Button","LB");
        }else {
            telemetry.addData("Button","None");
            robot.slideSystem.setPower(0);
        }
        //programming buttons for gamepad 2
        //X, Y, A, B
         */

        //Arm --> lift up
        if (isButtonRB2) {
            robot.arm1.setPower(1);
            robot.arm2.setPower(-1);
            telemetry.addData("Button","RB2");
        }else {
            robot.arm1.setPower(0);
            robot.arm2.setPower(0);
            telemetry.addData("Button", "None");
        }

        //Arm --> go down
        if (isButtonLB2) {
            robot.arm1.setPower(-1);
            robot.arm2.setPower(1);
            telemetry.addData("Button","LB2");
        }else {
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
        }else {
            robot.shooter.setPower(0);
            //robot.leftWheel.setPower(0);
            telemetry.addData("Button", "None");
        }
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

        robot.arm1.setTargetPosition(position);
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