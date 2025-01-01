package org.firstinspires.ftc.teamcode.Teleop;

// - - - - - - - - - - Imports - - - - - - - - - - - - -

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;


//@Config
@TeleOp(group = "Teleop")
public class SimpleTeleop extends LinearOpMode {

    //- - - - - - - - - - - - - - Initialization of Variables - - - - - - - - - - - -
    // - - - Timer for the teleop period - - - //
    private ElapsedTime teleopTimer = new ElapsedTime();
    private static final float TELEOP_TIME_OUT = 140; // Match time limit
    // - - - Timer for the teleop period - - - //

    // - - - Constants + Variables - - - //
    private static final double RAISE_SPEED = 1.0; // Full speed for raising
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.6;
    private double RuntoPositionPower =0.4;
    private double DepositAngle=55;
    private int ArmHangInd=0;
    private int ArmLatchInd= 0;
    private int ArmDepositInd=0;
    private int ArmIntakeInd=0;
    private int SliderExtendInd=0;
    private int SliderRetractInd=0;
    private double SliderCurLen;
    private double ArmCurPosDeg;
    private int GripperRollInInd=0;
    private double GripperTeleOpClosePos;
    private PIDFCoefficients Default_Pid;
    private int TighterGripAdjustInd=0;
    private int TighterGripAdjustIndPrev=0;


    FtcDashboard dashboard;
    // - - - Constants + Variables - - - //
    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialize components - - - - - - - - - -

        // - - - Setting up Mecanum Drive - - - //
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);
        

        // - - - Set up dashboard telemetry - - - //
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());



        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        // Obtain the default calibrated gripper close position to assign it to the
        // Teleop Close Position since the gripper does get loose after several usage. 
        GripperTeleOpClosePos= gripper.getGripperClosePos();
        //Gripper open state
        //gripper.setGripperOpen();
        //Gripper holder to the side
        //gripper.setGripperHolderParallel();
        //gripper.setAnglerSide();
        
        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();
        // - - - Waiting for start signal from driver station - - - //

        // - - - - - - - - - - Initialize components - - - - - - - - - -


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // Control the robot's movement with the gamepad 1's sticks
            if(gamepad1.y){
                speedFactor = 0.7;
            }
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.right_stick_y * speedFactor, // Forward/Backward Movement
                    -gamepad1.left_stick_x * speedFactor, // Strafing Left/right
                    -gamepad1.right_stick_x * speedFactor // Rotation
            ));


            // - - - Arm Control - - - /
 
            // A button for latching in current position
            if (gamepad2.a) {
                // Set latching indication to 1 to maintain the arm at the current position
                ArmLatchInd = 1;
                ArmIntakeInd=0;
                ArmDepositInd=0;
                ArmHangInd=0;
                ArmCurPosDeg= armControl.getActArmPosDeg();
            }

            if(ArmLatchInd ==1){
                // Holding the Arm in position when ArmLatchInd is 1
                armControl.setDesArmPosDeg(ArmCurPosDeg);
            }

            // Left bumper to set Arm to the Deposit angle for depositing the sample
            if (gamepad2.left_bumper) {
                ArmDepositInd=1;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=0;
            }
            if(ArmDepositInd==1){
                armControl.setArmDeposit();
            }
            // Gamepad 1 a button: set arm power to 0.7 to hang the robot
            if (gamepad1.a) {
                ArmDepositInd=0;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=1;
                ArmCurPosDeg= armControl.getActArmPosDeg();
            }
            if(ArmHangInd==1){
                armControl.setArmPower(-0.6);

            }
            // Allow user to control the arm position once it is pushed more than 0.1 in magnitude
            if (Math.abs(gamepad2.right_stick_y) > 0.2 ) {
                    // Reset all the position indicators
                    ArmIntakeInd = 0;
                    ArmLatchInd=0;
                    ArmDepositInd=0;
                    ArmHangInd=0;
                    armControl.ArmRunModEncoder();
                    armControl.setArmPower(-1.0 * gamepad2.right_stick_y * 0.8);
                } else {
                if( ArmIntakeInd==0 && ArmLatchInd==0 && ArmDepositInd==0 && ArmHangInd==0) {
                    // zero power plus run mode reset
                    armControl.ArmRunModReset();
                }
            }

            // gamepad1.x: Reset Arm motor encode when the arm is fully down:
            // this is to compensate the build up error.
            if (gamepad1.x){
                armControl.ArmEncoderReset();
            }


            // - - - Slider motor control - - - //

            // Gamepad1 B button to: Reset slide encoder: slide must be fully retracted
            if (gamepad1.b) {
                    sliderControl.SliderEncoderReset();
            }

            // Controlling the slider motor using game pad2's left and right
            // triggers once magnitude > 0.1
            if (gamepad2.left_trigger > 0.2) {
                // Retracting the slider through driver control
                // Reset the run to position indicators

                SliderExtendInd=0;
                SliderRetractInd=0;
                sliderControl.SliderRunModEncoder();
                sliderControl.setSliderPower(-gamepad2.left_trigger * 0.6);

            } else if (gamepad2.right_trigger > 0.2) {

                // extending the slider through driver control
                // Reset the run to position indicators
                SliderExtendInd = 0;
                SliderRetractInd = 0;
                sliderControl.SliderRunModEncoder();
                SliderCurLen = sliderControl.getSliderLen();
                sliderControl.setSliderPower(gamepad2.right_trigger * 0.6);
            } else {
                if (SliderExtendInd==0 && SliderRetractInd==0) {
                    // zero power plus run mode reset
                    sliderControl.SliderRunModReset();
                }
            }
            // dpad_up to extend the slide 2 In
            if (gamepad2.dpad_up) {
                SliderExtendInd = 1;
                SliderRetractInd = 0;
                SliderCurLen=sliderControl.getSliderLen();
            }
            if(SliderExtendInd==1) {
                sliderControl.setSliderLenIncrement(SliderCurLen,2);
            }
            // dpad_down to retract the slide 2 In
            if (gamepad2.dpad_down) {
                SliderExtendInd = 0;
                SliderRetractInd = 1;
                SliderCurLen=sliderControl.getSliderLen();
            }
            if(SliderRetractInd==1) {
                sliderControl.setSliderLenIncrement(SliderCurLen,-2);
            }

            // - - - Gripper control - - - //
            // gamepad2 b button for open the gripper
            if (gamepad2.b) {
                gripper.setGripperOpen();
            }
            // gamepad2 x button for close the gripper
            if (gamepad2.x){
                gripper.setGripperPosition(GripperTeleOpClosePos);
            }
            // - - - Gripper Holder control - - - //
            // gamepad2 y button for setting gripper holder forward
            if (gamepad2.y) {
                gripper.setGripperHolderPerpendicular();
            }
            // gamepad2 right bumper to turn the Gripper Holder to the side
            if (gamepad2.right_bumper) {
                gripper.setGripperHolderParallel();
            }
            // gamepad1 Y to set the Gripper closer position to make the grip tighter,
            // which is to compensate the fact that the gripper is not robust enough
            // and it will lose its grip after several usage.
            if (gamepad1.y ) {
                // Increment TighterGripAdjustInd when Y is pushed. Actual behaviour
                // is that even a simple push will produce a signal being true for
                // several loops.
                TighterGripAdjustInd = TighterGripAdjustInd +1;
            } else {
                // Reset TighterGripAdjustInd when Y button is not pushed, this enables
                // proper function of next Y button push
                TighterGripAdjustInd=0;
            }
            // Only perform one tighter adjustment with one Y button push
            if (TighterGripAdjustInd==1){
                GripperTeleOpClosePos=GripperTeleOpClosePos-0.02;
                gripper.setGripperPosition(GripperTeleOpClosePos);
            }

            // angler control using gamepad2 dpad left and right (Hat)
            // dpad_left to for the Gripper system to face forward
            if (gamepad2.dpad_left) {
                gripper.setAnglerForward();
            }
            // dpad_right to set the Gripper system to the Side position
            if (gamepad2.dpad_right) {
                gripper.setAnglerSide();
            }

            // - - - Telemetry Updates - - - //
            // Sending important data to telemetry to monitor
            telemetry.addData("Arm Actual Position in Degree","%.3f", armControl.getActArmPosDeg());
            telemetry.addData("Arm Tgt Position in Ticks", armControl.getTgtArmTick());
            telemetry.addData("Arm Current Position in Ticks", armControl.getActArmTick());
            telemetry.addData("Arm Motor Power", "%.2f",armControl.getArmPower());
            telemetry.addData("Elapsed Time", "%.2f", teleopTimer.time());
            telemetry.addData("TwoStage Position", sliderControl.getSliderLen());
            telemetry.addData("Gripper Roll In Indicator", GripperRollInInd);
            telemetry.addData("Active Speed Factor: ", speedFactor);
            telemetry.addData("Gripper tght request: ", TighterGripAdjustInd);
            telemetry.addData("Current  Gripper Position: ", gripper.getGripperCurPos());
            telemetry.update();

        }
    }

}