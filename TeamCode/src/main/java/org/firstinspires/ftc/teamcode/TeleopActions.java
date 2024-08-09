package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController;
import org.firstinspires.ftc.teamcode.helpers.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;
import org.firstinspires.ftc.teamcode.vision.pipelines.WhitePixelProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopActions extends ActionOpMode {


    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.6, 0.0, 1);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    double speed;
    Rotation2d targetHeading = PoseStorage.currentPose.heading;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public SparkFunOTOSDrive drive;

    List<Action> runningActions = new ArrayList<>();
    final ElapsedTime loopTime = new ElapsedTime();
    boolean pixelInClaw = false;
    boolean pixelInHook = true;//false;
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();
    boolean showMotorTelemetry = true;
    boolean showStateTelemetry = true;
    boolean showLoopTimes = true;
    boolean showPoseTelemetry = true;
    boolean showCameraTelemetry = false;

    MotorControl motorControl;
    MotorActions motorActions;
    boolean drivingEnabled = true;
    boolean actionRunning = false;
    boolean suspendSet = false;
    LinkedList<Double> loopTimeAvg = new LinkedList<>();


    @Override
    public void runOpMode() {

        //  Initialization Period

        // Enable Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive = new SparkFunOTOSDrive(hardwareMap, PoseStorage.currentPose);


        joystickHeadingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /*
        // Motor Init
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        motorControl.slide.findZero();
        motorControl.activatePreset(MotorControl.combinedPreset.IDLE);

         */


        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            // Reset measured loop time
            loopTime.reset();
            // Reset bulk cache
            allHubs.forEach(LynxModule::clearBulkCache);

            // This lets us do reliable rising edge detection, even if it changes mid loop
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            boolean padSlowMode = gamepad1.left_bumper;
            boolean padFastMode = gamepad1.right_bumper;
            boolean padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            // Misc/Obscure
            boolean padCameraAutoAim = gamepad1.right_stick_button;

            // Extra Settings
            boolean pad1ExtraSettings = gamepad1.share;
            boolean pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left; // 1 rumble blue, 2 rumble red
            boolean pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up;


            // Gamepad 2
            // Presets/Automated
            boolean padHalfCycle = gamepad2.left_trigger > 0.25;
            boolean padFullCycle = gamepad2.right_trigger > 0.25 || gamepad1.circle;

            boolean padHighPreset = gamepad2.y;
            boolean padMidPreset = gamepad2.b;
            boolean padLowPreset = gamepad2.a;

            boolean padClawToggle = (gamepad2.right_bumper && !previousGamepad2.right_bumper); //|| (gamepad1.square && !previousGamepad1.square);

            boolean padShooter = gamepad2.square; //|| gamepad1.square;

            // Manual Control
            double padSlideControl = -gamepad2.left_stick_y;
            double padSlideControlMultiplier = 40;
            /*
            double padArmControl = -gamepad2.right_stick_y;
            double padArmControlMultiplier = 2;

             */

            double padSuspendControl = -gamepad2.right_stick_y;
            double padSuspendControlMultiplier = 1;


            // Misc
            double padGunnerDrive = gamepad2.right_stick_x; // only when right trigger held
            boolean padForceDown = gamepad2.dpad_down && gamepad2.options;
            boolean padMissedHook = gamepad2.dpad_up;
            boolean padAutoPlacer = gamepad2.dpad_down && gamepad2.share;

            boolean padSuspendMode = gamepad2.triangle && !previousGamepad2.triangle;


            // Update the speed
            if (padSlowMode) {
                speed = .35;
            } else if (padFastMode) {
                speed = 1.5;
            } else {
                speed = 0.3;//.8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BLUE)) { // Team is declared and saved there for auto
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                targetHeading = drive.pose.heading;
                gamepad1.rumbleBlips(1); // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2);
                    } else {
                        gamepad1.rumbleBlips(1);
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1);
                        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                    } else {
                        gamepad1.rumbleBlips(2);
                        PoseStorage.currentTeam = PoseStorage.Team.RED;
                    }
                }
            }

            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );

            //Pose2d poseEstimate = drive.pose;
            double rotationAmount = -drive.pose.heading.log(); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric && !padCameraAutoAim) {
                if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) { // Depending on which side we're on the color changes
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(90)).times(new Vector2d(-input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    //input = drive.pose.heading.inverse().plus(Math.toRadians(-90)).times(new Vector2d(input.x, -input.y)); // magic courtesy of
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (drivingEnabled) {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            input.y
                                    ),
                                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                            )
                    );
                    targetHeading = drive.pose.heading;
                } else {
                    // Set the target heading for the heading controller to our desired angle
                    if (Math.sqrt(Math.pow(controllerHeading.x, 2.0) + Math.pow(controllerHeading.y, 2.0)) > 0.4) {
                        // Cast the angle based on the angleCast of the joystick as a heading
                        if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(-90));
                        } else {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(90));
                        }
                    }

                    joystickHeadingController.targetPosition = targetHeading.toDouble();


                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (joystickHeadingController.update(drive.pose.heading.log())
                            * MecanumDrive.PARAMS.kV
                            * MecanumDrive.PARAMS.trackWidthTicks);
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            input.y
                                    ),
                                    headingInput
                            )
                    );

                }
            }


            // LIFT CONTROL/FSM

            /*

            // Slide (Manual)
            if (motorControl.slide.getTargetPosition() > 1100 && padSlideControl > 0) {
                motorControl.slide.setTargetPosition(1100);

            } else if (motorControl.slide.getTargetPosition() <= 40 && padSlideControl < 0 && !padForceDown) {
                motorControl.slide.setTargetPosition(40);

            } else {
                motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() + (padSlideControl * padSlideControlMultiplier));
            }


            motorControl.suspend.setPower(padSuspendControl * padSuspendControlMultiplier);


            if (pixelInClaw && padHalfCycle && motorControl.slide.motor.getCurrentPosition() < 850 && !actionRunning) {
                run(pixelToHook());
            }
            if (pixelInHook && padFullCycle && !actionRunning) {
                run(placePixel(this::padRelease)); // TODO: maybe causes issues?
            }


            if (pixelInClaw && runningActions.isEmpty()) {
                motorControl.claw.setPosition(0.82); //0.85
            } else {
                motorControl.claw.setPosition(0.94);
            }



            if (padClawToggle) {
                pixelInClaw = !pixelInClaw;
                if (pixelInClaw) {
                    motorControl.claw.setPosition(0.82); //0.85
                } else {
                    motorControl.claw.setPosition(0.94);
                }
            }
            if (padMissedHook) {
                pixelInHook = false;
                motorControl.seperator.setPosition(0);
            }




            // Reset

            if (padForceDown) {
                motorControl.hookArm.setPosition(1);
            }

            if (padAutoPlacer) {
                motorControl.autoPlacer.setPosition(0.5);
            } else {
                motorControl.autoPlacer.setPosition(1);
            }

            // Shooter

            if (padShooter) {
                motorControl.shooter.setPower(-1);
            } else {
                motorControl.shooter.setPower(0);
            }

            /*
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                runningActions.add(motorActions.seperator.hold());
            } else if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down){
                runningActions.add(motorActions.seperator.release());
            }

             */
            /*
            if (gamepad2.dpad_down) {
                motorControl.seperator.setPosition(0.35);
            } else {
                motorControl.seperator.setPosition(0);
            }



             if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                    motorControl.hookArm.setPosition(0.6);
                    motorControl.slide.setTargetPosition(1200);
             }

             */

            double colorAlpha = 0;
            double pad2rumble;

            // rumble the gunner controller based on the claw color sensor
            if (colorAlpha > 200 && !pixelInClaw) {
                pad2rumble = Math.log10(colorAlpha) / 6;
            } else {
                pad2rumble = 0;
            }
            gamepad2.rumble(pad2rumble, pad2rumble, Gamepad.RUMBLE_DURATION_CONTINUOUS);

            // update RR, update motor controllers


            // TELEMETRY

            TelemetryPacket packet = new TelemetryPacket();
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

            updateAsync(packet);
            drive.updatePoseEstimate();
            //motorControl.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            double loopTimeMs = loopTime.milliseconds();
            loopTimeAvg.add(loopTimeMs);
            while (loopTimeAvg.size() > 1000) {
                loopTimeAvg.removeFirst();
            }

            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading.log());
                telemetry.addData("oldHeading",drive.lastOtosPose.heading.log());
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---");
                telemetry.addData("loopTimeMs", loopTimeMs);
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
                telemetry.addData("OTOSReadAverage ", loopTimeAvg.stream().reduce(0.0,Double::sum) / loopTimeAvg.size());
            }
            /*
            if (showMotorTelemetry) {
                telemetry.addLine("--- Motors ---");
                telemetry.addData("armTarget", motorControl.clawArm.getTargetPosition());
                telemetry.addData("armPosition", motorControl.clawArm.motor.getCurrentPosition());
                telemetry.addData("slideTarget", motorControl.slide.getTargetPosition());
                telemetry.addData("slidePosition", motorControl.slide.motor.getCurrentPosition());
                telemetry.addData("clawPower", motorControl.claw.getPosition());
                telemetry.addData("hookPower", motorControl.hookArm.getPosition());
                telemetry.addData("colorAlpha", colorAlpha);
                telemetry.addData("colorAlphaLog", Math.log10(colorAlpha));
                telemetry.addData("touchSensor", motorControl.touch.isPressed());
                telemetry.addData("touchSensor", motorControl.magnet.isPressed());
            }


            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---");
                telemetry.addData("pixelInClaw", pixelInClaw);
                telemetry.addData("pixelInHook", pixelInHook);
                telemetry.addData("armState", motorControl.clawArm.currentPreset);
                telemetry.addData("elapsedTime", liftTimer.milliseconds());
                telemetry.addData("driveAction", driveActionRunning());
                telemetry.addData("actions",runningActions);
                telemetry.addData("suspendMode",suspendSet);
                telemetry.addData("actionRunning", actionRunning);
            }

             */
            telemetry.update();
        }
    }

        Action pixelToHook () {
            return new ParallelAction(new SequentialAction(
                    new InstantAction(() -> actionRunning = true),
                    motorActions.pixelToHook(),
                    new InstantAction(() -> {
                        pixelInClaw = false;
                        pixelInHook = true;
                    }),
                    new InstantAction(() -> actionRunning = false)
            ),
                    new SequentialAction(
                            moveBack3In()
                    ));
        }
        Action placePixel (input input){
            return new SequentialAction(
                    new InstantAction(() -> actionRunning = true),
                    motorActions.hookToBackdrop(),
                    (telemetryPacket -> padRelease()),
                    motorActions.placeSecondPixel(),
                    (telemetryPacket -> padRelease()),
                    motorActions.returnHook(),
                    new InstantAction(() -> pixelInHook = false),
                    new InstantAction(() -> actionRunning = false)

            );

        }
        // TODO: probably not needed, just make a normal action
        interface input {
            boolean isPressed();
        }
        Action waitForInput (input input){
            return telemetryPacket -> input.isPressed();
        }

        boolean padRelease () {
            return !((gamepad2.right_trigger > 0.25) && previousGamepad2.right_trigger < 0.25);
        }

        Action moveBack3In() {
            return new SequentialAction(
                    new InstantAction(() -> drivingEnabled = false),
                    new MoveBackAction(),
                    new SleepAction(0.1),
                    new StopMovingAction(),
                    new InstantAction(() -> drivingEnabled = true)
            );
        }

        class MoveBackAction implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!gamepad2.left_bumper) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            -1,
                                            0
                                    ),
                                    0
                            )
                    );
                }
                    return false;

            }
        }
        class StopMovingAction implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    )
            );
                return false;
         }
        }

        boolean driveActionRunning() {
            return containsDriveAction(runningActions);
        }
        boolean containsDriveAction(List<Action> actions) {
            for (Action action : actions) {
                if (action.getClass() == MecanumDrive.FollowTrajectoryAction.class || action.getClass() == MoveBackAction.class || action.getClass() == StopMovingAction.class) {
                    return true;
                }
                if (action.getClass() == SequentialAction.class) {
                    SequentialAction sequentialAction = (SequentialAction) action;
                    return containsDriveAction(sequentialAction.getInitialActions());
                    }
                }
            return false;
        }

}

