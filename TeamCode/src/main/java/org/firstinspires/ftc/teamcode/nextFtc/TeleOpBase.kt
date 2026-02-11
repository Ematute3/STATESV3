package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Data.Lefile
import org.firstinspires.ftc.teamcode.Subsystem.Gate
import org.firstinspires.ftc.teamcode.Subsystem.Intake
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

import java.io.File
import kotlin.math.abs
import kotlin.math.hypot


data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
    // figure out what this stuff does
    private val resetModePhiAngle: Angle,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle,
    private val distanceToTime: (Double) -> Double
): NextFTCOpMode() {
    val x:  Double get() { return (PedroComponent.follower.pose.x);}
    val y:  Double get() { return (PedroComponent.follower.pose.y);}
    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad;}
    val vx: Double get() { return (PedroComponent.follower.velocity.xComponent);}
    val vy: Double get() { return (PedroComponent.follower.velocity.yComponent);}
    val vh: Angle  get() { return (PedroComponent.follower.velocity.theta.rad);}


    var driverControlled: PedroDriverControlled? = null;


    init {
        addComponents(
            SubsystemComponent(
        // add the subsystems
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {

        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { if (isBlue) it else -it },
            Gamepads.gamepad1.leftStickX.map { if (isBlue) it else -it },
            -Gamepads.gamepad1.rightStickX,
            false
        )


        val file = File(Lefile.filePath)
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

        PedroComponent.follower.pose = Pose(startX, startY, startH)
    }

    private var autoAimEnabled = true;
    private var resetMode = false;


    private var phiTrim = 0.0.deg;


    override fun onStartButtonPressed() {


    // ask will for colors
        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

        driverControlled!!()




        val intakeMotorDrive = Intake.DriverCommand(
            // change this for will
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger
        )
        intakeMotorDrive()

        Gamepads.gamepad1.rightBumper whenBecomesTrue{
            ParallelGroup(
                Gate.open,
                Intake.intake
            )()
//            ShooterSubsystem.isShooting = true  // todo do flywheel class
        } whenBecomesFalse {
            ParallelGroup(
                Gate.close,
                Intake.off
            )()
//            ShooterSubsystem.isShooting = false
        }



        /*
                Gamepads.gamepad2.circle whenTrue {
                    val dx = goalX - x
                    val dy = goalY - y
                    val dxy = hypot(dx, dy)
                    val dxp = dx + vx * distanceToTime(dxy)
                    val dyp = dy + vy * distanceToTime(dxy)
                    val dxyp = hypot(dxp, dyp)
                    val hp = h + vh * distanceToTime(dxyp)
                    TurretPhiSubsystem.AutoAim(
                       dxp, dyp, hp, phiTrim
                    )()
                }

         */

        // not trimming in reset mode
        // reset mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            resetMode = !resetMode;
            if (resetMode) {
                // 180.0.deg corresponds to turret facing backwards
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 255.0, 0.0, -1)
            } else {
                // reset position
                PedroComponent.follower.pose = Pose(resetModeParams.x, resetModeParams.y, resetModeParams.h.inRad)
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
            }
        }
        // I think l/r only makes sense when robot facing away (approx same direction person is facing)
        Gamepads.gamepad2.dpadRight whenBecomesTrue {
            phiTrim -= 2.0.deg
        }
        Gamepads.gamepad2.dpadLeft whenBecomesTrue {
            phiTrim += 2.0.deg
        }
    }

    var lastRuntime = 0.0
    override fun onUpdate() {
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

        PedroComponent.follower.update()


        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        val dxp = dx + vx * distanceToTime(dxy)
        val dyp = dy + vy * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)
        val hp = h + vh * distanceToTime(dxyp)

        /*   if (resetMode) {
               TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle, phiTrim).requires(TurretPhiSubsystem)()
               ShooterSubsystem.AutoAim(
                   dxyp,
                   distanceToVelocity
               )()
               TurretThetaSubsystem.AutoAim(
                   dxyp,
                   distanceToTheta
               )()
           } else if (autoAimEnabled) {
               ShooterSubsystem.AutoAim(
                   dxyp,
                   distanceToVelocity
               )()
   //            TurretPhiSubsystem.AutoAim(
   //                dxp, dyp, hp, phiTrim
   //            )()
               TurretThetaSubsystem.AutoAim(
                   dxyp,
                   distanceToTheta
               )()
           } else {
               //ShooterSubsystem.Manual(

               //)
           }

         */

        telemetry.addData("x (inch)", x);
        telemetry.addData("y (inch)", y);
        telemetry.addData("h (radians)", h);
        telemetry.addData(
            "distanceToGoal",
            hypot((goalX - x), (goalY - y))
        );
        // telemetry.addData("ShooterSpeed", speed1);
        //;  telemetry.addData("Angle", shootAngleDegrees.deg);
        telemetry.update()

//        PanelsTelemetry.telemetry.addData("Vx (in/s)", vx)
//        PanelsTelemetry.telemetry.addData("Vy (in/s)", vy)
        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }

    override fun onStop() {
        val file = File(Lefile.filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }
}