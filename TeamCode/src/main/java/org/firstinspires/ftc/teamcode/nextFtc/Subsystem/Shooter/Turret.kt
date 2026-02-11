package org.firstinspires.ftc.teamcode.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier
import kotlin.math.*

object Turret : Subsystem {

    // ============================================
    // HARDWARE & CONFIGURATION
    // ============================================

    enum class State { IDLE, MANUAL, VISION_AIM, ODOMETRY_AIM }
    enum class Alliance { RED, BLUE }

    private val motor = MotorEx("turret")

    // Encoder calibration points (Code 1 style)
    @JvmField var ENCODERS_FORWARD = 1367.0
    @JvmField var ENCODERS_BACKWARD = 0.0

    // Physical constants (Code 2 style)
    const val GEAR_RATIO = 3.62068965517  // 105/29
    const val MOTOR_TICKS_PER_REV = 537.7
    private const val RADIANS_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)

    // Physical limits
    const val MIN_ANGLE = -3.0 * PI / 4.0  // -135 degrees
    const val MAX_ANGLE = 3.0 * PI / 4.0   // +135 degrees

    // Field goals (alliance-aware)
    @JvmField var alliance = Alliance.RED
    const val GOAL_Y = 144.0
    const val RED_GOAL_X = 144.0
    const val BLUE_GOAL_X = 0.0
    val goalY = GOAL_Y
    val goalX: Double get() = if (alliance == Alliance.RED) RED_GOAL_X else BLUE_GOAL_X

    // ============================================
    // CONTROL SYSTEM
    // ============================================

    private val controller: ControlSystem
    @JvmField var squidCoefficients = PIDCoefficients(0.002, 0.0, 0.0)
    @JvmField var basicFF = Triple(0.25, 0.0, 0.0)  // kV, kA, kS

    // Control parameters
    @JvmField var minPower: Double = 0.15       // Overcome static friction
    @JvmField var maxPower: Double = 0.75       // Safety limit
    @JvmField var alignmentTolerance: Double = 2.0  // Degrees
    @JvmField var velocityCompensationGain: Double = 0.25  // Robot rotation compensation

    init {
        controller = ControlSystem()
            .posSquID(squidCoefficients)  // Smooth motion for turrets
            .build()
        controller.goal = KineticState()
    }

    // ============================================
    // STATE MANAGEMENT
    // ============================================

    var currentState = State.IDLE
        private set

    private var manualPower = 0.0
    private var lastCommand: Command? = null

    // Velocity tracking for compensation
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    private var robotAngularVelocity = 0.0

    // Odometry integration (inject these from your drive subsystem)
    var getCurrentX: () -> Double = { 0.0 }
    var getCurrentY: () -> Double = { 0.0 }
    var getCurrentHeading: () -> Double = { 0.0 }
    var isPoseValid: () -> Boolean = { false }

    // ============================================
    // ANGLE UTILITIES
    // ============================================

    /**
     * Normalizes angle to [-PI, PI] and wraps to minimize rotation distance
     * Combines both implementations' strengths
     */
    fun normalizeAngle(angle: Angle, centerPoint: Double = 0.0): Angle {
        var a = angle.inRad

        // First normalize to [-PI, PI]
        a %= (2.0 * PI)
        if (a <= -PI) a += 2.0 * PI
        if (a > PI) a -= 2.0 * PI

        // Then wrap around center point to minimize rotation
        val tolerance = PI / 6
        while (a < centerPoint - 2 * PI - tolerance) {
            a += 2 * PI
        }
        while (a > centerPoint + tolerance) {
            a -= 2 * PI
        }

        return a.rad
    }

    /**
     * Clamps angle to physical limits
     */
    fun clampAngle(angle: Angle): Angle {
        return angle.inRad.coerceIn(MIN_ANGLE, MAX_ANGLE).rad
    }

    /**
     * Current turret angle relative to robot
     */
    val turretYaw: Angle
        get() = (motor.currentPosition * RADIANS_PER_TICK).rad

    /**
     * Target angle in robot-relative coordinates
     */
    val targetPhi: Angle
        get() = normalizeAngle(
            PI.rad * (motor.currentPosition - ENCODERS_FORWARD) /
                    (ENCODERS_FORWARD - ENCODERS_BACKWARD)
        )

    // ============================================
    // ENCODER CALIBRATION
    // ============================================

    fun reset() {
        val d = ENCODERS_FORWARD - ENCODERS_BACKWARD
        ENCODERS_FORWARD = motor.currentPosition
        ENCODERS_BACKWARD = ENCODERS_FORWARD - d
    }

    // ============================================
    // CONTROL COMMANDS
    // ============================================

    /**
     * Vision-based aiming (Limelight, AprilTag, etc.)
     * Takes direct dx, dy measurements to target
     */
    class VisionAim(
        private val dx: Supplier<Double>,      // Delta X to goal (continuous)
        private val dy: Supplier<Double>,      // Delta Y to goal (continuous)
        private val robotHeading: Supplier<Angle>,  // Current robot heading
        private val ofsTurret: Angle = 0.0.rad
    ) : Command() {

        override val isDone = false  // Continuous tracking

        override fun start() {
            currentState = State.VISION_AIM
            if (lastCommand != null && lastCommand != this) {
                CommandManager.cancelCommand(lastCommand!!)
            }
            lastCommand = this
        }

        override fun update() {
            val targetAngle = normalizeAngle(
                atan2(dy.get(), dx.get()).rad - robotHeading.get() + ofsTurret
            )
            setTargetAngle(targetAngle, compensateVelocity = true)
        }

        override fun stop(interrupted: Boolean) {
            if (!interrupted) currentState = State.IDLE
        }
    }

    /**
     * Odometry-based aiming - aims at field goal using pose estimation
     */
    class OdometryAim(
        private val ofsTurret: Angle = 0.0.rad
    ) : Command() {

        override val isDone = false  // Continuous tracking

        override fun start() {
            currentState = State.ODOMETRY_AIM
            if (lastCommand != null && lastCommand != this) {
                CommandManager.cancelCommand(lastCommand!!)
            }
            lastCommand = this
        }

        override fun update() {
            if (!isPoseValid()) return

            val deltaX = goalX - getCurrentX()
            val deltaY = goalY - getCurrentY()
            val fieldAngle = atan2(deltaY, deltaX)

            val robotHeading = getCurrentHeading().let { heading ->
                if (abs(heading) > 2.0 * PI) Math.toRadians(heading) else heading
            }

            val targetAngle = normalizeAngle(
                (fieldAngle - robotHeading).rad + ofsTurret
            )

            setTargetAngle(targetAngle, compensateVelocity = true)
        }

        override fun stop(interrupted: Boolean) {
            if (!interrupted) currentState = State.IDLE
        }
    }

    /**
     * One-shot aim command (fires and completes)
     */
    class AimOnce(
        private val dx: Double,
        private val dy: Double,
        private val robotHeading: Angle,
        private val ofsTurret: Angle = 0.0.rad
    ) : Command() {

        override val isDone = true

        override fun start() {
            if (lastCommand != null) {
                CommandManager.cancelCommand(lastCommand!!)
            }

            val targetAngle = normalizeAngle(
                atan2(dy, dx).rad - robotHeading + ofsTurret
            )
            setTargetAngle(targetAngle, compensateVelocity = false)
            lastCommand = this
        }
    }

    /**
     * Manual control for tuning/testing
     */
    class Manual(
        private val powerSupplier: Supplier<Double>
    ) : Command() {

        override val isDone = false

        override fun start() {
            currentState = State.MANUAL
            if (lastCommand != null && lastCommand != this) {
                CommandManager.cancelCommand(lastCommand!!)
            }
            lastCommand = this
        }

        override fun update() {
            manualPower = powerSupplier.get()
        }

        override fun stop(interrupted: Boolean) {
            manualPower = 0.0
            if (!interrupted) currentState = State.IDLE
        }
    }

    /**
     * Stop all motion
     */
    fun stop() {
        if (lastCommand != null) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        currentState = State.IDLE
        motor.power = 0.0
    }

    // ============================================
    // INTERNAL CONTROL
    // ============================================

    /**
     * Sets target angle with optional velocity compensation
     */
    private fun setTargetAngle(targetAngle: Angle, compensateVelocity: Boolean) {
        // Clamp to physical limits
        val clampedAngle = clampAngle(targetAngle)

        // Convert to encoder position
        val encoderTarget = (clampedAngle.inRad / PI) *
                (ENCODERS_FORWARD - ENCODERS_BACKWARD) + ENCODERS_FORWARD

        // Set goal with velocity compensation
        val targetVelocity = if (compensateVelocity) {
            -robotAngularVelocity * velocityCompensationGain
        } else {
            0.0
        }

        controller.goal = KineticState(encoderTarget, targetVelocity)
    }

    /**
     * Updates robot angular velocity for compensation
     */
    private fun updateRobotVelocity() {
        val dt = velTimer.seconds()
        if (dt > 0.001) {
            val currentHeading = getCurrentHeading()
            var deltaHeading = currentHeading - lastRobotHeading

            // Normalize delta
            if (deltaHeading > PI) deltaHeading -= 2 * PI
            if (deltaHeading < -PI) deltaHeading += 2 * PI

            robotAngularVelocity = deltaHeading / dt
            lastRobotHeading = currentHeading
            velTimer.reset()
        }
    }

    /**
     * Applies minimum power threshold to overcome friction
     */
    private fun applyMinimumPower(power: Double, error: Double): Double {
        val errorDeg = Math.toDegrees(abs(error))

        return if (errorDeg > alignmentTolerance) {
            // Add minimum power to overcome static friction
            power + (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            // Within tolerance - stop if not tracking velocity
            if (abs(controller.goal.velocity) < 0.01) 0.0 else power
        }
    }

    // ============================================
    // MAIN LOOP
    // ============================================

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        currentState = State.IDLE
    }

    override fun periodic() {
        updateRobotVelocity()

        when (currentState) {
            State.IDLE -> {
                motor.power = 0.0
            }

            State.MANUAL -> {
                motor.power = manualPower.coerceIn(-maxPower, maxPower)
            }

            State.VISION_AIM, State.ODOMETRY_AIM -> {
                // Calculate control output from SQUID controller
                var power = controller.calculate(motor.state)

                // Add basic feedforward
                power += basicFF.first * controller.goal.velocity

                // Apply minimum power threshold
                val error = controller.goal.position - motor.currentPosition
                power = applyMinimumPower(power, error)

                // Clamp and apply
                motor.power = power.coerceIn(-maxPower, maxPower)
            }
        }

        // Telemetry
        PanelsTelemetry.telemetry.addData("Turret State", currentState.name)
        PanelsTelemetry.telemetry.addData("Turret Yaw (rad)", turretYaw.inRad)
        PanelsTelemetry.telemetry.addData("Turret Yaw (deg)", Math.toDegrees(turretYaw.inRad))
        PanelsTelemetry.telemetry.addData("Motor Encoder", motor.currentPosition)
        PanelsTelemetry.telemetry.addData("Controller Goal", controller.goal.position)
        PanelsTelemetry.telemetry.addData("Controller Ref", controller.reference)
        PanelsTelemetry.telemetry.addData("Motor Power", motor.power)
        PanelsTelemetry.telemetry.addData("Robot AngVel (rad/s)", robotAngularVelocity)
        PanelsTelemetry.telemetry.addData("Alliance", alliance.name)
    }
}