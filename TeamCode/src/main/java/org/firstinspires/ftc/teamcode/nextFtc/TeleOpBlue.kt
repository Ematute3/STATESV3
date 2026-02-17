package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad

/**
 * Blue Alliance TeleOp
 *
 * Field positions (inches) - adjust based on your field layout:
 * - Goal position: Left side (Blue alliance)
 * - Starting position: Right side of field
 */
@TeleOp(name = "BlueTeleOp")
class BlueTeleOp : TeleOpBase(
    isBlue = true,                           // Blue alliance
    goalX = 55.0,                            // Blue goal X (adjust to your field)
    goalY = -35.0,                           // Blue goal Y (opposite side)
    resetModeParams = ResetModeParams(
        x = 12.0,                            // Blue starting X
        y = -60.0,                           // Blue starting Y (negative for opposite side)
        h = 0.0.deg                          // Blue facing right (0°)
    ),
    distanceToTime = { distance ->
        // Tune this formula for your robot's speed
        distance / 40.0                      // 40 inches/second base speed
    }
)