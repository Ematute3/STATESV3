package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem

@Configurable
@TeleOp(name = "Shooter1 F Test", group = "Base Subsystem Tests")
class ShooterFTest : NextFTCOpMode() {
    companion object {
        @JvmField var speed = 0.0;
    }

    init {
        addComponents(
            SubsystemComponent(ShooterSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
//        telemetry.addData("test", "hi");
        CommandManager.cancelAll()
        ShooterSubsystem.On(speed)();
        PanelsTelemetry.telemetry.update();
    }
}