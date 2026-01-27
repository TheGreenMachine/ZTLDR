package com.team1816.season;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.BaseRobotContainer;
import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Indexer;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotContainer extends BaseRobotContainer {
    public static Mechanism2d inclineMech2d = new Mechanism2d (3, 3);
    public static MechanismRoot2d inclineRoot2d = inclineMech2d.getRoot("inclineHood", 2, 0);
    public static MechanismLigament2d m_inclineRoot;
    public static MechanismLigament2d m_inclineHood;

    public RobotContainer() {

        NamedCommands.registerCommand("InTheZone", new InTheZoneCommand());
        // call the base to initialize library objects
        // i.e. subsystems that always exist like the drivetrain and path planner
        initializeLibSubSystems();

        Singleton.CreateSubSystem(Indexer.class);
        Singleton.CreateSubSystem(Shooter.class);

        superstructure = new Superstructure(swerve);

        m_inclineRoot = inclineRoot2d.append(new MechanismLigament2d("inclineRoot", 2, 90));
        m_inclineHood =
            m_inclineRoot.append(
                new MechanismLigament2d("inclineHood", 1, 7, 6, new Color8Bit(Color.kPurple)));

        SmartDashboard.putData("Mech2d", inclineMech2d);

        initializeAutonomous();

        configureBindings();
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    private void configureBindings() {
        controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IN))
            .or(controller.rightTrigger().and(controller.leftTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_OUT)))
            .or(controller.leftTrigger().and(controller.rightTrigger()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IDLE)))
            .or(controller.leftTrigger().negate().and(controller.rightTrigger().negate()).whileTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_IDLE)));
//        controller.y().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_0))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
//        controller.a().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_TO_180))
//            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.TURRET_IDLE));
    }
}
