package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.robotState.StateManager;

import frc.robot.util.Constants.Subsystem1Constants;

public class Subsystem1 extends SubsystemBase {
    SparkMax motor1 = new SparkMax(Subsystem1Constants.MOTOR1_CAN_ID, MotorType.kBrushless);
    SparkMax motor2 = new SparkMax(Subsystem1Constants.MOTOR2_CAN_ID, MotorType.kBrushless);
    SparkMax motor3 = new SparkMax(Subsystem1Constants.MOTOR3_CAN_ID, MotorType.kBrushless);

    public Subsystem1() {
        //cool stuff
    }
    //Add the cool functions that do function thingies cooler
    //MAKE SURE TO USE INSTANT COMMANDS > VOID METHODS => EVERYTHING IS A COMMAND :)
    public Command coolFunction() {
        // Implementation of the cool function
        return new InstantCommand(() -> {
            // Cool function logic here
            //Make it cooler :)
        });
    }
}