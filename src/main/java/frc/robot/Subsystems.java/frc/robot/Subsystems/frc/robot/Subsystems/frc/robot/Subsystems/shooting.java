
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class shooting extends SubsystemBase {
    private final SparkMax inputMotor = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax outputMotor = new SparkMax(3, MotorType.kBrushed);

    public void intake(){
        inputMotor.set(0.7);
    }

    public void output(){
        outputMotor.set(0.7);
    }


}

