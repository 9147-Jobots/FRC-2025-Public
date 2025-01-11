// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionConstants {

    //measurements for transform3d
    public final class cameraFront { // 0.41 meters from the center backwards, 0.0 meters to the right, 0.2 meters up, 0.0 radians pitch, 0.0 radians roll, and pi radians/180 degreees yaw
        public static final double x = 0;
        public static final double y = 0;
        public static final double z = 0;
        public static final double pitch = 0;
        public static final double roll = 0;
        public static final double yaw = Math.PI;
    }
    public final class cameraBack {
        public static final double x = 0;
        public static final double y = 0;
        public static final double z = 0;
        public static final double pitch = 0;
        public static final double roll = 0;
        public static final double yaw = 0;
    }

    public final class PoseEstimatorConstants {
        public static final Matrix<N3, N1> stateStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{0.1, 0.1, 0.1});
    }
}
