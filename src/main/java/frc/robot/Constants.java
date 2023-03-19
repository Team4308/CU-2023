package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public final class Constants {
    public static class Mapping {
        public static class Drive {
            public static int frontLeft = 0;
            public static int backLeft = 2;
            public static int frontRight = 1;
            public static int backRight = 3;
        }

        public static class Arm {
            public static int motor1 = 4;
            public static int motor2 = 5;
        }

        public static class Claw {
            public static int PCM = 8;

        }
    }

    public static class Generic {
        public static int timeoutMs = 3000;
    }

    public static class Config {
        public static class Input {
            public static double kInputDeadband = 0.14;

            public static class Stick {
                public static double kInputScale = 2.0;
            }
        }

        public static class Arm {
            public static class AngleControl {
                public static double kP = 0.0002;
                public static double kI = 0.0001;
                public static double kD = 0.0;
            }
            public static class AutoAngleControl {
                public static double kP = 0.003;
                public static double kI = 0.0001;
                public static double kD = 0.0;
            }


            public static class ExtensionControl {
                public static double kP = 0.00001;
                public static double kI = 0.0;
                public static double kD = 0.0;
            }
        }

        public static class Drive {
            public static class Kinematics {
                public static final double kWheelDiameter = 6; // In Inches
                public static final double kInchesPerRotation = kWheelDiameter * Math.PI;
                public static final double kSensorUnitsPerRotation = 2048; // 2048 for talonfx
                public static final double kEncoderInchesPerCount = kInchesPerRotation / kSensorUnitsPerRotation;
                // input, small cluster, / large cluster, output
                public static final double kGearRatio = (14.0 * 24.0) / (50.0 * 50.0);
            }

            public static class Power {
                public static double kOpenLoopRamp = 0.0;
                public static double kClosedLoopRamp = 0.0;

                public static StatorCurrentLimitConfiguration kStatorCurrentLimit = new StatorCurrentLimitConfiguration(
                        true, 35, 40, 100);
            }

            public static class AngleControl {
                public static double kP = 0.005;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kTolerance = 0.5;
            }

            public static class RangeControl {
                public static double kP = 0.01;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kTolerance = 1.0;
            }

            public static class PitchControl {
                public static double kP = 0.007;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kTolerance = 2.0;
            }

            public static class VelocityControl {
                public static int profileSlot = 0;

                public static class Left {
                    public static double kP = 0.1;
                    public static double kI = 0.0;
                    public static double kD = 1.45;
                    public static double kF = 0.0468;
                }

                public static class Right {
                    public static double kP = 0.055;
                    public static double kI = 0.0;
                    public static double kD = 1.45;
                    public static double kF = 0.0468;
                }
            }

            public static class MotionMagic {
                public static int profileSlot = 1;

                public static int maxVel = 16000;
                public static int maxAcc = 6000;

                public static class Left {
                    public static double kP = 0.0001;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }

                public static class Right {
                    public static double kP = 0.0001;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }
            }
        }
    }

    public static class DynConfig {
        public static class Drive {
            public static double VelocityDriveRPM = 4000;
        }
    }
}