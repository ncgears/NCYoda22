package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class TwoMetersForward extends Path {
   private final static double[][] points = {
       {0,0.0,0.0,0.0,0.0,0.0,0.0},
       {0.034,0.0,0.0,0.0,0.0377,-0.0,-0.0},
       {0.068,0.0013,-0.0,-0.0,0.0753,-0.0,-0.0},
       {0.102,0.0038,-0.0,-0.0,0.113,-0.0,-0.0},
       {0.1359,0.0077,-0.0,-0.0,0.1506,-0.0,-0.0},
       {0.1699,0.0128,-0.0,-0.0,0.1883,-0.0,-0.0},
       {0.2039,0.0192,-0.0,-0.0,0.226,-0.0,-0.0},
       {0.2379,0.0269,-0.0,-0.0,0.2636,-0.0,0.0},
       {0.2719,0.0358,-0.0,-0.0,0.3013,-0.0,0.0},
       {0.3059,0.0461,-0.0,-0.0,0.339,-0.0,0.0},
       {0.3399,0.0576,-0.0,-0.0,0.3766,-0.0,0.0},
       {0.3739,0.0704,-0.0,-0.0,0.4143,-0.0,0.0},
       {0.4078,0.0845,-0.0,-0.0,0.4519,-0.0,0.0},
       {0.4418,0.0998,-0.0,-0.0,0.4896,-0.0,0.0},
       {0.4758,0.1165,-0.0,-0.0,0.5273,-0.0,-0.0},
       {0.5098,0.1344,-0.0,-0.0,0.5649,-0.0,-0.0},
       {0.5438,0.1536,-0.0,-0.0,0.6026,-0.0,0.0},
       {0.5778,0.1741,-0.0,-0.0,0.6402,-0.0,0.0},
       {0.6118,0.1958,-0.0,0.0,0.6779,-0.0,0.0},
       {0.6458,0.2189,-0.0,0.0,0.7156,-0.0,-0.0},
       {0.6797,0.2432,-0.0,-0.0,0.7532,-0.0,-0.0},
       {0.7137,0.2688,-0.0,-0.0,0.7909,-0.0,-0.0},
       {0.7477,0.2957,-0.0,-0.0,0.8286,-0.0,-0.0},
       {0.7817,0.3238,-0.0,-0.0,0.8662,-0.0,-0.0},
       {0.8157,0.3533,-0.0,-0.0,0.9039,-0.0,-0.0},
       {0.8497,0.384,-0.0,-0.0,0.9415,-0.0,0.0},
       {0.8837,0.416,-0.0,-0.0,0.9792,-0.0,-0.0},
       {0.9176,0.4493,-0.0,-0.0,1.0169,-0.0,0.0},
       {0.9516,0.4838,-0.0,-0.0,1.0545,-0.0,0.0},
       {0.9856,0.5197,-0.0,-0.0,1.0922,-0.0,0.0},
       {1.0196,0.5568,-0.0,-0.0,1.1298,-0.0,0.0},
       {1.0536,0.5952,-0.0,-0.0,1.1675,-0.0,0.0},
       {1.0876,0.6349,-0.0,-0.0,1.2052,-0.0,0.0},
       {1.1216,0.6758,-0.0,-0.0,1.2428,-0.0,-0.0},
       {1.1556,0.7181,-0.0,-0.0,1.2805,-0.0,-0.0},
       {1.1895,0.7616,-0.0,-0.0,1.3182,-0.0,-0.0},
       {1.2235,0.8064,-0.0,-0.0,1.3558,-0.0,-0.0},
       {1.2575,0.8525,-0.0,-0.0,1.3935,-0.0,0.0},
       {1.2915,0.8998,-0.0,-0.0,1.4311,-0.0,-0.0},
       {1.3255,0.9485,-0.0,-0.0,1.4688,-0.0,-0.0},
       {1.3595,0.9984,-0.0,-0.0,1.5065,-0.0,0.0},
       {1.3935,1.0496,-0.0,-0.0,1.5441,-0.0,0.0},
       {1.4275,1.1021,-0.0,-0.0,1.5818,-0.0,0.0},
       {1.4614,1.1558,-0.0,-0.0,1.6194,-0.0,0.0},
       {1.4954,1.2109,-0.0,-0.0,1.6571,-0.0,0.0},
       {1.5294,1.2672,-0.0,0.0,1.6948,-0.0,0.0},
       {1.5634,1.3248,-0.0,0.0,1.7324,-0.0,0.0},
       {1.5974,1.3837,-0.0,0.0,1.7701,-0.0,0.0},
       {1.6314,1.4438,-0.0,0.0,1.8078,-0.0,0.0},
       {1.6654,1.5053,-0.0,0.0,1.8454,-0.0,0.0},
       {1.6993,1.568,-0.0,0.0,1.8831,-0.0,0.0},
       {1.7333,1.632,-0.0,0.0,1.8454,0.0,0.0},
       {1.7673,1.6947,-0.0,0.0,1.8078,0.0,0.0},
       {1.8013,1.7562,-0.0,0.0,1.7701,0.0,0.0},
       {1.8353,1.8163,-0.0,0.0,1.7324,0.0,-0.0},
       {1.8693,1.8752,-0.0,0.0,1.6948,0.0,0.0},
       {1.9033,1.9328,-0.0,0.0,1.6571,0.0,-0.0},
       {1.9373,1.9891,-0.0,0.0,1.6194,0.0,0.0},
       {1.9712,2.0442,-0.0,0.0,1.5818,0.0,0.0},
       {2.0052,2.0979,-0.0,0.0,1.5441,0.0,0.0},
       {2.0392,2.1504,-0.0,0.0,1.5065,0.0,-0.0},
       {2.0732,2.2016,-0.0,0.0,1.4688,0.0,-0.0},
       {2.1072,2.2515,-0.0,0.0,1.4311,0.0,-0.0},
       {2.1412,2.3002,-0.0,0.0,1.3935,0.0,-0.0},
       {2.1752,2.3475,-0.0,0.0,1.3558,0.0,-0.0},
       {2.2092,2.3936,-0.0,0.0,1.3182,0.0,-0.0},
       {2.2431,2.4384,-0.0,0.0,1.2805,0.0,-0.0},
       {2.2771,2.4819,-0.0,0.0,1.2428,0.0,-0.0},
       {2.3111,2.5242,-0.0,0.0,1.2052,0.0,-0.0},
       {2.3451,2.5651,-0.0,0.0,1.1675,0.0,0.0},
       {2.3791,2.6048,-0.0,0.0,1.1298,0.0,0.0},
       {2.4131,2.6432,-0.0,0.0,1.0922,0.0,0.0},
       {2.4471,2.6803,-0.0,0.0,1.0545,0.0,0.0},
       {2.481,2.7162,-0.0,0.0,1.0169,0.0,0.0},
       {2.515,2.7507,-0.0,0.0,0.9792,0.0,-0.0},
       {2.549,2.784,-0.0,0.0,0.9415,0.0,-0.0},
       {2.583,2.816,-0.0,0.0,0.9039,0.0,-0.0},
       {2.617,2.8467,-0.0,0.0,0.8662,0.0,-0.0},
       {2.651,2.8762,-0.0,0.0,0.8286,0.0,-0.0},
       {2.685,2.9043,-0.0,0.0,0.7909,0.0,-0.0},
       {2.719,2.9312,-0.0,0.0,0.7532,0.0,-0.0},
       {2.7529,2.9568,-0.0,0.0,0.7156,0.0,-0.0},
       {2.7869,2.9811,-0.0,0.0,0.6779,0.0,-0.0},
       {2.8209,3.0042,-0.0,0.0,0.6402,0.0,0.0},
       {2.8549,3.0259,-0.0,0.0,0.6026,0.0,0.0},
       {2.8889,3.0464,-0.0,0.0,0.5649,0.0,-0.0},
       {2.9229,3.0656,-0.0,0.0,0.5273,0.0,-0.0},
       {2.9569,3.0835,-0.0,0.0,0.4896,0.0,0.0},
       {2.9908,3.1002,-0.0,0.0,0.4519,0.0,0.0},
       {3.0248,3.1155,-0.0,0.0,0.4143,0.0,0.0},
       {3.0588,3.1296,-0.0,0.0,0.3766,0.0,0.0},
       {3.0928,3.1424,-0.0,0.0,0.339,0.0,0.0},
       {3.1268,3.1539,-0.0,0.0,0.3013,0.0,0.0},
       {3.1608,3.1642,-0.0,0.0,0.2636,0.0,0.0},
       {3.1948,3.1731,-0.0,0.0,0.226,0.0,-0.0},
       {3.2288,3.1808,-0.0,0.0,0.1883,0.0,-0.0},
       {3.2627,3.1872,-0.0,0.0,0.1506,0.0,-0.0},
       {3.2967,3.1923,-0.0,0.0,0.113,0.0,-0.0},
       {3.3307,3.1962,-0.0,0.0,0.0753,0.0,-0.0},
       {3.3647,3.1987,-0.0,0.0,0.0377,0.0,-0.0},
       {3.3987,3.2,0.0,0.0,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
