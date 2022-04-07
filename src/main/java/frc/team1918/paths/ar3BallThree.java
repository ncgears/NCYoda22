package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class ar3BallThree extends Path {
   private final static double[][] points = {
       {0,-3.7,0.4,-0.4,0.0,0.0,0.0},
       {0.0442,-3.7,0.4,-0.4,-0.0489,0.0036,-0.0},
       {0.0884,-3.7022,0.4002,-0.4,-0.0977,0.0072,-0.0},
       {0.1326,-3.7065,0.4005,-0.4,-0.1466,0.0109,-0.0},
       {0.1768,-3.713,0.401,-0.4,-0.1954,0.0145,-0.0},
       {0.2211,-3.7216,0.4016,-0.4,-0.2443,0.0181,-0.0},
       {0.2653,-3.7324,0.4024,-0.4,-0.2931,0.0217,-0.0},
       {0.3095,-3.7454,0.4034,-0.4,-0.342,0.0253,-0.0},
       {0.3537,-3.7605,0.4045,-0.4,-0.3909,0.029,-0.0},
       {0.3979,-3.7778,0.4058,-0.4,-0.4397,0.0326,-0.0},
       {0.4421,-3.7972,0.4072,-0.4,-0.4886,0.0362,-0.0},
       {0.4863,-3.8188,0.4088,-0.4,-0.5374,0.0398,-0.0},
       {0.5305,-3.8426,0.4106,-0.4,-0.5863,0.0434,-0.0},
       {0.5747,-3.8685,0.4125,-0.4,-0.6351,0.047,-0.0},
       {0.619,-3.8966,0.4146,-0.4,-0.684,0.0507,-0.0},
       {0.6632,-3.9268,0.4168,-0.4,-0.7329,0.0543,-0.0},
       {0.7074,-3.9592,0.4192,-0.4,-0.7817,0.0579,-0.0},
       {0.7516,-3.9938,0.4218,-0.4,-0.8306,0.0615,-0.0},
       {0.7958,-4.0305,0.4245,-0.4,-0.8794,0.0651,-0.0},
       {0.84,-4.0694,0.4274,-0.4,-0.9283,0.0688,-0.0},
       {0.8842,-4.1104,0.4304,-0.4,-0.9771,0.0724,-0.0},
       {0.9284,-4.1536,0.4336,-0.4,-1.026,0.076,-0.0},
       {0.9726,-4.199,0.437,-0.4,-1.0749,0.0796,-0.0},
       {1.0168,-4.2465,0.4405,-0.4,-1.1237,0.0832,-0.0},
       {1.0611,-4.2962,0.4442,-0.4,-1.1726,0.0869,-0.0},
       {1.1053,-4.348,0.448,-0.4,-1.2214,0.0905,-0.0},
       {1.1495,-4.402,0.452,-0.4,-1.2703,0.0941,-0.0},
       {1.1937,-4.4582,0.4562,-0.4,-1.3191,0.0977,-0.0},
       {1.2379,-4.5165,0.4605,-0.4,-1.368,0.1013,-0.0},
       {1.2821,-4.577,0.465,-0.4,-1.4169,0.105,-0.0},
       {1.3263,-4.6396,0.4696,-0.4,-1.4657,0.1086,-0.0},
       {1.3705,-4.7044,0.4744,-0.4,-1.5146,0.1122,-0.0},
       {1.4147,-4.7714,0.4794,-0.4,-1.5634,0.1158,-0.0},
       {1.459,-4.8405,0.4845,-0.4,-1.6123,0.1194,-0.0},
       {1.5032,-4.9118,0.4898,-0.4,-1.6611,0.123,-0.0},
       {1.5474,-4.9852,0.4952,-0.4,-1.71,0.1267,-0.0},
       {1.5916,-5.0608,0.5008,-0.4,-1.7588,0.1303,-0.0},
       {1.6358,-5.1386,0.5066,-0.4,-1.8077,0.1339,-0.0},
       {1.68,-5.2185,0.5125,-0.4,-1.8566,0.1375,-0.0},
       {1.7242,-5.3006,0.5186,-0.4,-1.9054,0.1411,-0.0},
       {1.7684,-5.3848,0.5248,-0.4,-1.9543,0.1448,0.0},
       {1.8126,-5.4712,0.5312,-0.4,-2.0031,0.1484,0.0},
       {1.8569,-5.5598,0.5378,-0.4,-2.052,0.152,0.0},
       {1.9011,-5.6505,0.5445,-0.4,-2.1008,0.1556,0.0},
       {1.9453,-5.7434,0.5514,-0.4,-2.1497,0.1592,0.0},
       {1.9895,-5.8384,0.5584,-0.4,-2.1986,0.1629,0.0},
       {2.0337,-5.9356,0.5656,-0.4,-2.2474,0.1665,0.0},
       {2.0779,-6.035,0.573,-0.4,-2.2963,0.1701,0.0},
       {2.1221,-6.1365,0.5805,-0.4,-2.3451,0.1737,0.0},
       {2.1663,-6.2402,0.5882,-0.4,-2.394,0.1773,0.0},
       {2.2105,-6.346,0.596,-0.4,-2.4428,0.181,0.0},
       {2.2547,-6.454,0.604,-0.4,-2.394,0.1773,0.0},
       {2.299,-6.5598,0.6118,-0.4,-2.3451,0.1737,-0.0},
       {2.3432,-6.6635,0.6195,-0.4,-2.2963,0.1701,-0.0},
       {2.3874,-6.765,0.627,-0.4,-2.2474,0.1665,-0.0},
       {2.4316,-6.8644,0.6344,-0.4,-2.1986,0.1629,-0.0},
       {2.4758,-6.9616,0.6416,-0.4,-2.1497,0.1592,-0.0},
       {2.52,-7.0566,0.6486,-0.4,-2.1008,0.1556,-0.0},
       {2.5642,-7.1495,0.6555,-0.4,-2.052,0.152,-0.0},
       {2.6084,-7.2402,0.6622,-0.4,-2.0031,0.1484,-0.0},
       {2.6526,-7.3288,0.6688,-0.4,-1.9543,0.1448,-0.0},
       {2.6969,-7.4152,0.6752,-0.4,-1.9054,0.1411,-0.0},
       {2.7411,-7.4994,0.6814,-0.4,-1.8566,0.1375,-0.0},
       {2.7853,-7.5815,0.6875,-0.4,-1.8077,0.1339,-0.0},
       {2.8295,-7.6614,0.6934,-0.4,-1.7588,0.1303,-0.0},
       {2.8737,-7.7392,0.6992,-0.4,-1.71,0.1267,-0.0},
       {2.9179,-7.8148,0.7048,-0.4,-1.6611,0.123,-0.0},
       {2.9621,-7.8882,0.7102,-0.4,-1.6123,0.1194,-0.0},
       {3.0063,-7.9595,0.7155,-0.4,-1.5634,0.1158,-0.0},
       {3.0505,-8.0286,0.7206,-0.4,-1.5146,0.1122,-0.0},
       {3.0948,-8.0956,0.7256,-0.4,-1.4657,0.1086,-0.0},
       {3.139,-8.1604,0.7304,-0.4,-1.4169,0.105,-0.0},
       {3.1832,-8.223,0.735,-0.4,-1.368,0.1013,-0.0},
       {3.2274,-8.2835,0.7395,-0.4,-1.3191,0.0977,-0.0},
       {3.2716,-8.3418,0.7438,-0.4,-1.2703,0.0941,-0.0},
       {3.3158,-8.398,0.748,-0.4,-1.2214,0.0905,-0.0},
       {3.36,-8.452,0.752,-0.4,-1.1726,0.0869,-0.0},
       {3.4042,-8.5038,0.7558,-0.4,-1.1237,0.0832,-0.0},
       {3.4484,-8.5535,0.7595,-0.4,-1.0749,0.0796,-0.0},
       {3.4926,-8.601,0.763,-0.4,-1.026,0.076,-0.0},
       {3.5369,-8.6464,0.7664,-0.4,-0.9771,0.0724,-0.0},
       {3.5811,-8.6896,0.7696,-0.4,-0.9283,0.0688,-0.0},
       {3.6253,-8.7306,0.7726,-0.4,-0.8794,0.0651,-0.0},
       {3.6695,-8.7695,0.7755,-0.4,-0.8306,0.0615,-0.0},
       {3.7137,-8.8062,0.7782,-0.4,-0.7817,0.0579,-0.0},
       {3.7579,-8.8408,0.7808,-0.4,-0.7329,0.0543,-0.0},
       {3.8021,-8.8732,0.7832,-0.4,-0.684,0.0507,-0.0},
       {3.8463,-8.9034,0.7854,-0.4,-0.6351,0.047,-0.0},
       {3.8905,-8.9315,0.7875,-0.4,-0.5863,0.0434,-0.0},
       {3.9348,-8.9574,0.7894,-0.4,-0.5374,0.0398,-0.0},
       {3.979,-8.9812,0.7912,-0.4,-0.4886,0.0362,-0.0},
       {4.0232,-9.0028,0.7928,-0.4,-0.4397,0.0326,-0.0},
       {4.0674,-9.0222,0.7942,-0.4,-0.3909,0.029,-0.0},
       {4.1116,-9.0395,0.7955,-0.4,-0.342,0.0253,-0.0},
       {4.1558,-9.0546,0.7966,-0.4,-0.2931,0.0217,-0.0},
       {4.2,-9.0676,0.7976,-0.4,-0.2443,0.0181,-0.0},
       {4.2442,-9.0784,0.7984,-0.4,-0.1954,0.0145,-0.0},
       {4.2884,-9.087,0.799,-0.4,-0.1466,0.0109,-0.0},
       {4.3327,-9.0935,0.7995,-0.4,-0.0977,0.0072,0.0},
       {4.3769,-9.0978,0.7998,-0.4,-0.0489,0.0036,-0.0},
       {4.4211,-9.1,0.8,-0.4,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
