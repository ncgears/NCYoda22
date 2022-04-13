package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class al1BallOne extends Path {
   private final static double[][] points = {
       {0,0.0,0.0,0.0,-0.0,0.0,0.0},
       {0.0287,-0.0,0.0,0.0,-0.0307,-0.0084,0.0028},
       {0.0574,-0.0009,-0.0002,0.0001,-0.0613,-0.0167,0.0055},
       {0.0861,-0.0026,-0.0007,0.0002,-0.092,-0.0251,0.0083},
       {0.1148,-0.0053,-0.0014,0.0005,-0.1227,-0.0335,0.0111},
       {0.1435,-0.0088,-0.0024,0.0008,-0.1534,-0.0418,0.0139},
       {0.1722,-0.0132,-0.0036,0.0012,-0.184,-0.0502,0.0166},
       {0.2008,-0.0185,-0.005,0.0017,-0.2147,-0.0586,0.0194},
       {0.2295,-0.0246,-0.0067,0.0022,-0.2454,-0.0669,0.0222},
       {0.2582,-0.0317,-0.0086,0.0029,-0.276,-0.0753,0.025},
       {0.2869,-0.0396,-0.0108,0.0036,-0.3067,-0.0836,0.0277},
       {0.3156,-0.0484,-0.0132,0.0044,-0.3374,-0.092,0.0305},
       {0.3443,-0.0581,-0.0158,0.0053,-0.368,-0.1004,0.0333},
       {0.373,-0.0686,-0.0187,0.0062,-0.3987,-0.1087,0.036},
       {0.4017,-0.0801,-0.0218,0.0072,-0.4294,-0.1171,0.0388},
       {0.4304,-0.0924,-0.0252,0.0084,-0.4601,-0.1255,0.0416},
       {0.4591,-0.1056,-0.0288,0.0095,-0.4907,-0.1338,0.0444},
       {0.4878,-0.1197,-0.0326,0.0108,-0.5214,-0.1422,0.0471},
       {0.5165,-0.1346,-0.0367,0.0122,-0.5521,-0.1506,0.0499},
       {0.5451,-0.1505,-0.041,0.0136,-0.5827,-0.1589,0.0527},
       {0.5738,-0.1672,-0.0456,0.0151,-0.6134,-0.1673,0.0555},
       {0.6025,-0.1848,-0.0504,0.0167,-0.6441,-0.1757,0.0582},
       {0.6312,-0.2033,-0.0554,0.0184,-0.6748,-0.184,0.061},
       {0.6599,-0.2226,-0.0607,0.0201,-0.7054,-0.1924,0.0638},
       {0.6886,-0.2429,-0.0662,0.022,-0.7361,-0.2008,0.0666},
       {0.7173,-0.264,-0.072,0.0239,-0.7668,-0.2091,0.0694},
       {0.746,-0.286,-0.078,0.0259,-0.7974,-0.2175,0.0721},
       {0.7747,-0.3089,-0.0842,0.0279,-0.8281,-0.2258,0.0749},
       {0.8034,-0.3326,-0.0907,0.0301,-0.8588,-0.2342,0.0777},
       {0.8321,-0.3573,-0.0974,0.0323,-0.8895,-0.2426,0.0805},
       {0.8608,-0.3828,-0.1044,0.0346,-0.9201,-0.2509,0.0833},
       {0.8894,-0.4092,-0.1116,0.037,-0.9508,-0.2593,0.086},
       {0.9181,-0.4365,-0.119,0.0395,-0.9815,-0.2677,0.0888},
       {0.9468,-0.4646,-0.1267,0.042,-1.0121,-0.276,0.0916},
       {0.9755,-0.4937,-0.1346,0.0447,-1.0428,-0.2844,0.0944},
       {1.0042,-0.5236,-0.1428,0.0474,-1.0735,-0.2928,0.0972},
       {1.0329,-0.5544,-0.1512,0.0501,-1.1041,-0.3011,0.1},
       {1.0616,-0.5861,-0.1598,0.053,-1.1348,-0.3095,0.1028},
       {1.0903,-0.6186,-0.1687,0.056,-1.1655,-0.3179,0.1055},
       {1.119,-0.6521,-0.1778,0.059,-1.1962,-0.3262,0.1083},
       {1.1477,-0.6864,-0.1872,0.0621,-1.2268,-0.3346,0.1111},
       {1.1764,-0.7216,-0.1968,0.0653,-1.2575,-0.343,0.1139},
       {1.2051,-0.7577,-0.2066,0.0686,-1.2882,-0.3513,0.1167},
       {1.2337,-0.7946,-0.2167,0.0719,-1.3188,-0.3597,0.1195},
       {1.2624,-0.8325,-0.227,0.0753,-1.3495,-0.368,0.1223},
       {1.2911,-0.8712,-0.2376,0.0788,-1.3802,-0.3764,0.1251},
       {1.3198,-0.9108,-0.2484,0.0824,-1.4109,-0.3848,0.1279},
       {1.3485,-0.9513,-0.2594,0.0861,-1.4415,-0.3931,0.1308},
       {1.3772,-0.9926,-0.2707,0.0899,-1.4722,-0.4015,0.1336},
       {1.4059,-1.0349,-0.2822,0.0937,-1.5029,-0.4099,0.1365},
       {1.4346,-1.078,-0.294,0.0976,-1.5335,-0.4182,0.1394},
       {1.4633,-1.122,-0.306,0.1016,-1.5029,-0.4099,0.1369},
       {1.492,-1.1651,-0.3178,0.1055,-1.4722,-0.4015,0.1341},
       {1.5207,-1.2074,-0.3293,0.1094,-1.4415,-0.3931,0.1314},
       {1.5494,-1.2487,-0.3406,0.1132,-1.4109,-0.3848,0.1286},
       {1.5781,-1.2892,-0.3516,0.1168,-1.3802,-0.3764,0.1259},
       {1.6067,-1.3288,-0.3624,0.1205,-1.3495,-0.3681,0.1231},
       {1.6354,-1.3675,-0.373,0.124,-1.3188,-0.3597,0.1203},
       {1.6641,-1.4054,-0.3833,0.1274,-1.2882,-0.3513,0.1175},
       {1.6928,-1.4423,-0.3934,0.1308,-1.2575,-0.343,0.1147},
       {1.7215,-1.4784,-0.4032,0.1341,-1.2268,-0.3346,0.112},
       {1.7502,-1.5136,-0.4128,0.1373,-1.1962,-0.3262,0.1092},
       {1.7789,-1.5479,-0.4222,0.1404,-1.1655,-0.3179,0.1064},
       {1.8076,-1.5814,-0.4313,0.1435,-1.1348,-0.3095,0.1036},
       {1.8363,-1.6139,-0.4402,0.1465,-1.1041,-0.3011,0.1008},
       {1.865,-1.6456,-0.4488,0.1494,-1.0735,-0.2928,0.098},
       {1.8937,-1.6764,-0.4572,0.1522,-1.0428,-0.2844,0.0952},
       {1.9224,-1.7063,-0.4654,0.1549,-1.0121,-0.276,0.0924},
       {1.951,-1.7354,-0.4733,0.1576,-0.9815,-0.2677,0.0896},
       {1.9797,-1.7635,-0.481,0.1601,-0.9508,-0.2593,0.0868},
       {2.0084,-1.7908,-0.4884,0.1626,-0.9201,-0.2509,0.084},
       {2.0371,-1.8172,-0.4956,0.165,-0.8895,-0.2426,0.0812},
       {2.0658,-1.8427,-0.5026,0.1674,-0.8588,-0.2342,0.0784},
       {2.0945,-1.8674,-0.5093,0.1696,-0.8281,-0.2259,0.0756},
       {2.1232,-1.8911,-0.5158,0.1718,-0.7974,-0.2175,0.0728},
       {2.1519,-1.914,-0.522,0.1739,-0.7668,-0.2091,0.07},
       {2.1806,-1.936,-0.528,0.1759,-0.7361,-0.2008,0.0672},
       {2.2093,-1.9571,-0.5338,0.1778,-0.7054,-0.1924,0.0644},
       {2.238,-1.9774,-0.5393,0.1797,-0.6748,-0.184,0.0616},
       {2.2667,-1.9967,-0.5446,0.1814,-0.6441,-0.1757,0.0588},
       {2.2953,-2.0152,-0.5496,0.1831,-0.6134,-0.1673,0.056},
       {2.324,-2.0328,-0.5544,0.1847,-0.5827,-0.1589,0.0532},
       {2.3527,-2.0495,-0.559,0.1863,-0.5521,-0.1506,0.0504},
       {2.3814,-2.0654,-0.5633,0.1877,-0.5214,-0.1422,0.0476},
       {2.4101,-2.0803,-0.5674,0.1891,-0.4907,-0.1338,0.0448},
       {2.4388,-2.0944,-0.5712,0.1904,-0.4601,-0.1255,0.042},
       {2.4675,-2.1076,-0.5748,0.1916,-0.4294,-0.1171,0.0392},
       {2.4962,-2.1199,-0.5782,0.1927,-0.3987,-0.1087,0.0364},
       {2.5249,-2.1314,-0.5813,0.1937,-0.368,-0.1004,0.0336},
       {2.5536,-2.1419,-0.5842,0.1947,-0.3374,-0.092,0.0308},
       {2.5823,-2.1516,-0.5868,0.1956,-0.3067,-0.0836,0.028},
       {2.611,-2.1604,-0.5892,0.1964,-0.276,-0.0753,0.0252},
       {2.6396,-2.1683,-0.5914,0.1971,-0.2454,-0.0669,0.0224},
       {2.6683,-2.1754,-0.5933,0.1977,-0.2147,-0.0586,0.0196},
       {2.697,-2.1815,-0.595,0.1983,-0.184,-0.0502,0.0168},
       {2.7257,-2.1868,-0.5964,0.1988,-0.1534,-0.0418,0.014},
       {2.7544,-2.1912,-0.5976,0.1992,-0.1227,-0.0335,0.0112},
       {2.7831,-2.1947,-0.5986,0.1995,-0.092,-0.0251,0.0084},
       {2.8118,-2.1974,-0.5993,0.1998,-0.0613,-0.0167,0.0056},
       {2.8405,-2.1991,-0.5998,0.1999,-0.0307,-0.0084,0.0028},
       {2.8692,-2.2,-0.6,0.2,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}