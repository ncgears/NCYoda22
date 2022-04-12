package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class al2BallOne extends Path {
   private final static double[][] points = {
       {0,0.0,0.0,0.0,0.0,-0.0,0.0},
       {0.02,0.0,-0.0,0.0,-0.0449,0.0071,0.014},
       {0.04,-0.0009,0.0001,0.0003,-0.0899,0.0142,0.0279},
       {0.06,-0.0027,0.0004,0.0008,-0.1349,0.0212,0.0419},
       {0.08,-0.0054,0.0009,0.0017,-0.1798,0.0282,0.0558},
       {0.1,-0.009,0.0014,0.0028,-0.2248,0.035,0.0697},
       {0.12,-0.0135,0.0021,0.0042,-0.2698,0.0418,0.0836},
       {0.14,-0.0189,0.003,0.0059,-0.3148,0.0486,0.0974},
       {0.16,-0.0252,0.0039,0.0078,-0.3599,0.0552,0.1113},
       {0.18,-0.0324,0.005,0.01,-0.4049,0.0618,0.125},
       {0.2,-0.0405,0.0063,0.0125,-0.45,0.0682,0.1388},
       {0.22,-0.0495,0.0076,0.0153,-0.495,0.0746,0.1525},
       {0.24,-0.0594,0.0091,0.0184,-0.5401,0.0808,0.1662},
       {0.26,-0.0702,0.0107,0.0217,-0.5852,0.087,0.1799},
       {0.28,-0.0819,0.0125,0.0253,-0.6303,0.0931,0.1935},
       {0.3,-0.0945,0.0143,0.0291,-0.6754,0.099,0.2071},
       {0.32,-0.108,0.0163,0.0333,-0.7206,0.1048,0.2207},
       {0.34,-0.1224,0.0184,0.0377,-0.7657,0.1105,0.2342},
       {0.36,-0.1377,0.0206,0.0424,-0.8109,0.116,0.2476},
       {0.38,-0.1539,0.0229,0.0473,-0.8561,0.1214,0.261},
       {0.3999,-0.171,0.0254,0.0526,-0.9013,0.1267,0.2743},
       {0.4199,-0.1891,0.0279,0.058,-0.9465,0.1318,0.2876},
       {0.4399,-0.208,0.0305,0.0638,-0.9918,0.1367,0.3008},
       {0.4599,-0.2278,0.0333,0.0698,-1.037,0.1414,0.314},
       {0.4799,-0.2486,0.0361,0.0761,-1.0823,0.1459,0.3271},
       {0.4999,-0.2702,0.039,0.0826,-1.1277,0.1502,0.34},
       {0.5199,-0.2928,0.042,0.0894,-1.173,0.1542,0.353},
       {0.5399,-0.3162,0.0451,0.0965,-1.2183,0.158,0.3658},
       {0.5599,-0.3406,0.0483,0.1038,-1.2637,0.1616,0.3785},
       {0.5799,-0.3658,0.0515,0.1114,-1.3091,0.1648,0.3911},
       {0.5999,-0.392,0.0548,0.1192,-1.3546,0.1677,0.4036},
       {0.6199,-0.4191,0.0581,0.1273,-1.4,0.1702,0.4159},
       {0.6399,-0.4471,0.0615,0.1356,-1.4455,0.1724,0.4281},
       {0.6599,-0.476,0.065,0.1441,-1.491,0.1741,0.4402},
       {0.6799,-0.5058,0.0685,0.1529,-1.5365,0.1753,0.4521},
       {0.6999,-0.5366,0.072,0.162,-1.582,0.176,0.4637},
       {0.7199,-0.5682,0.0755,0.1713,-1.6275,0.1761,0.4752},
       {0.7399,-0.6007,0.079,0.1808,-1.673,0.1755,0.4863},
       {0.7599,-0.6342,0.0825,0.1905,-1.7186,0.1741,0.4972},
       {0.7799,-0.6686,0.086,0.2004,-1.764,0.1718,0.5077},
       {0.7999,-0.7038,0.0895,0.2106,-1.8094,0.1685,0.5179},
       {0.8199,-0.74,0.0928,0.2209,-1.8547,0.1639,0.5275},
       {0.8399,-0.7771,0.0961,0.2315,-1.8999,0.1579,0.5365},
       {0.8599,-0.8151,0.0993,0.2422,-1.9448,0.1501,0.5448},
       {0.8799,-0.854,0.1023,0.2531,-1.9892,0.1403,0.5522},
       {0.8999,-0.8938,0.1051,0.2642,-2.033,0.1278,0.5584},
       {0.9199,-0.9344,0.1076,0.2753,-2.0758,0.112,0.5631},
       {0.9399,-0.9759,0.1099,0.2866,-2.1168,0.0922,0.5657},
       {0.9599,-1.0183,0.1117,0.2979,-2.1549,0.0672,0.5657},
       {0.9799,-1.0614,0.113,0.3092,-2.1882,0.0362,0.5623},
       {0.9999,-1.1051,0.1138,0.3204,-2.214,-0.0014,0.5549},
       {1.0199,-1.1494,0.1137,0.3315,-2.229,-0.0444,0.5439},
       {1.0399,-1.194,0.1129,0.3424,-2.2315,-0.0898,0.5301},
       {1.0599,-1.2386,0.1111,0.353,-2.2224,-0.1344,0.5147},
       {1.0799,-1.283,0.1084,0.3633,-2.2043,-0.1761,0.4984},
       {1.0999,-1.3271,0.1049,0.3733,-2.1798,-0.2144,0.4817},
       {1.1199,-1.3707,0.1006,0.3829,-2.1509,-0.2495,0.4649},
       {1.1399,-1.4137,0.0956,0.3922,-2.1188,-0.2817,0.4481},
       {1.1599,-1.4561,0.0899,0.4012,-2.0846,-0.3116,0.4315},
       {1.1799,-1.4978,0.0837,0.4098,-2.0487,-0.3396,0.4149},
       {1.1998,-1.5388,0.0769,0.4181,-2.0116,-0.3659,0.3986},
       {1.2198,-1.579,0.0696,0.4261,-1.9736,-0.3908,0.3823},
       {1.2398,-1.6185,0.0618,0.4337,-1.9348,-0.4145,0.3662},
       {1.2598,-1.6571,0.0535,0.441,-1.8953,-0.4372,0.3503},
       {1.2798,-1.695,0.0448,0.448,-1.8554,-0.459,0.3345},
       {1.2998,-1.7321,0.0356,0.4547,-1.8151,-0.4801,0.3188},
       {1.3198,-1.7684,0.026,0.4611,-1.7745,-0.5005,0.3032},
       {1.3398,-1.8039,0.016,0.4672,-1.7335,-0.5203,0.2877},
       {1.3598,-1.8386,0.0056,0.4729,-1.6923,-0.5395,0.2723},
       {1.3798,-1.8724,-0.0052,0.4784,-1.6508,-0.5583,0.2569},
       {1.3998,-1.9054,-0.0164,0.4835,-1.6092,-0.5766,0.2417},
       {1.4198,-1.9376,-0.0279,0.4883,-1.5674,-0.5946,0.2265},
       {1.4398,-1.969,-0.0398,0.4929,-1.5255,-0.6122,0.2114},
       {1.4598,-1.9995,-0.0521,0.4971,-1.4834,-0.6294,0.1964},
       {1.4798,-2.0291,-0.0646,0.501,-1.4412,-0.6464,0.1814},
       {1.4998,-2.058,-0.0776,0.5047,-1.3989,-0.6631,0.1665},
       {1.5198,-2.0859,-0.0908,0.508,-1.3564,-0.6795,0.1516},
       {1.5398,-2.1131,-0.1044,0.511,-1.3139,-0.6958,0.1368},
       {1.5598,-2.1393,-0.1183,0.5137,-1.2713,-0.7118,0.122},
       {1.5798,-2.1648,-0.1326,0.5162,-1.2287,-0.7276,0.1073},
       {1.5998,-2.1893,-0.1471,0.5183,-1.1859,-0.7432,0.0925},
       {1.6198,-2.213,-0.162,0.5202,-1.1431,-0.7586,0.0779},
       {1.6398,-2.2359,-0.1771,0.5217,-1.1003,-0.7739,0.0632},
       {1.6598,-2.2579,-0.1926,0.523,-1.0574,-0.789,0.0486},
       {1.6798,-2.2791,-0.2084,0.524,-1.0144,-0.804,0.034},
       {1.6998,-2.2993,-0.2245,0.5247,-0.9714,-0.8188,0.0194},
       {1.7198,-2.3188,-0.2408,0.525,-0.9283,-0.8335,0.0048},
       {1.7398,-2.3373,-0.2575,0.5251,-0.8853,-0.8481,-0.0097},
       {1.7598,-2.355,-0.2745,0.5249,-0.8421,-0.8626,-0.0242},
       {1.7798,-2.3719,-0.2917,0.5245,-0.7989,-0.8769,-0.0387},
       {1.7998,-2.3879,-0.3093,0.5237,-0.7557,-0.8912,-0.0533},
       {1.8198,-2.403,-0.3271,0.5226,-0.7125,-0.9053,-0.0677},
       {1.8398,-2.4172,-0.3452,0.5213,-0.6692,-0.9194,-0.0822},
       {1.8598,-2.4306,-0.3636,0.5196,-0.6259,-0.9333,-0.0967},
       {1.8798,-2.4431,-0.3822,0.5177,-0.5826,-0.9472,-0.1112},
       {1.8998,-2.4548,-0.4012,0.5155,-0.5392,-0.961,-0.1257},
       {1.9198,-2.4655,-0.4204,0.513,-0.4959,-0.9748,-0.1402},
       {1.9398,-2.4755,-0.4399,0.5101,-0.4525,-0.9884,-0.1547},
       {1.9598,-2.4845,-0.4597,0.5071,-0.409,-1.002,-0.1692},
       {1.9798,-2.4927,-0.4797,0.5037,-0.3656,-1.0155,-0.1837},
       {1.9997,-2.5,-0.5,0.5,-0.3221,-1.0289,-0.1982},
       {2.016,-2.5052,-0.5168,0.4968,-0.2867,-1.0398,-0.21},
       {2.0323,-2.5099,-0.5337,0.4934,-0.2512,-1.0506,-0.2218},
       {2.0486,-2.514,-0.5508,0.4897,-0.2158,-1.0614,-0.2336},
       {2.0649,-2.5175,-0.5681,0.4859,-0.1803,-1.072,-0.2454},
       {2.0812,-2.5205,-0.5856,0.4819,-0.1448,-1.0826,-0.2572},
       {2.0975,-2.5228,-0.6032,0.4777,-0.1092,-1.093,-0.2691},
       {2.1138,-2.5246,-0.621,0.4734,-0.0736,-1.1034,-0.2809},
       {2.1301,-2.5258,-0.639,0.4688,-0.038,-1.1137,-0.2927},
       {2.1463,-2.5264,-0.6571,0.464,-0.0024,-1.1239,-0.3045},
       {2.1626,-2.5265,-0.6754,0.4591,0.0333,-1.134,-0.3163},
       {2.1789,-2.5259,-0.6939,0.4539,0.0689,-1.144,-0.3281},
       {2.1952,-2.5248,-0.7125,0.4486,0.1047,-1.1538,-0.3399},
       {2.2115,-2.5231,-0.7313,0.443,0.1404,-1.1635,-0.3516},
       {2.2278,-2.5208,-0.7503,0.4373,0.1762,-1.1731,-0.3634},
       {2.2441,-2.5179,-0.7694,0.4314,0.2121,-1.1826,-0.3752},
       {2.2604,-2.5145,-0.7886,0.4253,0.2479,-1.1919,-0.387},
       {2.2766,-2.5104,-0.808,0.419,0.2838,-1.2011,-0.3988},
       {2.2929,-2.5058,-0.8276,0.4125,0.3198,-1.21,-0.4105},
       {2.3092,-2.5006,-0.8473,0.4058,0.3558,-1.2189,-0.4223},
       {2.3255,-2.4948,-0.8672,0.3989,0.3918,-1.2275,-0.4341},
       {2.3418,-2.4884,-0.8872,0.3918,0.4279,-1.2359,-0.4458},
       {2.3581,-2.4815,-0.9073,0.3846,0.4641,-1.2441,-0.4575},
       {2.3744,-2.4739,-0.9276,0.3771,0.5003,-1.252,-0.4692},
       {2.3907,-2.4657,-0.9479,0.3695,0.5365,-1.2598,-0.4809},
       {2.4069,-2.457,-0.9685,0.3617,0.5728,-1.2672,-0.4926},
       {2.4232,-2.4477,-0.9891,0.3536,0.6092,-1.2743,-0.5042},
       {2.4395,-2.4378,-1.0099,0.3454,0.6456,-1.2812,-0.5158},
       {2.4558,-2.4272,-1.0307,0.337,0.6821,-1.2876,-0.5274},
       {2.4721,-2.4161,-1.0517,0.3284,0.7187,-1.2937,-0.5389},
       {2.4884,-2.4044,-1.0728,0.3196,0.7553,-1.2993,-0.5504},
       {2.5047,-2.3921,-1.0939,0.3107,0.792,-1.3045,-0.5617},
       {2.521,-2.3792,-1.1152,0.3015,0.8288,-1.3092,-0.573},
       {2.5373,-2.3657,-1.1365,0.2922,0.8656,-1.3132,-0.5842},
       {2.5535,-2.3516,-1.1579,0.2827,0.9026,-1.3167,-0.5953},
       {2.5698,-2.3369,-1.1793,0.273,0.9395,-1.3193,-0.6063},
       {2.5861,-2.3216,-1.2008,0.2631,0.9766,-1.3212,-0.617},
       {2.6024,-2.3057,-1.2224,0.2531,1.0136,-1.322,-0.6275},
       {2.6187,-2.2892,-1.2439,0.2428,1.0507,-1.3218,-0.6378},
       {2.635,-2.2721,-1.2654,0.2325,1.0877,-1.3202,-0.6477},
       {2.6513,-2.2544,-1.2869,0.2219,1.1247,-1.3171,-0.6571},
       {2.6676,-2.2361,-1.3084,0.2112,1.1615,-1.3122,-0.666},
       {2.6838,-2.2171,-1.3297,0.2004,1.1979,-1.3052,-0.6741},
       {2.7001,-2.1976,-1.351,0.1894,1.2337,-1.2955,-0.6813},
       {2.7164,-2.1775,-1.3721,0.1783,1.2685,-1.2828,-0.6873},
       {2.7327,-2.1569,-1.393,0.1671,1.3018,-1.2664,-0.6916},
       {2.749,-2.1357,-1.4136,0.1558,1.3326,-1.2457,-0.694},
       {2.7653,-2.114,-1.4339,0.1445,1.3597,-1.2203,-0.694},
       {2.7816,-2.0918,-1.4538,0.1332,1.3815,-1.1903,-0.6914},
       {2.7979,-2.0693,-1.4732,0.122,1.3968,-1.1565,-0.6865},
       {2.8141,-2.0466,-1.492,0.1108,1.4047,-1.1203,-0.6795},
       {2.8304,-2.0237,-1.5103,0.0997,1.4055,-1.0832,-0.6708},
       {2.8467,-2.0008,-1.5279,0.0888,1.4,-1.0465,-0.661},
       {2.863,-1.978,-1.545,0.078,1.3895,-1.011,-0.6501},
       {2.8793,-1.9554,-1.5614,0.0674,1.375,-0.9769,-0.6386},
       {2.8956,-1.933,-1.5773,0.057,1.3575,-0.9443,-0.6264},
       {2.9119,-1.9108,-1.5927,0.0468,1.3375,-0.9131,-0.6139},
       {2.9282,-1.8891,-1.6076,0.0368,1.3156,-0.8832,-0.6011},
       {2.9444,-1.8676,-1.622,0.027,1.2922,-0.8544,-0.588},
       {2.9607,-1.8466,-1.6359,0.0174,1.2676,-0.8267,-0.5747},
       {2.977,-1.8259,-1.6493,0.0081,1.242,-0.7999,-0.5613},
       {2.9933,-1.8057,-1.6624,-0.0011,1.2156,-0.774,-0.5477},
       {3.0096,-1.7859,-1.675,-0.01,1.1885,-0.7487,-0.5341},
       {3.0259,-1.7665,-1.6872,-0.0187,1.1608,-0.7241,-0.5204},
       {3.0422,-1.7476,-1.699,-0.0272,1.1326,-0.7001,-0.5066},
       {3.0585,-1.7292,-1.7104,-0.0354,1.104,-0.6766,-0.4927},
       {3.0748,-1.7112,-1.7214,-0.0434,1.075,-0.6536,-0.4788},
       {3.091,-1.6937,-1.732,-0.0512,1.0456,-0.631,-0.4649},
       {3.1073,-1.6767,-1.7423,-0.0588,1.016,-0.6088,-0.4509},
       {3.1236,-1.6601,-1.7522,-0.0661,0.986,-0.587,-0.4369},
       {3.1399,-1.6441,-1.7618,-0.0733,0.9559,-0.5654,-0.4229},
       {3.1562,-1.6285,-1.771,-0.0801,0.9255,-0.5442,-0.4089},
       {3.1725,-1.6134,-1.7799,-0.0868,0.895,-0.5233,-0.3948},
       {3.1888,-1.5988,-1.7884,-0.0932,0.8643,-0.5026,-0.3807},
       {3.2051,-1.5848,-1.7966,-0.0994,0.8334,-0.4821,-0.3667},
       {3.2213,-1.5712,-1.8044,-0.1054,0.8024,-0.4619,-0.3526},
       {3.2376,-1.5581,-1.812,-0.1112,0.7712,-0.4419,-0.3385},
       {3.2539,-1.5456,-1.8192,-0.1167,0.7399,-0.422,-0.3243},
       {3.2702,-1.5335,-1.826,-0.122,0.7085,-0.4024,-0.3102},
       {3.2865,-1.522,-1.8326,-0.127,0.677,-0.3829,-0.2961},
       {3.3028,-1.5109,-1.8388,-0.1318,0.6454,-0.3635,-0.282},
       {3.3191,-1.5004,-1.8447,-0.1364,0.6137,-0.3444,-0.2679},
       {3.3354,-1.4904,-1.8503,-0.1408,0.582,-0.3253,-0.2537},
       {3.3516,-1.481,-1.8556,-0.1449,0.5501,-0.3064,-0.2396},
       {3.3679,-1.472,-1.8606,-0.1488,0.5182,-0.2876,-0.2255},
       {3.3842,-1.4636,-1.8653,-0.1525,0.4862,-0.269,-0.2113},
       {3.4005,-1.4556,-1.8697,-0.1559,0.4541,-0.2504,-0.1972},
       {3.4168,-1.4482,-1.8738,-0.1591,0.422,-0.232,-0.1831},
       {3.4331,-1.4414,-1.8776,-0.1621,0.3898,-0.2136,-0.169},
       {3.4494,-1.435,-1.881,-0.1649,0.3576,-0.1954,-0.1549},
       {3.4657,-1.4292,-1.8842,-0.1674,0.3253,-0.1772,-0.1407},
       {3.4819,-1.4239,-1.8871,-0.1697,0.2929,-0.1592,-0.1266},
       {3.4982,-1.4191,-1.8897,-0.1718,0.2606,-0.1412,-0.1125},
       {3.5145,-1.4149,-1.892,-0.1736,0.2281,-0.1233,-0.0985},
       {3.5308,-1.4112,-1.894,-0.1752,0.1956,-0.1055,-0.0844},
       {3.5471,-1.408,-1.8957,-0.1766,0.1631,-0.0877,-0.0703},
       {3.5634,-1.4053,-1.8972,-0.1777,0.1306,-0.07,-0.0562},
       {3.5797,-1.4032,-1.8983,-0.1786,0.098,-0.0524,-0.0421},
       {3.596,-1.4016,-1.8991,-0.1793,0.0654,-0.0349,-0.0281},
       {3.6123,-1.4005,-1.8997,-0.1798,0.0327,-0.0174,-0.014},
       {3.6285,-1.4,-1.9,-0.18,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
