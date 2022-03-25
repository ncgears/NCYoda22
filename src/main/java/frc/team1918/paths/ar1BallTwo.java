package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class ar1BallTwo extends Path {
   private final static double[][] points = {
       {0,-0.6,1.8,0.0,0.0,-0.0,0.0},
       {0.0168,-0.6,1.8,0.0,0.0052,-0.0177,-0.0149},
       {0.0336,-0.5999,1.7997,-0.0002,0.0102,-0.0355,-0.0297},
       {0.0504,-0.5997,1.7991,-0.0007,0.0152,-0.0532,-0.0446},
       {0.0673,-0.5995,1.7982,-0.0015,0.02,-0.071,-0.0595},
       {0.0841,-0.5992,1.797,-0.0025,0.0247,-0.0889,-0.0745},
       {0.1009,-0.5987,1.7955,-0.0038,0.0293,-0.1067,-0.0894},
       {0.1177,-0.5982,1.7937,-0.0053,0.0338,-0.1246,-0.1044},
       {0.1345,-0.5977,1.7916,-0.007,0.0382,-0.1426,-0.1194},
       {0.1513,-0.597,1.7892,-0.009,0.0425,-0.1605,-0.1344},
       {0.1681,-0.5963,1.7865,-0.0113,0.0466,-0.1785,-0.1495},
       {0.1849,-0.5955,1.7835,-0.0138,0.0506,-0.1965,-0.1646},
       {0.2018,-0.5947,1.7802,-0.0166,0.0544,-0.2145,-0.1797},
       {0.2186,-0.5938,1.7766,-0.0196,0.0581,-0.2326,-0.1948},
       {0.2354,-0.5928,1.7727,-0.0229,0.0616,-0.2507,-0.2099},
       {0.2522,-0.5918,1.7685,-0.0264,0.065,-0.2689,-0.2251},
       {0.269,-0.5907,1.764,-0.0302,0.0682,-0.287,-0.2403},
       {0.2858,-0.5895,1.7592,-0.0342,0.0712,-0.3052,-0.2555},
       {0.3026,-0.5883,1.754,-0.0385,0.074,-0.3234,-0.2707},
       {0.3195,-0.5871,1.7486,-0.0431,0.0767,-0.3417,-0.286},
       {0.3363,-0.5858,1.7428,-0.0479,0.0791,-0.36,-0.3013},
       {0.3531,-0.5845,1.7368,-0.0529,0.0814,-0.3783,-0.3166},
       {0.3699,-0.5831,1.7304,-0.0583,0.0834,-0.3966,-0.3319},
       {0.3867,-0.5817,1.7238,-0.0638,0.0852,-0.415,-0.3472},
       {0.4035,-0.5803,1.7168,-0.0697,0.0867,-0.4333,-0.3626},
       {0.4203,-0.5788,1.7095,-0.0758,0.088,-0.4517,-0.378},
       {0.4371,-0.5773,1.7019,-0.0821,0.0891,-0.4701,-0.3934},
       {0.454,-0.5758,1.694,-0.0887,0.0898,-0.4886,-0.4088},
       {0.4708,-0.5743,1.6858,-0.0956,0.0903,-0.507,-0.4242},
       {0.4876,-0.5728,1.6773,-0.1027,0.0905,-0.5254,-0.4396},
       {0.5044,-0.5713,1.6684,-0.1101,0.0904,-0.5439,-0.455},
       {0.5212,-0.5698,1.6593,-0.1178,0.0899,-0.5623,-0.4704},
       {0.538,-0.5682,1.6498,-0.1257,0.0891,-0.5807,-0.4858},
       {0.5548,-0.5667,1.6401,-0.1339,0.0879,-0.5991,-0.5012},
       {0.5717,-0.5653,1.63,-0.1423,0.0863,-0.6175,-0.5165},
       {0.5885,-0.5638,1.6196,-0.151,0.0843,-0.6358,-0.5318},
       {0.6053,-0.5624,1.6089,-0.1599,0.0819,-0.6541,-0.5471},
       {0.6221,-0.561,1.5979,-0.1691,0.079,-0.6723,-0.5623},
       {0.6389,-0.5597,1.5866,-0.1786,0.0757,-0.6904,-0.5774},
       {0.6557,-0.5584,1.575,-0.1883,0.0718,-0.7085,-0.5924},
       {0.6725,-0.5572,1.5631,-0.1982,0.0675,-0.7264,-0.6074},
       {0.6893,-0.5561,1.5509,-0.2084,0.0626,-0.7442,-0.6222},
       {0.7062,-0.555,1.5384,-0.2189,0.0572,-0.7618,-0.6369},
       {0.723,-0.5541,1.5256,-0.2296,0.0511,-0.7793,-0.6514},
       {0.7398,-0.5532,1.5125,-0.2406,0.0445,-0.7965,-0.6657},
       {0.7566,-0.5525,1.4991,-0.2518,0.0372,-0.8135,-0.6798},
       {0.7734,-0.5518,1.4854,-0.2632,0.0292,-0.8302,-0.6936},
       {0.7902,-0.5513,1.4714,-0.2749,0.0206,-0.8465,-0.7072},
       {0.807,-0.551,1.4572,-0.2867,0.0114,-0.8625,-0.7205},
       {0.8239,-0.5508,1.4427,-0.2989,0.0014,-0.8781,-0.7334},
       {0.8407,-0.5508,1.4279,-0.3112,-0.0093,-0.8932,-0.746},
       {0.8575,-0.5509,1.4129,-0.3237,-0.0207,-0.9078,-0.7581},
       {0.8743,-0.5513,1.3976,-0.3365,-0.0328,-0.9218,-0.7698},
       {0.8911,-0.5518,1.3822,-0.3494,-0.0455,-0.9353,-0.7811},
       {0.9079,-0.5526,1.3664,-0.3626,-0.059,-0.948,-0.7918},
       {0.9247,-0.5536,1.3505,-0.3759,-0.073,-0.9601,-0.8021},
       {0.9415,-0.5548,1.3343,-0.3893,-0.0877,-0.9715,-0.8117},
       {0.9584,-0.5563,1.318,-0.403,-0.103,-0.982,-0.8208},
       {0.9752,-0.558,1.3015,-0.4168,-0.1188,-0.9918,-0.8292},
       {0.992,-0.56,1.2848,-0.4307,-0.1351,-1.0008,-0.837},
       {1.0088,-0.5623,1.268,-0.4448,-0.1518,-1.0089,-0.8442},
       {1.0256,-0.5648,1.251,-0.459,-0.1689,-1.0162,-0.8506},
       {1.0424,-0.5677,1.2339,-0.4733,-0.1864,-1.0226,-0.8563},
       {1.0592,-0.5708,1.2168,-0.4877,-0.2041,-1.0282,-0.8614},
       {1.076,-0.5743,1.1995,-0.5022,-0.2221,-1.0329,-0.8657},
       {1.0929,-0.578,1.1821,-0.5167,-0.2403,-1.0369,-0.8692},
       {1.1097,-0.582,1.1647,-0.5314,-0.2587,-1.0401,-0.8721},
       {1.1265,-0.5864,1.1472,-0.546,-0.2772,-1.0425,-0.8743},
       {1.1433,-0.591,1.1297,-0.5607,-0.2957,-1.0441,-0.8758},
       {1.1601,-0.596,1.1121,-0.5754,-0.3143,-1.0451,-0.8766},
       {1.1769,-0.6013,1.0945,-0.5902,-0.333,-1.0454,-0.8768},
       {1.1937,-0.6069,1.077,-0.6049,-0.3516,-1.045,-0.8764},
       {1.2106,-0.6128,1.0594,-0.6197,-0.3702,-1.044,-0.8754},
       {1.2274,-0.619,1.0418,-0.6344,-0.3887,-1.0425,-0.8739},
       {1.2442,-0.6256,1.0243,-0.6491,-0.4073,-1.0404,-0.8718},
       {1.261,-0.6324,1.0068,-0.6637,-0.4257,-1.0377,-0.8692},
       {1.2778,-0.6396,0.9894,-0.6783,-0.4441,-1.0346,-0.8661},
       {1.2946,-0.647,0.972,-0.6929,-0.4623,-1.031,-0.8625},
       {1.3114,-0.6548,0.9546,-0.7074,-0.4805,-1.027,-0.8585},
       {1.3282,-0.6629,0.9374,-0.7218,-0.4986,-1.0226,-0.8541},
       {1.3451,-0.6713,0.9202,-0.7362,-0.5166,-1.0177,-0.8494},
       {1.3619,-0.68,0.9031,-0.7505,-0.5344,-1.0125,-0.8442},
       {1.3787,-0.6889,0.886,-0.7647,-0.5522,-1.007,-0.8387},
       {1.3955,-0.6982,0.8691,-0.7788,-0.5699,-1.0011,-0.8329},
       {1.4123,-0.7078,0.8523,-0.7928,-0.5874,-0.9949,-0.8268},
       {1.4291,-0.7177,0.8355,-0.8067,-0.6048,-0.9884,-0.8203},
       {1.4459,-0.7279,0.8189,-0.8205,-0.6221,-0.9816,-0.8136},
       {1.4628,-0.7383,0.8024,-0.8342,-0.6394,-0.9746,-0.8066},
       {1.4796,-0.7491,0.786,-0.8477,-0.6565,-0.9673,-0.7994},
       {1.4964,-0.7601,0.7698,-0.8612,-0.6735,-0.9597,-0.792},
       {1.5132,-0.7714,0.7536,-0.8745,-0.6904,-0.952,-0.7843},
       {1.53,-0.783,0.7376,-0.8877,-0.7071,-0.944,-0.7764},
       {1.5468,-0.7949,0.7218,-0.9007,-0.7238,-0.9358,-0.7683},
       {1.5636,-0.8071,0.706,-0.9136,-0.7404,-0.9274,-0.76},
       {1.5804,-0.8195,0.6904,-0.9264,-0.7569,-0.9189,-0.7516},
       {1.5973,-0.8323,0.675,-0.939,-0.7733,-0.9101,-0.7429},
       {1.6141,-0.8453,0.6597,-0.9515,-0.7896,-0.9012,-0.7341},
       {1.6309,-0.8585,0.6445,-0.9639,-0.8058,-0.8921,-0.7252},
       {1.6477,-0.8721,0.6295,-0.9761,-0.822,-0.8829,-0.7161},
       {1.6645,-0.8859,0.6147,-0.9881,-0.838,-0.8736,-0.7069},
       {1.6813,-0.9,0.6,-1.0,-0.8539,-0.8641,-0.6975},
       {1.7128,-0.9269,0.5728,-1.022,-0.8838,-0.8462,-0.6799},
       {1.7443,-0.9547,0.5461,-1.0434,-0.9136,-0.8282,-0.6624},
       {1.7758,-0.9835,0.52,-1.0643,-0.9433,-0.8101,-0.6448},
       {1.8073,-1.0132,0.4945,-1.0846,-0.9729,-0.7919,-0.6273},
       {1.8388,-1.0439,0.4696,-1.1043,-1.0025,-0.7736,-0.6097},
       {1.8703,-1.0755,0.4452,-1.1235,-1.0321,-0.7552,-0.5921},
       {1.9018,-1.108,0.4214,-1.1422,-1.0615,-0.7367,-0.5746},
       {1.9333,-1.1414,0.3982,-1.1603,-1.0909,-0.718,-0.557},
       {1.9648,-1.1758,0.3756,-1.1778,-1.1202,-0.6993,-0.5394},
       {1.9963,-1.211,0.3536,-1.1948,-1.1494,-0.6804,-0.5217},
       {2.0278,-1.2473,0.3321,-1.2112,-1.1785,-0.6613,-0.504},
       {2.0593,-1.2844,0.3113,-1.2271,-1.2075,-0.6421,-0.4862},
       {2.0908,-1.3224,0.2911,-1.2424,-1.2364,-0.6228,-0.4684},
       {2.1223,-1.3614,0.2715,-1.2572,-1.2652,-0.6033,-0.4505},
       {2.1538,-1.4012,0.2525,-1.2714,-1.2939,-0.5836,-0.4325},
       {2.1853,-1.442,0.2341,-1.285,-1.3224,-0.5637,-0.4145},
       {2.2168,-1.4836,0.2163,-1.2981,-1.3508,-0.5436,-0.3963},
       {2.2483,-1.5262,0.1992,-1.3105,-1.3791,-0.5233,-0.378},
       {2.2798,-1.5696,0.1827,-1.3225,-1.4071,-0.5028,-0.3595},
       {2.3113,-1.6139,0.1669,-1.3338,-1.435,-0.482,-0.3409},
       {2.3428,-1.6591,0.1517,-1.3445,-1.4627,-0.461,-0.3221},
       {2.3743,-1.7052,0.1372,-1.3547,-1.4901,-0.4396,-0.3031},
       {2.4058,-1.7522,0.1233,-1.3642,-1.5174,-0.418,-0.2838},
       {2.4373,-1.8,0.1102,-1.3731,-1.5443,-0.396,-0.2643},
       {2.4688,-1.8486,0.0977,-1.3815,-1.5709,-0.3737,-0.2446},
       {2.5003,-1.8981,0.0859,-1.3892,-1.5972,-0.351,-0.2245},
       {2.5318,-1.9484,0.0749,-1.3962,-1.6232,-0.3279,-0.204},
       {2.5633,-1.9995,0.0645,-1.4027,-1.6487,-0.3043,-0.1832},
       {2.5948,-2.0515,0.0549,-1.4084,-1.6737,-0.2802,-0.1619},
       {2.6263,-2.1042,0.0461,-1.4135,-1.6981,-0.2555,-0.1401},
       {2.6578,-2.1577,0.0381,-1.418,-1.7219,-0.2302,-0.1177},
       {2.6893,-2.2119,0.0308,-1.4217,-1.7449,-0.2042,-0.0946},
       {2.7208,-2.2669,0.0244,-1.4246,-1.7671,-0.1775,-0.0708},
       {2.7523,-2.3225,0.0188,-1.4269,-1.7881,-0.15,-0.0462},
       {2.7838,-2.3789,0.0141,-1.4283,-1.808,-0.1215,-0.0206},
       {2.8153,-2.4358,0.0102,-1.429,-1.8263,-0.0921,0.006},
       {2.8468,-2.4933,0.0073,-1.4288,-1.8427,-0.0616,0.0338},
       {2.8783,-2.5514,0.0054,-1.4277,-1.8568,-0.0301,0.0629},
       {2.9098,-2.6099,0.0045,-1.4257,-1.8682,0.0026,0.0933},
       {2.9413,-2.6687,0.0045,-1.4228,-1.8761,0.0363,0.1246},
       {2.9728,-2.7278,0.0057,-1.4189,-1.8798,0.0706,0.1565},
       {3.0043,-2.787,0.0079,-1.4139,-1.8787,0.1052,0.1878},
       {3.0358,-2.8462,0.0112,-1.408,-1.872,0.1391,0.2173},
       {3.0673,-2.9052,0.0156,-1.4012,-1.8596,0.1715,0.2438},
       {3.0988,-2.9637,0.021,-1.3935,-1.8417,0.2012,0.2668},
       {3.1303,-3.0218,0.0273,-1.3851,-1.819,0.2275,0.2859},
       {3.1618,-3.0791,0.0345,-1.3761,-1.7926,0.2502,0.3014},
       {3.1933,-3.1355,0.0424,-1.3666,-1.7634,0.2692,0.3137},
       {3.2248,-3.1911,0.0509,-1.3567,-1.7322,0.2848,0.3232},
       {3.2563,-3.2456,0.0598,-1.3465,-1.6997,0.2974,0.3304},
       {3.2878,-3.2992,0.0692,-1.3361,-1.6662,0.3073,0.3357},
       {3.3193,-3.3517,0.0789,-1.3256,-1.6322,0.315,0.3394},
       {3.3508,-3.4031,0.0888,-1.3149,-1.5978,0.3208,0.3417},
       {3.3823,-3.4534,0.0989,-1.3041,-1.5631,0.3249,0.3428},
       {3.4138,-3.5026,0.1091,-1.2933,-1.5283,0.3276,0.3429},
       {3.4453,-3.5508,0.1195,-1.2825,-1.4934,0.329,0.3421},
       {3.4768,-3.5978,0.1298,-1.2717,-1.4585,0.3293,0.3406},
       {3.5083,-3.6438,0.1402,-1.261,-1.4236,0.3287,0.3383},
       {3.5398,-3.6886,0.1506,-1.2503,-1.3888,0.3272,0.3354},
       {3.5713,-3.7323,0.1609,-1.2398,-1.3539,0.325,0.332},
       {3.6028,-3.775,0.1711,-1.2293,-1.3191,0.3222,0.328},
       {3.6343,-3.8166,0.1812,-1.219,-1.2844,0.3187,0.3236},
       {3.6658,-3.857,0.1913,-1.2088,-1.2498,0.3147,0.3188},
       {3.6973,-3.8964,0.2012,-1.1988,-1.2152,0.3102,0.3136},
       {3.7288,-3.9347,0.211,-1.1889,-1.1806,0.3052,0.3081},
       {3.7603,-3.9718,0.2206,-1.1792,-1.1461,0.2999,0.3022},
       {3.7918,-4.0079,0.23,-1.1697,-1.1117,0.2942,0.296},
       {3.8233,-4.043,0.2393,-1.1603,-1.0774,0.2881,0.2895},
       {3.8548,-4.0769,0.2484,-1.1512,-1.0431,0.2817,0.2828},
       {3.8863,-4.1098,0.2572,-1.1423,-1.0088,0.2751,0.2759},
       {3.9178,-4.1415,0.2659,-1.1336,-0.9746,0.2681,0.2687},
       {3.9493,-4.1722,0.2744,-1.1251,-0.9405,0.261,0.2612},
       {3.9808,-4.2019,0.2826,-1.1169,-0.9064,0.2535,0.2536},
       {4.0123,-4.2304,0.2906,-1.1089,-0.8724,0.2459,0.2458},
       {4.0438,-4.2579,0.2983,-1.1012,-0.8384,0.2381,0.2378},
       {4.0753,-4.2843,0.3058,-1.0937,-0.8044,0.23,0.2297},
       {4.1068,-4.3096,0.3131,-1.0865,-0.7705,0.2218,0.2213},
       {4.1383,-4.3339,0.32,-1.0795,-0.7367,0.2134,0.2129},
       {4.1698,-4.3571,0.3268,-1.0728,-0.7029,0.2049,0.2043},
       {4.2013,-4.3793,0.3332,-1.0663,-0.6691,0.1962,0.1955},
       {4.2328,-4.4003,0.3394,-1.0602,-0.6354,0.1874,0.1866},
       {4.2643,-4.4203,0.3453,-1.0543,-0.6017,0.1784,0.1776},
       {4.2958,-4.4393,0.3509,-1.0487,-0.568,0.1693,0.1685},
       {4.3273,-4.4572,0.3563,-1.0434,-0.5344,0.1601,0.1592},
       {4.3588,-4.474,0.3613,-1.0384,-0.5008,0.1508,0.1499},
       {4.3903,-4.4898,0.366,-1.0337,-0.4672,0.1413,0.1405},
       {4.4218,-4.5045,0.3705,-1.0292,-0.4337,0.1318,0.1309},
       {4.4533,-4.5182,0.3747,-1.0251,-0.4001,0.1222,0.1213},
       {4.4848,-4.5308,0.3785,-1.0213,-0.3667,0.1124,0.1116},
       {4.5163,-4.5423,0.382,-1.0178,-0.3332,0.1026,0.1018},
       {4.5478,-4.5528,0.3853,-1.0146,-0.2998,0.0927,0.0919},
       {4.5793,-4.5623,0.3882,-1.0117,-0.2664,0.0827,0.0819},
       {4.6108,-4.5707,0.3908,-1.0091,-0.233,0.0726,0.0719},
       {4.6423,-4.578,0.3931,-1.0068,-0.1997,0.0624,0.0618},
       {4.6738,-4.5843,0.395,-1.0049,-0.1663,0.0522,0.0516},
       {4.7053,-4.5895,0.3967,-1.0033,-0.133,0.0419,0.0414},
       {4.7368,-4.5937,0.398,-1.002,-0.0997,0.0315,0.0311},
       {4.7683,-4.5969,0.399,-1.001,-0.0665,0.0211,0.0208},
       {4.7998,-4.599,0.3997,-1.0003,-0.0332,0.0106,0.0104},
       {4.8313,-4.6,0.4,-1.0,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
