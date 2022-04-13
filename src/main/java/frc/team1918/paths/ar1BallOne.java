package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class ar1BallOne extends Path {
   private final static double[][] points = {
       {0,0.0,0.0,0.0,-0.0,-0.0,0.0},
       {0.0091,-0.0,-0.0,0.0,-0.0186,0.009,0.0},
       {0.0181,-0.0002,0.0001,0.0,-0.0371,0.0181,-0.0},
       {0.0272,-0.0005,0.0002,-0.0,-0.0557,0.0273,-0.0},
       {0.0363,-0.001,0.0005,-0.0,-0.0741,0.0365,-0.0},
       {0.0453,-0.0017,0.0008,-0.0,-0.0926,0.0459,-0.0},
       {0.0544,-0.0025,0.0012,-0.0,-0.111,0.0553,-0.0},
       {0.0635,-0.0035,0.0017,-0.0,-0.1293,0.0648,-0.0},
       {0.0726,-0.0047,0.0023,-0.0,-0.1476,0.0744,-0.0},
       {0.0816,-0.006,0.003,-0.0,-0.1659,0.0841,-0.0},
       {0.0907,-0.0075,0.0038,-0.0,-0.1841,0.0939,-0.0},
       {0.0998,-0.0092,0.0046,-0.0,-0.2022,0.1038,-0.0},
       {0.1088,-0.011,0.0056,-0.0,-0.2203,0.1137,-0.0},
       {0.1179,-0.013,0.0066,-0.0,-0.2383,0.1238,-0.0},
       {0.127,-0.0152,0.0077,-0.0,-0.2563,0.134,-0.0},
       {0.136,-0.0175,0.0089,-0.0,-0.2742,0.1443,-0.0},
       {0.1451,-0.02,0.0102,-0.0,-0.2921,0.1547,-0.0},
       {0.1542,-0.0227,0.0116,-0.0,-0.3099,0.1652,-0.0},
       {0.1632,-0.0255,0.0131,-0.0,-0.3276,0.1758,-0.0},
       {0.1723,-0.0284,0.0147,-0.0,-0.3453,0.1866,-0.0},
       {0.1814,-0.0316,0.0164,-0.0,-0.3628,0.1974,-0.0},
       {0.1905,-0.0349,0.0182,-0.0,-0.3803,0.2084,-0.0},
       {0.1995,-0.0383,0.0201,-0.0,-0.3978,0.2195,-0.0},
       {0.2086,-0.0419,0.0221,-0.0,-0.4151,0.2308,-0.0},
       {0.2177,-0.0457,0.0242,-0.0,-0.4323,0.2422,-0.0},
       {0.2267,-0.0496,0.0264,-0.0,-0.4495,0.2537,-0.0},
       {0.2358,-0.0537,0.0287,-0.0,-0.4666,0.2653,-0.0},
       {0.2449,-0.0579,0.0311,-0.0,-0.4835,0.2771,-0.0},
       {0.2539,-0.0623,0.0336,-0.0,-0.5004,0.2891,-0.0},
       {0.263,-0.0668,0.0362,-0.0,-0.5171,0.3012,-0.0},
       {0.2721,-0.0715,0.039,-0.0,-0.5338,0.3135,-0.0},
       {0.2812,-0.0764,0.0418,-0.0,-0.5503,0.3259,-0.0},
       {0.2902,-0.0814,0.0448,-0.0,-0.5666,0.3385,-0.0},
       {0.2993,-0.0865,0.0478,-0.0,-0.5829,0.3512,-0.0},
       {0.3084,-0.0918,0.051,-0.0,-0.599,0.3642,-0.0},
       {0.3174,-0.0972,0.0543,-0.0,-0.615,0.3773,-0.0},
       {0.3265,-0.1028,0.0577,-0.0,-0.6308,0.3906,0.0},
       {0.3356,-0.1085,0.0613,-0.0,-0.6464,0.4041,0.0},
       {0.3446,-0.1144,0.0649,-0.0,-0.6619,0.4178,-0.0},
       {0.3537,-0.1204,0.0687,-0.0,-0.6772,0.4317,0.0},
       {0.3628,-0.1265,0.0727,-0.0,-0.6923,0.4458,-0.0},
       {0.3718,-0.1328,0.0767,-0.0,-0.7072,0.4601,-0.0},
       {0.3809,-0.1392,0.0809,-0.0,-0.7219,0.4746,-0.0},
       {0.39,-0.1458,0.0852,-0.0,-0.7364,0.4894,-0.0},
       {0.3991,-0.1524,0.0896,-0.0,-0.7506,0.5044,-0.0},
       {0.4081,-0.1593,0.0942,-0.0,-0.7646,0.5196,0.0},
       {0.4172,-0.1662,0.0989,-0.0,-0.7783,0.535,0.0},
       {0.4263,-0.1732,0.1038,-0.0,-0.7917,0.5507,0.0},
       {0.4353,-0.1804,0.1087,-0.0,-0.8049,0.5667,0.0},
       {0.4444,-0.1877,0.1139,-0.0,-0.8177,0.5829,0.0},
       {0.4535,-0.1951,0.1192,-0.0,-0.8302,0.5993,0.0},
       {0.4625,-0.2027,0.1246,-0.0,-0.8424,0.616,0.0},
       {0.4716,-0.2103,0.1302,-0.0,-0.8542,0.633,0.0},
       {0.4807,-0.2181,0.1359,-0.0,-0.8656,0.6502,0.0},
       {0.4897,-0.2259,0.1418,-0.0,-0.8765,0.6677,0.0},
       {0.4988,-0.2339,0.1479,-0.0,-0.8871,0.6855,0.0},
       {0.5079,-0.2419,0.1541,-0.0,-0.8972,0.7036,0.0},
       {0.517,-0.25,0.1605,-0.0,-0.9068,0.7218,0.0},
       {0.526,-0.2583,0.167,-0.0,-0.9159,0.7404,0.0},
       {0.5351,-0.2666,0.1737,-0.0,-0.9244,0.7592,0.0},
       {0.5442,-0.275,0.1806,-0.0,-0.9324,0.7783,0.0},
       {0.5532,-0.2834,0.1877,-0.0,-0.9398,0.7976,0.0},
       {0.5623,-0.2919,0.1949,-0.0,-0.9465,0.8171,0.0},
       {0.5714,-0.3005,0.2023,-0.0,-0.9527,0.8368,0.0},
       {0.5804,-0.3092,0.2099,-0.0,-0.9581,0.8568,0.0},
       {0.5895,-0.3179,0.2177,-0.0,-0.9628,0.8769,0.0},
       {0.5986,-0.3266,0.2257,-0.0,-0.9669,0.8972,0.0},
       {0.6077,-0.3354,0.2338,-0.0,-0.9701,0.9176,0.0},
       {0.6167,-0.3442,0.2421,-0.0,-0.9726,0.9381,0.0},
       {0.6258,-0.353,0.2506,-0.0,-0.9743,0.9587,0.0},
       {0.6349,-0.3618,0.2593,0.0,-0.9752,0.9793,0.0},
       {0.6439,-0.3707,0.2682,0.0,-0.9753,1.0,0.0},
       {0.653,-0.3795,0.2773,0.0,-0.9745,1.0206,0.0},
       {0.6621,-0.3883,0.2865,0.0,-0.9729,1.0412,0.0},
       {0.6711,-0.3972,0.296,0.0,-0.9705,1.0617,0.0},
       {0.6802,-0.406,0.3056,0.0,-0.9673,1.0822,0.0},
       {0.6893,-0.4147,0.3154,0.0,-0.9632,1.1024,0.0},
       {0.6983,-0.4235,0.3254,0.0,-0.9583,1.1225,0.0},
       {0.7074,-0.4322,0.3356,0.0,-0.9527,1.1424,0.0},
       {0.7165,-0.4408,0.3459,0.0,-0.9462,1.162,0.0},
       {0.7256,-0.4494,0.3565,0.0,-0.939,1.1814,0.0},
       {0.7346,-0.4579,0.3672,0.0,-0.9311,1.2004,0.0},
       {0.7437,-0.4663,0.3781,0.0,-0.9224,1.2192,-0.0},
       {0.7528,-0.4747,0.3891,0.0,-0.9131,1.2376,-0.0},
       {0.7618,-0.483,0.4004,0.0,-0.9031,1.2557,-0.0},
       {0.7709,-0.4912,0.4118,0.0,-0.8925,1.2735,-0.0},
       {0.78,-0.4993,0.4233,0.0,-0.8813,1.2909,-0.0},
       {0.789,-0.5073,0.435,0.0,-0.8696,1.3079,-0.0},
       {0.7981,-0.5152,0.4469,0.0,-0.8573,1.3245,-0.0},
       {0.8072,-0.5229,0.4589,0.0,-0.8446,1.3407,-0.0},
       {0.8162,-0.5306,0.471,0.0,-0.8313,1.3566,-0.0},
       {0.8253,-0.5381,0.4834,0.0,-0.8176,1.3721,-0.0},
       {0.8344,-0.5455,0.4958,0.0,-0.8035,1.3872,-0.0},
       {0.8435,-0.5528,0.5084,0.0,-0.7891,1.4019,-0.0},
       {0.8525,-0.56,0.5211,0.0,-0.7742,1.4163,-0.0},
       {0.8616,-0.567,0.5339,0.0,-0.759,1.4303,-0.0},
       {0.8707,-0.5739,0.5469,0.0,-0.7435,1.4439,-0.0},
       {0.8797,-0.5806,0.56,-0.0,-0.7277,1.4572,0.0},
       {0.8888,-0.5872,0.5732,0.0,-0.7116,1.4702,-0.0},
       {0.8979,-0.5937,0.5866,-0.0,-0.6952,1.4828,0.0},
       {0.9069,-0.6,0.6,0.0,-0.6786,1.4951,0.0},
       {0.9179,-0.6074,0.6164,0.0,-0.6583,1.5097,0.0},
       {0.9289,-0.6147,0.633,0.0,-0.6378,1.524,0.0},
       {0.9399,-0.6217,0.6497,0.0,-0.6171,1.5381,0.0},
       {0.9508,-0.6284,0.6666,0.0,-0.5962,1.5518,0.0},
       {0.9618,-0.635,0.6836,0.0,-0.5751,1.5652,0.0},
       {0.9728,-0.6413,0.7008,0.0,-0.5537,1.5782,0.0},
       {0.9838,-0.6474,0.7181,0.0,-0.5321,1.5908,0.0},
       {0.9947,-0.6532,0.7356,0.0,-0.5103,1.603,0.0},
       {1.0057,-0.6588,0.7532,0.0,-0.4882,1.6147,0.0},
       {1.0167,-0.6642,0.7709,0.0,-0.4659,1.6259,0.0},
       {1.0277,-0.6693,0.7887,0.0,-0.4433,1.6366,0.0},
       {1.0386,-0.6742,0.8067,0.0,-0.4204,1.6468,0.0},
       {1.0496,-0.6788,0.8248,0.0,-0.3973,1.6562,0.0},
       {1.0606,-0.6831,0.8429,0.0,-0.3739,1.665,0.0},
       {1.0716,-0.6872,0.8612,0.0,-0.3502,1.6731,0.0},
       {1.0825,-0.6911,0.8796,0.0,-0.3263,1.6803,0.0},
       {1.0935,-0.6947,0.898,0.0,-0.3021,1.6867,0.0},
       {1.1045,-0.698,0.9165,0.0,-0.2777,1.692,0.0},
       {1.1155,-0.701,0.9351,0.0,-0.253,1.6964,0.0},
       {1.1264,-0.7038,0.9537,0.0,-0.2282,1.6996,0.0},
       {1.1374,-0.7063,0.9724,0.0,-0.2033,1.7016,0.0},
       {1.1484,-0.7085,0.991,0.0,-0.1783,1.7023,0.0},
       {1.1594,-0.7105,1.0097,0.0,-0.1533,1.7016,0.0},
       {1.1703,-0.7122,1.0284,0.0,-0.1284,1.6994,0.0},
       {1.1813,-0.7136,1.0471,0.0,-0.1037,1.6957,0.0},
       {1.1923,-0.7147,1.0657,0.0,-0.0793,1.6903,0.0},
       {1.2033,-0.7156,1.0842,0.0,-0.0553,1.6833,0.0},
       {1.2142,-0.7162,1.1027,0.0,-0.0318,1.6747,0.0},
       {1.2252,-0.7165,1.1211,0.0,-0.009,1.6643,0.0},
       {1.2362,-0.7166,1.1393,0.0,0.0129,1.6524,0.0},
       {1.2472,-0.7165,1.1575,0.0,0.0339,1.6388,0.0},
       {1.2581,-0.7161,1.1755,0.0,0.0539,1.6238,0.0},
       {1.2691,-0.7155,1.1933,0.0,0.0728,1.6074,0.0},
       {1.2801,-0.7147,1.2109,0.0,0.0905,1.5898,-0.0},
       {1.2911,-0.7137,1.2284,0.0,0.107,1.571,-0.0},
       {1.302,-0.7126,1.2456,0.0,0.1223,1.5512,-0.0},
       {1.313,-0.7112,1.2626,0.0,0.1364,1.5305,-0.0},
       {1.324,-0.7097,1.2794,0.0,0.1493,1.5091,-0.0},
       {1.335,-0.7081,1.296,0.0,0.161,1.487,-0.0},
       {1.3459,-0.7063,1.3123,0.0,0.1716,1.4643,-0.0},
       {1.3569,-0.7044,1.3284,0.0,0.1811,1.4412,-0.0},
       {1.3679,-0.7025,1.3442,0.0,0.1897,1.4177,-0.0},
       {1.3789,-0.7004,1.3598,0.0,0.1973,1.3939,-0.0},
       {1.3898,-0.6982,1.3751,0.0,0.2039,1.3698,-0.0},
       {1.4008,-0.696,1.3901,0.0,0.2098,1.3455,-0.0},
       {1.4118,-0.6937,1.4049,0.0,0.2149,1.321,-0.0},
       {1.4228,-0.6913,1.4194,0.0,0.2192,1.2964,-0.0},
       {1.4337,-0.6889,1.4336,0.0,0.2228,1.2717,-0.0},
       {1.4447,-0.6865,1.4475,0.0,0.2258,1.2468,-0.0},
       {1.4557,-0.684,1.4612,0.0,0.2282,1.2219,-0.0},
       {1.4667,-0.6815,1.4746,0.0,0.23,1.197,-0.0},
       {1.4776,-0.6789,1.4878,0.0,0.2313,1.172,-0.0},
       {1.4886,-0.6764,1.5006,0.0,0.2321,1.147,-0.0},
       {1.4996,-0.6739,1.5132,0.0,0.2325,1.122,-0.0},
       {1.5106,-0.6713,1.5255,0.0,0.2324,1.097,-0.0},
       {1.5215,-0.6688,1.5376,0.0,0.2318,1.072,-0.0},
       {1.5325,-0.6662,1.5493,0.0,0.2309,1.047,-0.0},
       {1.5435,-0.6637,1.5608,0.0,0.2297,1.0221,-0.0},
       {1.5545,-0.6612,1.572,0.0,0.228,0.9971,-0.0},
       {1.5654,-0.6587,1.583,0.0,0.2261,0.9722,-0.0},
       {1.5764,-0.6562,1.5937,0.0,0.2239,0.9473,-0.0},
       {1.5874,-0.6537,1.6041,0.0,0.2213,0.9224,-0.0},
       {1.5984,-0.6513,1.6142,0.0,0.2185,0.8976,-0.0},
       {1.6093,-0.6489,1.624,0.0,0.2154,0.8727,-0.0},
       {1.6203,-0.6465,1.6336,0.0,0.2121,0.848,-0.0},
       {1.6313,-0.6442,1.6429,0.0,0.2086,0.8232,-0.0},
       {1.6423,-0.6419,1.6519,0.0,0.2048,0.7985,-0.0},
       {1.6532,-0.6397,1.6607,0.0,0.2008,0.7738,-0.0},
       {1.6642,-0.6375,1.6692,0.0,0.1966,0.7492,-0.0},
       {1.6752,-0.6353,1.6774,0.0,0.1922,0.7245,-0.0},
       {1.6862,-0.6332,1.6854,0.0,0.1876,0.7,-0.0},
       {1.6971,-0.6311,1.6931,0.0,0.1829,0.6754,-0.0},
       {1.7081,-0.6291,1.7005,0.0,0.1779,0.6509,-0.0},
       {1.7191,-0.6272,1.7076,0.0,0.1729,0.6264,-0.0},
       {1.7301,-0.6253,1.7145,0.0,0.1676,0.602,-0.0},
       {1.741,-0.6234,1.7211,0.0,0.1622,0.5775,-0.0},
       {1.752,-0.6217,1.7274,0.0,0.1567,0.5532,-0.0},
       {1.763,-0.6199,1.7335,0.0,0.151,0.5288,-0.0},
       {1.774,-0.6183,1.7393,0.0,0.1452,0.5045,-0.0},
       {1.7849,-0.6167,1.7448,0.0,0.1393,0.4802,-0.0},
       {1.7959,-0.6152,1.7501,0.0,0.1332,0.4559,-0.0},
       {1.8069,-0.6137,1.7551,0.0,0.1271,0.4317,-0.0},
       {1.8179,-0.6123,1.7599,0.0,0.1208,0.4075,-0.0},
       {1.8288,-0.611,1.7643,0.0,0.1144,0.3833,-0.0},
       {1.8398,-0.6097,1.7685,0.0,0.1079,0.3592,-0.0},
       {1.8508,-0.6085,1.7725,0.0,0.1013,0.3351,-0.0},
       {1.8618,-0.6074,1.7762,0.0,0.0946,0.311,-0.0},
       {1.8727,-0.6064,1.7796,0.0,0.0878,0.2869,-0.0},
       {1.8837,-0.6054,1.7827,0.0,0.081,0.2629,-0.0},
       {1.8947,-0.6045,1.7856,0.0,0.074,0.2388,-0.0},
       {1.9057,-0.6037,1.7882,0.0,0.0669,0.2149,-0.0},
       {1.9166,-0.603,1.7906,0.0,0.0598,0.1909,-0.0},
       {1.9276,-0.6023,1.7927,0.0,0.0526,0.1669,-0.0},
       {1.9386,-0.6018,1.7945,0.0,0.0453,0.143,-0.0},
       {1.9496,-0.6013,1.7961,0.0,0.0379,0.1191,-0.0},
       {1.9605,-0.6008,1.7974,0.0,0.0305,0.0953,-0.0},
       {1.9715,-0.6005,1.7984,0.0,0.023,0.0714,-0.0},
       {1.9825,-0.6003,1.7992,0.0,0.0154,0.0476,-0.0},
       {1.9935,-0.6001,1.7997,0.0,0.0077,0.0238,-0.0},
       {2.0044,-0.6,1.8,0.0,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}