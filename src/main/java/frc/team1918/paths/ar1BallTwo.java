package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class ar1BallTwo extends Path {
   private final static double[][] points = {
       {0,1.5,-0.3,0.0,0.0,0.0,-0.0},
       {0.0313,1.5,-0.3,-0.0,-0.0231,-0.0255,-0.0247},
       {0.0626,1.4993,-0.3008,-0.0008,-0.0461,-0.0511,-0.0494},
       {0.0938,1.4978,-0.3024,-0.0023,-0.0692,-0.0766,-0.0741},
       {0.1251,1.4957,-0.3048,-0.0046,-0.0923,-0.1022,-0.0989},
       {0.1564,1.4928,-0.308,-0.0077,-0.1153,-0.1277,-0.1237},
       {0.1877,1.4892,-0.312,-0.0116,-0.1384,-0.1533,-0.1484},
       {0.219,1.4848,-0.3168,-0.0162,-0.1614,-0.1788,-0.1732},
       {0.2502,1.4798,-0.3224,-0.0217,-0.1845,-0.2044,-0.1981},
       {0.2815,1.474,-0.3288,-0.0279,-0.2076,-0.2299,-0.2229},
       {0.3128,1.4675,-0.336,-0.0348,-0.2306,-0.2555,-0.2477},
       {0.3441,1.4603,-0.344,-0.0426,-0.2537,-0.281,-0.2724},
       {0.3754,1.4524,-0.3527,-0.0511,-0.2768,-0.3065,-0.2972},
       {0.4067,1.4437,-0.3623,-0.0604,-0.2998,-0.3321,-0.322},
       {0.4379,1.4343,-0.3727,-0.0705,-0.3229,-0.3576,-0.3467},
       {0.4692,1.4242,-0.3839,-0.0813,-0.3459,-0.3832,-0.3714},
       {0.5005,1.4134,-0.3959,-0.0929,-0.369,-0.4087,-0.396},
       {0.5318,1.4019,-0.4087,-0.1053,-0.392,-0.4343,-0.4207},
       {0.5631,1.3896,-0.4223,-0.1185,-0.4151,-0.4598,-0.4453},
       {0.5943,1.3766,-0.4366,-0.1324,-0.4382,-0.4854,-0.4698},
       {0.6256,1.3629,-0.4518,-0.1471,-0.4612,-0.5109,-0.4943},
       {0.6569,1.3485,-0.4678,-0.1626,-0.4843,-0.5365,-0.5188},
       {0.6882,1.3334,-0.4846,-0.1788,-0.5073,-0.562,-0.5432},
       {0.7195,1.3175,-0.5022,-0.1958,-0.5304,-0.5876,-0.5676},
       {0.7507,1.3009,-0.5206,-0.2135,-0.5534,-0.6131,-0.5919},
       {0.782,1.2836,-0.5397,-0.232,-0.5765,-0.6387,-0.6162},
       {0.8133,1.2655,-0.5597,-0.2513,-0.5995,-0.6642,-0.6404},
       {0.8446,1.2468,-0.5805,-0.2714,-0.6226,-0.6898,-0.6645},
       {0.8759,1.2273,-0.6021,-0.2921,-0.6456,-0.7154,-0.6887},
       {0.9071,1.2071,-0.6244,-0.3137,-0.6687,-0.7409,-0.7128},
       {0.9384,1.1862,-0.6476,-0.336,-0.6917,-0.7665,-0.7368},
       {0.9697,1.1646,-0.6716,-0.359,-0.7148,-0.7921,-0.7608},
       {1.001,1.1422,-0.6964,-0.3828,-0.7378,-0.8176,-0.7848},
       {1.0323,1.1191,-0.7219,-0.4074,-0.7608,-0.8432,-0.8087},
       {1.0636,1.0953,-0.7483,-0.4327,-0.7839,-0.8688,-0.8326},
       {1.0948,1.0708,-0.7755,-0.4587,-0.8069,-0.8943,-0.8565},
       {1.1261,1.0456,-0.8035,-0.4855,-0.8299,-0.9199,-0.8803},
       {1.1574,1.0196,-0.8322,-0.513,-0.853,-0.9455,-0.9042},
       {1.1887,0.9929,-0.8618,-0.5413,-0.876,-0.9711,-0.9281},
       {1.22,0.9655,-0.8922,-0.5704,-0.899,-0.9966,-0.9519},
       {1.2512,0.9374,-0.9234,-0.6001,-0.922,-1.0222,-0.9758},
       {1.2825,0.9086,-0.9554,-0.6307,-0.945,-1.0478,-0.9997},
       {1.3138,0.879,-0.9881,-0.6619,-0.968,-1.0734,-1.0237},
       {1.3451,0.8487,-1.0217,-0.694,-0.991,-1.099,-1.0477},
       {1.3764,0.8177,-1.0561,-0.7267,-1.014,-1.1246,-1.0719},
       {1.4076,0.786,-1.0913,-0.7603,-1.037,-1.1502,-1.0962},
       {1.4389,0.7536,-1.1272,-0.7945,-1.06,-1.1758,-1.1207},
       {1.4702,0.7204,-1.164,-0.8296,-1.0829,-1.2014,-1.1454},
       {1.5015,0.6865,-1.2016,-0.8654,-1.1059,-1.227,-1.1708},
       {1.5328,0.6519,-1.24,-0.9021,-1.1287,-1.2526,-1.1972},
       {1.564,0.6166,-1.2792,-0.9395,-1.1515,-1.2783,-1.2272},
       {1.5953,0.5806,-1.3192,-0.9779,-1.1269,-1.2543,-1.2539},
       {1.6266,0.5454,-1.3584,-1.0171,-1.1037,-1.2287,-1.2356},
       {1.6579,0.5108,-1.3968,-1.0558,-1.0806,-1.2032,-1.2148},
       {1.6892,0.477,-1.4345,-1.0938,-1.0575,-1.1776,-1.1931},
       {1.7205,0.444,-1.4713,-1.1311,-1.0344,-1.152,-1.1708},
       {1.7517,0.4116,-1.5073,-1.1677,-1.0114,-1.1264,-1.1482},
       {1.783,0.38,-1.5426,-1.2036,-0.9883,-1.1008,-1.1253},
       {1.8143,0.3491,-1.577,-1.2388,-0.9653,-1.0752,-1.1022},
       {1.8456,0.3189,-1.6106,-1.2733,-0.9423,-1.0496,-1.0788},
       {1.8769,0.2894,-1.6435,-1.3071,-0.9192,-1.024,-1.0553},
       {1.9081,0.2606,-1.6755,-1.3401,-0.8962,-0.9984,-1.0315},
       {1.9394,0.2326,-1.7067,-1.3723,-0.8732,-0.9728,-1.0076},
       {1.9707,0.2053,-1.7372,-1.4038,-0.8502,-0.9472,-0.9835},
       {2.002,0.1787,-1.7668,-1.4346,-0.8272,-0.9216,-0.9592},
       {2.0333,0.1528,-1.7956,-1.4646,-0.8042,-0.896,-0.9347},
       {2.0645,0.1276,-1.8236,-1.4939,-0.7812,-0.8704,-0.91},
       {2.0958,0.1032,-1.8509,-1.5223,-0.7582,-0.8447,-0.8852},
       {2.1271,0.0795,-1.8773,-1.55,-0.7352,-0.8191,-0.8601},
       {2.1584,0.0565,-1.9029,-1.5769,-0.7123,-0.7935,-0.8349},
       {2.1897,0.0342,-1.9277,-1.603,-0.6893,-0.7679,-0.8095},
       {2.2209,0.0127,-1.9518,-1.6284,-0.6663,-0.7423,-0.784},
       {2.2522,-0.0082,-1.975,-1.6529,-0.6433,-0.7167,-0.7582},
       {2.2835,-0.0283,-1.9974,-1.6766,-0.6203,-0.6911,-0.7324},
       {2.3148,-0.0477,-2.019,-1.6995,-0.5974,-0.6655,-0.7063},
       {2.3461,-0.0664,-2.0398,-1.7216,-0.5744,-0.6399,-0.6801},
       {2.3773,-0.0844,-2.0598,-1.7429,-0.5514,-0.6143,-0.6538},
       {2.4086,-0.1016,-2.0791,-1.7633,-0.5284,-0.5886,-0.6274},
       {2.4399,-0.1182,-2.0975,-1.783,-0.5055,-0.563,-0.6008},
       {2.4712,-0.134,-2.1151,-1.8017,-0.4825,-0.5374,-0.5741},
       {2.5025,-0.1491,-2.1319,-1.8197,-0.4595,-0.5118,-0.5473},
       {2.5338,-0.1634,-2.1479,-1.8368,-0.4366,-0.4862,-0.5203},
       {2.565,-0.1771,-2.1631,-1.8531,-0.4136,-0.4606,-0.4933},
       {2.5963,-0.19,-2.1775,-1.8685,-0.3906,-0.435,-0.4662},
       {2.6276,-0.2022,-2.1911,-1.8831,-0.3676,-0.4094,-0.4391},
       {2.6589,-0.2137,-2.2039,-1.8968,-0.3447,-0.3839,-0.4118},
       {2.6902,-0.2245,-2.216,-1.9097,-0.3217,-0.3583,-0.3846},
       {2.7214,-0.2346,-2.2272,-1.9218,-0.2987,-0.3327,-0.3572},
       {2.7527,-0.2439,-2.2376,-1.9329,-0.2757,-0.3071,-0.3298},
       {2.784,-0.2526,-2.2472,-1.9433,-0.2528,-0.2815,-0.3024},
       {2.8153,-0.2605,-2.256,-1.9527,-0.2298,-0.2559,-0.2749},
       {2.8466,-0.2677,-2.264,-1.9613,-0.2068,-0.2303,-0.2475},
       {2.8778,-0.2741,-2.2712,-1.9691,-0.1838,-0.2047,-0.22},
       {2.9091,-0.2799,-2.2776,-1.9759,-0.1609,-0.1791,-0.1924},
       {2.9404,-0.2849,-2.2832,-1.982,-0.1379,-0.1535,-0.1649},
       {2.9717,-0.2892,-2.288,-1.9871,-0.1149,-0.1279,-0.1374},
       {3.003,-0.2928,-2.292,-1.9914,-0.0919,-0.1024,-0.1099},
       {3.0342,-0.2957,-2.2952,-1.9948,-0.0689,-0.0768,-0.0824},
       {3.0655,-0.2978,-2.2976,-1.9974,-0.046,-0.0512,-0.0549},
       {3.0968,-0.2993,-2.2992,-1.9991,-0.023,-0.0256,-0.0274},
       {3.1281,-0.3,-2.3,-2.0,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
