
/** @file filter.h
 * @author Melker
 * @brief TODO: (:
*/

#ifndef FILTER_H
#define FILTER_H

const float filter_coeffs [101][8] = {
    {-0.0000000,  0.0000004, -0.0000044,  0.9999948,  0.0000129, -0.0000050,  0.0000015, -0.0000002},
    {-0.0000000,  0.0004251, -0.0043286,  0.9946655,  0.0129883, -0.0049983,  0.0014622, -0.0002141},
    {-0.0000000,  0.0008412, -0.0085242,  0.9890886,  0.0260945, -0.0099908,  0.0029177, -0.0004269},
    {-0.0000000,  0.0012482, -0.0125877,  0.9832733,  0.0393128, -0.0149737,  0.0043654, -0.0006382},
    {-0.0000000,  0.0016459, -0.0165200,  0.9772238,  0.0526372, -0.0199433,  0.0058043, -0.0008479},
    {-0.0000000,  0.0020342, -0.0203219,  0.9709444,  0.0660618, -0.0248960,  0.0072332, -0.0010557},
    {-0.0000000,  0.0024129, -0.0239943,  0.9644393,  0.0795807, -0.0298280,  0.0086510, -0.0012616},
    {-0.0000000,  0.0027819, -0.0275384,  0.9577127,  0.0931880, -0.0347357,  0.0100567, -0.0014653},
    {-0.0000000,  0.0031411, -0.0309549,  0.9507692,  0.1068777, -0.0396154,  0.0114491, -0.0016667},
    {-0.0000000,  0.0034904, -0.0342451,  0.9436129,  0.1206437, -0.0444634,  0.0128272, -0.0018657},
    {-0.0000000,  0.0038296, -0.0374099,  0.9362485,  0.1344802, -0.0492762,  0.0141900, -0.0020621},
    {-0.0000000,  0.0041587, -0.0404506,  0.9286803,  0.1483811, -0.0540502,  0.0155364, -0.0022558},
    {-0.0000000,  0.0044776, -0.0433681,  0.9209128,  0.1623405, -0.0587817,  0.0168654, -0.0024465},
    {-0.0000000,  0.0047862, -0.0461637,  0.9129507,  0.1763523, -0.0634672,  0.0181760, -0.0026343},
    {-0.0000000,  0.0050845, -0.0488386,  0.9047984,  0.1904106, -0.0681031,  0.0194672, -0.0028189},
    {-0.0000000,  0.0053724, -0.0513941,  0.8964605,  0.2045094, -0.0726860,  0.0207380, -0.0030002},
    {-0.0000000,  0.0056498, -0.0538313,  0.8879418,  0.2186427, -0.0772123,  0.0219874, -0.0031780},
    {-0.0000000,  0.0059167, -0.0561516,  0.8792468,  0.2328045, -0.0816787,  0.0232146, -0.0033523},
    {-0.0000000,  0.0061732, -0.0583563,  0.8703802,  0.2469889, -0.0860816,  0.0244186, -0.0035229},
    {-0.0000000,  0.0064190, -0.0604468,  0.8613467,  0.2611899, -0.0904176,  0.0255985, -0.0036897},
    {-0.0000000,  0.0066544, -0.0624243,  0.8521511,  0.2754015, -0.0946835,  0.0267533, -0.0038525},
    {-0.0000000,  0.0068791, -0.0642904,  0.8427982,  0.2896180, -0.0988758,  0.0278822, -0.0040113},
    {-0.0000000,  0.0070933, -0.0660464,  0.8332926,  0.3038332, -0.1029912,  0.0289844, -0.0041659},
    {-0.0000000,  0.0072970, -0.0676937,  0.8236391,  0.3180415, -0.1070266,  0.0300589, -0.0043162},
    {-0.0000000,  0.0074901, -0.0692339,  0.8138426,  0.3322368, -0.1109785,  0.0311051, -0.0044622},
    {-0.0000000,  0.0076727, -0.0706683,  0.8039078,  0.3464134, -0.1148440,  0.0321220, -0.0046036},
    {-0.0000000,  0.0078448, -0.0719986,  0.7938397,  0.3605655, -0.1186197,  0.0331089, -0.0047404},
    {-0.0000000,  0.0080064, -0.0732263,  0.7836429,  0.3746872, -0.1223026,  0.0340650, -0.0048725},
    {-0.0000000,  0.0081577, -0.0743528,  0.7733224,  0.3887727, -0.1258897,  0.0349895, -0.0049999},
    {-0.0000000,  0.0082986, -0.0753798,  0.7628829,  0.4028165, -0.1293778,  0.0358819, -0.0051223},
    {-0.0000000,  0.0084292, -0.0763088,  0.7523294,  0.4168127, -0.1327640,  0.0367413, -0.0052397},
    {-0.0000000,  0.0085495, -0.0771414,  0.7416667,  0.4307557, -0.1360454,  0.0375670, -0.0053521},
    {-0.0000000,  0.0086597, -0.0778794,  0.7308996,  0.4446399, -0.1392190,  0.0383585, -0.0054594},
    {-0.0000000,  0.0087598, -0.0785242,  0.7200329,  0.4584597, -0.1422820,  0.0391150, -0.0055614},
    {-0.0000000,  0.0088498, -0.0790775,  0.7090717,  0.4722096, -0.1452315,  0.0398360, -0.0056581},
    {-0.0000000,  0.0089300, -0.0795410,  0.6980205,  0.4858840, -0.1480650,  0.0405209, -0.0057494},
    {-0.0000000,  0.0090003, -0.0799164,  0.6868844,  0.4994775, -0.1507795,  0.0411691, -0.0058354},
    {-0.0000000,  0.0090609, -0.0802053,  0.6756682,  0.5129846, -0.1533725,  0.0417800, -0.0059158},
    {-0.0000000,  0.0091118, -0.0804095,  0.6643766,  0.5264001, -0.1558414,  0.0423531, -0.0059906},
    {-0.0000000,  0.0091532, -0.0805307,  0.6530145,  0.5397185, -0.1581836,  0.0428880, -0.0060599},
    {-0.0000000,  0.0091852, -0.0805705,  0.6415867,  0.5529346, -0.1603967,  0.0433841, -0.0061234},
    {-0.0000000,  0.0092078, -0.0805307,  0.6300980,  0.5660433, -0.1624781,  0.0438410, -0.0061813},
    {-0.0000000,  0.0092213, -0.0804131,  0.6185531,  0.5790392, -0.1644255,  0.0442583, -0.0062334},
    {-0.0000000,  0.0092258, -0.0802193,  0.6069568,  0.5919174, -0.1662366,  0.0446356, -0.0062797},
    {-0.0000000,  0.0092213, -0.0799512,  0.5953139,  0.6046727, -0.1679090,  0.0449725, -0.0063202},
    {-0.0000000,  0.0092080, -0.0796104,  0.5836290,  0.6173002, -0.1694407,  0.0452687, -0.0063548},
    {-0.0000000,  0.0091861, -0.0791989,  0.5719070,  0.6297950, -0.1708294,  0.0455238, -0.0063836},
    {-0.0000000,  0.0091557, -0.0787182,  0.5601524,  0.6421520, -0.1720730,  0.0457375, -0.0064064},
    {-0.0000000,  0.0091169, -0.0781703,  0.5483699,  0.6543667, -0.1731694,  0.0459095, -0.0064232},
    {-0.0000000,  0.0090699, -0.0775568,  0.5365641,  0.6664341, -0.1741168,  0.0460397, -0.0064342},
    {-0.0000000,  0.0090148, -0.0768796,  0.5247397,  0.6783497, -0.1749132,  0.0461278, -0.0064391},
    {-0.0000000,  0.0089518, -0.0761405,  0.5129012,  0.6901088, -0.1755568,  0.0461735, -0.0064381},
    {-0.0000000,  0.0088812, -0.0753412,  0.5010533,  0.7017068, -0.1760457,  0.0461768, -0.0064312},
    {-0.0000000,  0.0088029, -0.0744834,  0.4892003,  0.7131393, -0.1763783,  0.0461375, -0.0064182},
    {-0.0000000,  0.0087172, -0.0735691,  0.4773468,  0.7244019, -0.1765529,  0.0460555, -0.0063993},
    {-0.0000000,  0.0086243, -0.0726000,  0.4654972,  0.7354903, -0.1765679,  0.0459306, -0.0063745},
    {-0.0000000,  0.0085243, -0.0715779,  0.4536560,  0.7464001, -0.1764218,  0.0457629, -0.0063436},
    {-0.0000000,  0.0084174, -0.0705045,  0.4418275,  0.7571273, -0.1761131,  0.0455523, -0.0063069},
    {-0.0000000,  0.0083038, -0.0693816,  0.4300162,  0.7676676, -0.1756404,  0.0452987, -0.0062643},
    {-0.0000000,  0.0081836, -0.0682111,  0.4182262,  0.7780171, -0.1750025,  0.0450023, -0.0062157},
    {-0.0000000,  0.0080571, -0.0669946,  0.4064620,  0.7881718, -0.1741980,  0.0446631, -0.0061613},
    {-0.0000000,  0.0079245, -0.0657340,  0.3947276,  0.7981278, -0.1732258,  0.0442811, -0.0061011},
    {-0.0000000,  0.0077858, -0.0644310,  0.3830274,  0.8078814, -0.1720848,  0.0438564, -0.0060351},
    {-0.0000000,  0.0076413, -0.0630874,  0.3713654,  0.8174288, -0.1707738,  0.0433893, -0.0059634},
    {-0.0000000,  0.0074912, -0.0617050,  0.3597457,  0.8267664, -0.1692921,  0.0428797, -0.0058860},
    {-0.0000000,  0.0073357, -0.0602854,  0.3481725,  0.8358906, -0.1676386,  0.0423281, -0.0058029},
    {-0.0000000,  0.0071750, -0.0588305,  0.3366497,  0.8447981, -0.1658125,  0.0417345, -0.0057142},
    {-0.0000000,  0.0070093, -0.0573420,  0.3251813,  0.8534854, -0.1638131,  0.0410992, -0.0056200},
    {-0.0000000,  0.0068386, -0.0558215,  0.3137712,  0.8619493, -0.1616397,  0.0404224, -0.0055203},
    {-0.0000000,  0.0066634, -0.0542708,  0.3024233,  0.8701864, -0.1592916,  0.0397046, -0.0054153},
    {-0.0000000,  0.0064837, -0.0526916,  0.2911414,  0.8781939, -0.1567684,  0.0389460, -0.0053048},
    {-0.0000000,  0.0062997, -0.0510857,  0.2799292,  0.8859685, -0.1540696,  0.0381469, -0.0051891},
    {-0.0000000,  0.0061117, -0.0494546,  0.2687905,  0.8935074, -0.1511947,  0.0373079, -0.0050682},
    {-0.0000000,  0.0059198, -0.0478001,  0.2577290,  0.9008078, -0.1481435,  0.0364291, -0.0049422},
    {-0.0000000,  0.0057242, -0.0461238,  0.2467483,  0.9078669, -0.1449156,  0.0355113, -0.0048112},
    {-0.0000000,  0.0055252, -0.0444274,  0.2358518,  0.9146820, -0.1415111,  0.0345547, -0.0046752},
    {-0.0000000,  0.0053230, -0.0427125,  0.2250431,  0.9212506, -0.1379297,  0.0335598, -0.0045344},
    {-0.0000000,  0.0051176, -0.0409808,  0.2143257,  0.9275703, -0.1341714,  0.0325274, -0.0043888},
    {-0.0000000,  0.0049094, -0.0392338,  0.2037029,  0.9336386, -0.1302363,  0.0314577, -0.0042386},
    {-0.0000000,  0.0046985, -0.0374732,  0.1931780,  0.9394533, -0.1261245,  0.0303516, -0.0040838},
    {-0.0000000,  0.0044852, -0.0357005,  0.1827544,  0.9450122, -0.1218362,  0.0292095, -0.0039246},
    {-0.0000000,  0.0042696, -0.0339173,  0.1724351,  0.9503133, -0.1173718,  0.0280321, -0.0037610},
    {-0.0000000,  0.0040519, -0.0321251,  0.1622234,  0.9553545, -0.1127315,  0.0268200, -0.0035932},
    {-0.0000000,  0.0038324, -0.0303255,  0.1521223,  0.9601339, -0.1079158,  0.0255741, -0.0034213},
    {-0.0000000,  0.0036111, -0.0285201,  0.1421349,  0.9646499, -0.1029253,  0.0242949, -0.0032455},
    {-0.0000000,  0.0033884, -0.0267102,  0.1322641,  0.9689006, -0.0977604,  0.0229832, -0.0030657},
    {-0.0000000,  0.0031644, -0.0248975,  0.1225127,  0.9728846, -0.0924219,  0.0216399, -0.0028823},
    {-0.0000000,  0.0029393, -0.0230833,  0.1128837,  0.9766003, -0.0869105,  0.0202656, -0.0026952},
    {-0.0000000,  0.0027134, -0.0212691,  0.1033797,  0.9800463, -0.0812269,  0.0188613, -0.0025046},
    {-0.0000000,  0.0024867, -0.0194563,  0.0940036,  0.9832214, -0.0753722,  0.0174277, -0.0023108},
    {-0.0000000,  0.0022595, -0.0176464,  0.0847578,  0.9861243, -0.0693473,  0.0159658, -0.0021137},
    {-0.0000000,  0.0020319, -0.0158408,  0.0756449,  0.9887541, -0.0631531,  0.0144765, -0.0019136},
    {-0.0000000,  0.0018043, -0.0140407,  0.0666675,  0.9911097, -0.0567908,  0.0129607, -0.0017105},
    {-0.0000000,  0.0015767, -0.0122476,  0.0578280,  0.9931902, -0.0502617,  0.0114193, -0.0015048},
    {-0.0000000,  0.0013493, -0.0104628,  0.0491286,  0.9949948, -0.0435669,  0.0098533, -0.0012964},
    {-0.0000000,  0.0011223, -0.0086876,  0.0405718,  0.9965230, -0.0367078,  0.0082638, -0.0010855},
    {-0.0000000,  0.0008959, -0.0069233,  0.0321596,  0.9977741, -0.0296858,  0.0066518, -0.0008724},
    {-0.0000000,  0.0006703, -0.0051712,  0.0238944,  0.9987477, -0.0225025,  0.0050183, -0.0006571},
    {-0.0000000,  0.0004457, -0.0034324,  0.0157780,  0.9994433, -0.0151593,  0.0033645, -0.0004398},
    {-0.0000000,  0.0002222, -0.0017083,  0.0078126,  0.9998608, -0.0076579,  0.0016913, -0.0002207},
    {-0.0000000,  0.0000002, -0.0000017,  0.0000077,  1.0000000, -0.0000077,  0.0000017, -0.0000002}
};


#endif

