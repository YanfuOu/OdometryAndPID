/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature REDBLOCK = vex::vision::signature (1, 5765, 7597, 6682, -1669, -1241, -1456, 3, 0);
vex::vision::signature BLUEBLOCK = vex::vision::signature (2, -3223, -2831, -3026, 8483, 10357, 9420, 3, 0);
vex::vision::signature YELLOWBLOCK = vex::vision::signature (3, 1269, 1565, 1417, -3565, -3355, -3460, 3, 0);
vex::vision::signature YELLOWBLOCK2 = vex::vision::signature (4, 955, 1205, 1080, -4331, -4131, -4231, 7.1, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT6, 50, REDBLOCK, BLUEBLOCK, YELLOWBLOCK, YELLOWBLOCK2, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/