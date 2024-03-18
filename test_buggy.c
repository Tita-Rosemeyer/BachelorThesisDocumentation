/* --- Generated the 19/2/2024 at 17:23 --- */
/* --- heptagon compiler, version 1.05.00 (compiled mon. jan. 8 16:15:38 CET 2024) --- */
/* --- Command line: /usr/local/bin/heptc -target c -memalloc test.ept --- */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "test.h"

void Test__assign_step(float val_x, float val_y, Test__assign_out* _out) {
  _out->v.x.z = val_y;
  _out->v.y.z = val_x;
  _out->v.x = _out->v.y;
  _out->v.y = _out->v.x;
}

