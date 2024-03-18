/* --- Generated the 17/3/2024 at 21:12 --- */
/* --- heptagon compiler, version 1.05.00 (compiled mon. mar. 4 17:8:15 CET 2024) --- */
/* --- Command line: /usr/local/bin/heptc -O -v -v -target c justbug.ept --- */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "justbug.h"

void Justbug__help_step(float x, float y, Justbug__help_out* _out) {
  _out->x_updated.t = x;
  _out->y_updated.t = y;
}

void Justbug__assign_step(Justbug__vec v_in, float val_x, float val_y,
                          Justbug__assign_out* _out) {
  Justbug__help_out Justbug__help_out_st;
  
  Justbug__wrap y;
  Justbug__help_step(val_x, val_y, &Justbug__help_out_st);
  _out->v.y = Justbug__help_out_st.x_updated;
  y = Justbug__help_out_st.y_updated;
  _out->v = v_in;
  _out->v.x = _out->v.y;
  _out->v.y = y;;
}

