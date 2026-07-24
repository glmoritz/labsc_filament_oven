#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#define OK(cond) do{ int _r=(cond); printf("  %-56s %s\n", #cond, _r?"PASS":"*** FAIL ***"); if(!_r) fails++; }while(0)
int fails=0;
int main(void){
  struct pid_state st, st2; struct pid_gains g;
  uint32_t o;

  printf("1) inactive -> always 0, integrator frozen\n");
  pid_reset(&st); g=(struct pid_gains){1000,1000,0};
  for(int i=0;i<10;i++){ o=pid_update(&st,&g,10000,2000,false); }
  OK(o==0); OK(st.yi_u==0);

  printf("2) P-only: Kp=1.0 -> 1 half-cycle per centi-degC, clamped [0,120]\n");
  pid_reset(&st); g=(struct pid_gains){1000,0,0};
  OK(pid_update(&st,&g,2100,2000,true)==100);   /* e=+100 */
  OK(pid_update(&st,&g,2500,2000,true)==120);   /* e=+500 -> clamp */
  OK(pid_update(&st,&g,1900,2000,true)==0);     /* e=-100 -> clamp */

  printf("3) I-only trapezoidal: ramps, reaches rail, holds (no windup), releases\n");
  pid_reset(&st); g=(struct pid_gains){0,1000,0}; /* Ki=1.0 */
  OK(pid_update(&st,&g,2100,2000,true)==50);    /* seeded e_prev -> inc=50 */
  OK(pid_update(&st,&g,2100,2000,true)==100);
  OK(pid_update(&st,&g,2100,2000,true)==120);   /* clamps AT the rail now */
  int64_t yi_rail=st.yi_u;
  OK(pid_update(&st,&g,2100,2000,true)==120);   /* held */
  OK(st.yi_u==yi_rail);                          /* no windup past the rail */
  pid_update(&st,&g,1900,2000,true);             /* e=-100 (avg 0 this step) */
  OK(pid_update(&st,&g,1900,2000,true)<120);     /* sustained reversal releases */

  printf("4) derivative on MEASUREMENT: a setpoint step causes NO kick\n");
  pid_reset(&st); g=(struct pid_gains){0,0,5000};
  pid_update(&st,&g,2000,2000,true);
  OK(pid_update(&st,&g,9000,2000,true)==0);     /* huge SP jump, pv flat -> 0 */

  printf("5) derivative reacts to MEASUREMENT rise, then filter decays\n");
  pid_reset(&st); g=(struct pid_gains){0,0,5000};
  pid_update(&st,&g,0,2000,true);
  pid_update(&st,&g,0,2100,true);   int64_t yd1=st.yd_u;
  OK(yd1<0);
  pid_update(&st,&g,0,2100,true);
  OK(st.yd_u>yd1);                  /* magnitude shrinking toward 0 */

  printf("6) bumpless retune: gain change adds no jump beyond normal evolution\n");
  /* run two states identically, then change Kp on one; outputs must match */
  pid_reset(&st); pid_reset(&st2);
  struct pid_gains g1={0,1000,0};
  for(int i=0;i<3;i++){ pid_update(&st,&g1,2050,2000,true); pid_update(&st2,&g1,2050,2000,true); }
  struct pid_gains g2={2000,1000,0};
  uint32_t keep   = pid_update(&st, &g1, 2050,2000,true);  /* no change */
  uint32_t change = pid_update(&st2,&g2, 2050,2000,true);  /* Kp changed, bumpless */
  OK(change==keep);

  printf("\n%s (%d failures)\n", fails? "FAILURES":"ALL PASS", fails);
  return fails?1:0;
}
