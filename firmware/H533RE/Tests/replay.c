/* Replay of an input sequence through the FIRMWARE side: a Bno085 struct, heading_hold_sample()
 * and the control core, as in main.c. test_dashboard.py sends the same sequence through the
 * dashboard library and compares the states. One command for each line on stdin:
 *   k KEY NOW | t NOW | w SEQ COUNT_L COUNT_R NOW | i RAW_YAW T_MS N | s NOW | z ZERO | r | b IMU_OK NOW
 * (i: N new reports, the newest has RAW_YAW at T_MS. r: the sensor did a reset. b: start.)
 * After each command it prints the controller state (control_view) as hex floats. */
#include <stdio.h>
#include <stdlib.h>
#include "heading_hold.h"

uint32_t HAL_GetTick(void)        { return 0; }
void     HAL_Delay(uint32_t ms)   { (void)ms; }
void     bno085_update(Bno085 *b) { (void)b; }

extern const char view_names[];
void control_view(const Control *c, double *out);

int main(void)
{
  static Bno085 imu;
  HeadingHold hh;
  Control     c;
  char        line[128];
  int         fields = 1;
  for (const char *p = view_names; *p; p++) { fields += (*p == ' '); }
  double *v = malloc(sizeof(double) * (size_t)fields);

  imu.initialized = true;
  heading_hold_init(&hh, &imu);
  control_init(&c, true, 0);
  while (fgets(line, sizeof line, stdin))
  {
    unsigned a = 0, b = 0, d = 0, e = 0;
    float f = 0.0f;
    switch (line[0])
    {
      case 'b': sscanf(line + 1, "%u %u", &a, &b); control_init(&c, a != 0U, b); break;
      case 'k': sscanf(line + 1, "%u %u", &a, &b); control_key(&c, (char)a, b); break;
      case 't': sscanf(line + 1, "%u", &a); control_tick(&c, a); break;
      case 'w': sscanf(line + 1, "%u %u %u %u", &a, &b, &d, &e);
                control_wheels(&c, a, (uint16_t)b, (uint16_t)d, e); break;
      case 'i': sscanf(line + 1, "%a %u %u", &f, &a, &b);
                imu.yaw_deg[HEADING_SOURCE] = f; imu.t_ms[HEADING_SOURCE] = a; imu.count[HEADING_SOURCE] += b; break;
      case 's': { sscanf(line + 1, "%u", &a);
                  const HeadingSample s = heading_hold_sample(&hh, a);
                  control_steering(&c, &s); } break;
      case 'z': sscanf(line + 1, "%a", &f); hh.yaw_zero = f; hh.reset_count = imu.reset_count; break;
      case 'r': imu.reset_count++; break;
      default: return 2;
    }
    control_view(&c, v);
    for (int i = 0; i < fields - 1; i++) { printf("%a ", v[i]); }
    printf("\n");
  }
  return 0;
}
