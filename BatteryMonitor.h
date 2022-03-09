#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

typedef struct {
  bool too_low;
  int scaled_val;
} processed_bat_t;

class BatteryMonitor
{
  public:
    processed_bat_t handle(int new_value);
  private:
    bool _check(int new_value);
    const int _shutdown_value = 18;
};

#endif
