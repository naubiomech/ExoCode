#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H


class BatteryMonitor
{
  public:
    bool handle(int& new_value);
  private:
    bool _check(int new_value);
    const int _new_zero = 20;
    const float _shutdown_value = 0.18;
};

#endif
