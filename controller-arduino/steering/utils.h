#ifndef UTILS_H_
#define UTILS_H_

double fmap(double value, double in_min, double in_max, double out_min, double out_max) {
  value = constrain(value, in_min, in_max);
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif