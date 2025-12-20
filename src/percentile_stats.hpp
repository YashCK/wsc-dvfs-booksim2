#ifndef _PERCENTILE_STATS_HPP_
#define _PERCENTILE_STATS_HPP_

#include <vector>

class PercentileStats {
public:
  PercentileStats();

  void Clear();
  void AddSample(double v);

  int Count() const { return static_cast<int>(_samples.size()); }
  double Average() const;
  double Max() const;
  double Percentile(double p) const;

private:
  std::vector<double> _samples;
};

#endif
