#include "percentile_stats.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

PercentileStats::PercentileStats() {}

void PercentileStats::Clear() { _samples.clear(); }

void PercentileStats::AddSample(double v) { _samples.push_back(v); }

double PercentileStats::Average() const {
  if (_samples.empty()) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < _samples.size(); ++i) sum += _samples[i];
  return sum / static_cast<double>(_samples.size());
}

double PercentileStats::Max() const {
  if (_samples.empty()) return 0.0;
  return *std::max_element(_samples.begin(), _samples.end());
}

double PercentileStats::Percentile(double p) const {
  if (_samples.empty()) return 0.0;
  if (p <= 0.0) return *std::min_element(_samples.begin(), _samples.end());
  if (p >= 1.0) return *std::max_element(_samples.begin(), _samples.end());
  std::vector<double> tmp = _samples;
  const size_t idx =
      static_cast<size_t>(std::floor(p * static_cast<double>(tmp.size() - 1)));
  std::nth_element(tmp.begin(), tmp.begin() + idx, tmp.end());
  return tmp[idx];
}

