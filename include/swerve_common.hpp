#pragma once

#include <utility>

namespace robo {
struct pidf_config {
   public:
	std::pair<double, double> output_range;	 // min, max
	double kP, kI, kD, kF, iz; // PIDF and integral zone

	pidf_config(std::pair<double, double> output_range_, double p_, double i_, double d_, double f_, double iz_);
	pidf_config(double min_output, double max_output, double p_, double i_, double d_, double f_, double iz_);
};
};	// namespace robo
