#include "FlameBlackbody.h"

#include <algorithm>
#include <cmath>

namespace Phantom {
namespace FlameBlackbody {

namespace {

// Piecewise Gaussian of the Wyman-Sloan-Shirley CMF fit.
double lobe(double lambda, double mu, double sigmaLow, double sigmaHigh)
{
	const double s = (lambda < mu) ? sigmaLow : sigmaHigh;
	const double t = (lambda - mu) / s;
	return std::exp(-0.5 * t * t);
}

void cmf(double lambda, double& x, double& y, double& z)
{
	x = 1.056 * lobe(lambda, 599.8, 37.9, 31.0) + 0.362 * lobe(lambda, 442.0, 16.0, 26.7)
		- 0.065 * lobe(lambda, 501.1, 20.4, 26.2);
	y = 0.821 * lobe(lambda, 568.8, 46.9, 40.5) + 0.286 * lobe(lambda, 530.9, 16.3, 31.1);
	z = 1.217 * lobe(lambda, 437.0, 11.8, 36.0) + 0.681 * lobe(lambda, 459.0, 26.0, 13.8);
}

// Planck's law up to a constant factor (it cancels in the unit-luminance normalization).
double planck(double lambdaNm, double temperature)
{
	constexpr double c2 = 1.4387769e7; // second radiation constant, nm*K
	const double l = lambdaNm * 1.0e-3; // in um, keeps lambda^-5 in range
	return 1.0 / (l * l * l * l * l * (std::exp(c2 / (lambdaNm * temperature)) - 1.0));
}

// Bradford cone-response matrix and its inverse (row-major).
constexpr double kBradford[3][3] = {
	{ 0.8951, 0.2664, -0.1614 },
	{ -0.7502, 1.7135, 0.0367 },
	{ 0.0389, -0.0685, 1.0296 } };
constexpr double kBradfordInv[3][3] = {
	{ 0.9869929, -0.1470543, 0.1599627 },
	{ 0.4323053, 0.5183603, 0.0492912 },
	{ -0.0085287, 0.0400428, 0.9684867 } };
constexpr double kD65[3] = { 0.95047, 1.0, 1.08883 };

void mul(const double m[3][3], const double v[3], double out[3])
{
	for (int r = 0; r < 3; ++r) {
		out[r] = m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2];
	}
}

glm::vec3 xyzToLinearSrgb(const double X, const double Y, const double Z)
{
	return glm::vec3(
		static_cast<float>(3.2406 * X - 1.5372 * Y - 0.4986 * Z),
		static_cast<float>(-0.9689 * X + 1.8758 * Y + 0.0415 * Z),
		static_cast<float>(0.0557 * X - 0.2040 * Y + 1.0570 * Z));
}

}

glm::vec3 xyz(float temperature)
{
	const double t = std::max(static_cast<double>(temperature), 100.0);
	double X = 0.0, Y = 0.0, Z = 0.0;
	for (double lambda = 380.0; lambda <= 780.0; lambda += 5.0) {
		double x, y, z;
		cmf(lambda, x, y, z);
		const double b = planck(lambda, t);
		X += x * b;
		Y += y * b;
		Z += z * b;
	}
	return Y > 0.0 ? glm::vec3(static_cast<float>(X / Y), 1.0f, static_cast<float>(Z / Y)) : glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec3 chromaticity(float temperature, float whiteTemperature, float degree)
{
	const glm::vec3 c = xyz(temperature);
	double v[3] = { c.x, c.y, c.z };
	if (whiteTemperature > 0.0f && degree > 0.0f) {
		// Bradford: cone responses scaled by D65 / source white, blended by D.
		const glm::vec3 w = xyz(whiteTemperature);
		const double wv[3] = { w.x, w.y, w.z };
		double rhoW[3], rhoD[3], rho[3];
		mul(kBradford, wv, rhoW);
		mul(kBradford, kD65, rhoD);
		mul(kBradford, v, rho);
		const double d = std::clamp(static_cast<double>(degree), 0.0, 1.0);
		for (int k = 0; k < 3; ++k) {
			rho[k] *= d * (rhoD[k] / rhoW[k]) + (1.0 - d);
		}
		mul(kBradfordInv, rho, v);
	}
	glm::vec3 rgb = xyzToLinearSrgb(v[0], v[1], v[2]);
	rgb = glm::max(rgb, glm::vec3(0.0f));
	const float lum = luminance(rgb);
	return lum > 0.0f ? rgb / lum : glm::vec3(1.0f, 0.0f, 0.0f) / 0.2126f;
}

float relativeRadiance(float temperature, float ambient, float reference)
{
	const auto p4 = [](float v) { const float v2 = v * v; return v2 * v2; };
	const float denom = p4(reference) - p4(ambient);
	if (denom <= 0.0f) {
		return 0.0f;
	}
	return std::max(0.0f, (p4(temperature) - p4(ambient)) / denom);
}

std::array<glm::vec4, kLutSize> makeLut(float whiteTemperature, float degree)
{
	std::array<glm::vec4, kLutSize> lut{};
	for (int i = 0; i < kLutSize; ++i) {
		const float t = kLutMinT + (kLutMaxT - kLutMinT) * static_cast<float>(i) / (kLutSize - 1);
		lut[i] = glm::vec4(chromaticity(t, whiteTemperature, degree), 1.0f);
	}
	return lut;
}

} // namespace FlameBlackbody
} // namespace Phantom
