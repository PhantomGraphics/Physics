#pragma once
#include <algorithm>
#include <cmath>

namespace Phantom {
// Unit support radius. A uniform sphere has mean density 1; normalized poly6
// is (105/16)*(1-r^2)^3 and has the SAME volume integral. Tau remains the
// old uniform sphere's central optical-depth scale, not the new centre depth.
class FlameSmokeProfile {
public:
    static double density(double r, double profile) {
        if (r<0 || r>=1) return 0;
        const double a=std::clamp(profile,0.0,1.0), q=1-r*r;
        return (1-a)+a*(105.0/16.0)*q*q*q;
    }
    static double radialCdf(double r, double profile) {
        r=std::clamp(r,0.0,1.0);
        const double q=r*r;
        const double poly6=(105+q*(-189+q*(135-35*q)))/16;
        return r*q*((1-std::clamp(profile,0.0,1.0))+std::clamp(profile,0.0,1.0)*poly6);
    }
    static double sampleRadius(double uniform, double profile) {
        const double u=std::clamp(uniform,0.0,1.0);
        if (u==0 || u==1) return u;
        if (profile<=0) return std::cbrt(u);
        double lo=0,hi=1;
        for (int i=0;i<24;++i) {
            const double r=0.5*(lo+hi);
            if (radialCdf(r,profile)<u) lo=r; else hi=r;
        }
        return 0.5*(lo+hi);
    }
    // Optical-depth multiplier for a segment along a unit-radius sphere.
    // impactSquared is perpendicular distance^2; begin/end are signed axial
    // coordinates relative to the puff centre, measured in support radii.
    static double column(double impactSquared, double begin, double end, double profile) {
        const double a=std::max(0.0,1-impactSquared);
        if (a<=0) return 0;
        const double limit=std::sqrt(a);
        begin=std::clamp(begin,-limit,limit); end=std::clamp(end,-limit,limit);
        if (end<=begin) return 0;
        const auto primitive=[a](double t) {
            const double q=t*t;
            return t*(a*a*a+q*(-a*a+q*(0.6*a-q/7)));
        };
        const double smooth=(105.0/16.0)*std::max(0.0,primitive(end)-primitive(begin));
        const double blend=std::clamp(profile,0.0,1.0);
        return 0.5*((1-blend)*(end-begin)+blend*smooth);
    }
};
}
