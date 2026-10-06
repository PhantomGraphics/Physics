// Mass-preserving mixture of a uniform sphere and compact SPH poly6.
// These expressions match FlameSmokeProfile.h. The mean volume density is 1,
// so the PBVR Poisson count stays unchanged; only particle positions change.
float smokeRadialCdf(float r,float profile) {
    float q=r*r;
    float poly6=(105.0+q*(-189.0+q*(135.0-35.0*q)))/16.0;
    return r*q*mix(1.0,poly6,profile);
}
float smokeSampleRadius(float sampleValue,float profile) {
    if (profile<=0.0) return pow(sampleValue,1.0/3.0);
    float lo=0.0,hi=1.0;
    // Fixed work and no rejected particles: inverse CDF of the radial mass.
    for (int i=0;i<18;++i) {
        float r=0.5*(lo+hi);
        if (smokeRadialCdf(r,profile)<sampleValue) lo=r; else hi=r;
    }
    return 0.5*(lo+hi);
}
float smokeProfilePrimitive(float t,float a) {
    float q=t*t;
    return t*(a*a*a+q*(-a*a+q*(0.6*a-q/7.0)));
}
float smokeProfileColumn(float impactSquared,float begin,float end,float profile) {
    float a=max(0.0,1.0-impactSquared);
    float limit=sqrt(a);
    begin=clamp(begin,-limit,limit); end=clamp(end,-limit,limit);
    float poly6=(105.0/16.0)*max(0.0,smokeProfilePrimitive(end,a)-smokeProfilePrimitive(begin,a));
    return 0.5*mix(max(0.0,end-begin),poly6,profile);
}
float smokeProfileProjectedColumn(float impactSquared,float profile) {
    float chord=sqrt(max(0.0,1.0-impactSquared));
    float poly6=3.0*chord*pow(max(0.0,1.0-impactSquared),3.0);
    return mix(chord,poly6,profile);
}
