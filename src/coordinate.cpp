/**
* 根据一个点的经纬度/方位角/距离得到另一个点的经纬度位置
* @param destination_lat 纬度
*@param destination_lng 经度
* @param bearing 方位角
* @param dist 距离
* @param return caculate_latlng 经纬度
*/
/*
target_latlng_t according_distance_caculate_latlng(float dist,float bearing,double destination_lat,double destination_lng)
{
    // printf("destination_lat = %d ,destination_lng = %d \n",destination_lat,destination_lng);
    target_latlng_t caculate_latlng;

    double a = 6378137;
    double b = 6356752.3142;
    double f = 1/298.257223563;

    double s = dist;
    double alpha1 = rad(bearing);
    double sinAlpha1 = sin(alpha1);
    double cosAlpha1 = cos(alpha1);

    double tanU1 = (1-f) * tan(rad(destination_lat));
    double cosU1 = 1 / sqrt((1 + tanU1*tanU1)), sinU1 = tanU1*cosU1;
    double sigma1 = atan2(tanU1, cosAlpha1);
    double sinAlpha = cosU1 * sinAlpha1;
    double cosSqAlpha = 1 - sinAlpha*sinAlpha;
    double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
    double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));

    double sigma = s / (b*A), sigmaP = 2*PI;
    double sinSigma ,cosSigma,cos2SigmaM;
    while (fabs(sigma-sigmaP) > 1e-12) {
    cos2SigmaM = cos(2*sigma1 + sigma);
    sinSigma = sin(sigma);
    cosSigma = cos(sigma);
    double deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
    B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
    sigmaP = sigma;
    sigma = s / (b*A) + deltaSigma;
    }

    double tmp = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
    double lat2 = atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1,
    (1-f)*sqrt(sinAlpha*sinAlpha + tmp*tmp));
    double lambda = atan2(sinSigma*sinAlpha1, cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
    double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
    double L = lambda - (1-C) * f * sinAlpha * (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));

    double revAz = atan2(sinAlpha, -tmp); // final bearing
    // float moveLongitude = (destination_lng*0.0000001)+deg(L);
    // float moveLatitude = deg(destination_lat*0.0000001);

    caculate_latlng.target_lat = deg(lat2);
    caculate_latlng.target_lng = destination_lng+deg(L);
    // printf("deg = %f,ddeg = %f \n", deg(lat2),deg(L));

    return caculate_latlng;
    // return {long:lon1+u.deg(L),lati:u.deg(lat2)};

}
*/