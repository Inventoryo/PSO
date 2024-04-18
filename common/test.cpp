float getSqrt(float x){

    if(x<0)
        return 0;
    float res = 0;
    while((x - res * res) >= 0.001){
        res = (x / res + res) / 2;
    }

    return res;
}