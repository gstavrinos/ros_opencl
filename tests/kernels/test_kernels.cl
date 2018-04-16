kernel void cubedPointcloud(global uint* v){
    unsigned int i = get_global_id(0);
    //v[i] = v[i];
    v[i] = v[i] * v[i] * v[i] * 0.15f;
    // The calculation above does not make any sense
    // and does not produce any visual results!
    // You will have to play with byte reading to produce
    // meaningful results.
}

kernel void closerLaserScan(global float* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * 0.1f;
}

kernel void grayScale(global uint* v){
    unsigned int i = get_global_id(0);
     v[i] = v[i];
    // if(i % 3 == 0){
    //     v[i] = (v[i] + v[i+1] + v[i+2]) / 3;
    //     v[i+1] = (v[i] + v[i+1] + v[i+2]) / 3;
    //     v[i+2] = (v[i] + v[i+1] + v[i+2]) / 3;
    // }
}