kernel void cubedPointcloud(global uint* v){
    unsigned int i = get_global_id(0);
    v[i] = 1;
    //v[i] = v[i] * v[i] * v[i] * 0.15f;
}

kernel void closerLaserScan(global float* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * 0.3;
}