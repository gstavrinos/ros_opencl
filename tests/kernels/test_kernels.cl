__kernel void cubedPointcloud(__global uint* v){
    unsigned int i = get_global_id(0);
    if (i % 2 == 0){
        v[i] = v[i];
        //v[i] = v[i] * v[i] * v[i] * 0.15f;
    }
}