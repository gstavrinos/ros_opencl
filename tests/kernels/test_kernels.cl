kernel void cubedPointcloud(global char* v){
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

kernel void closerLaserScanDouble(global double* v){
    unsigned int i = get_global_id(0);
    v[i] = v[i] * v[i] * 0.1f;
}

kernel void grayScale(global char* v){
    unsigned int i = get_global_id(0);
    //v[i] = v[i];
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
    }
}

kernel void frameDiff(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - v2[i]) < 20) {
        v[i] = 0;
    }
}

kernel void doubleGrayScale(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    unsigned int j = get_global_id(1);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
    }
    if(j % 3 == 0){
        v[j] = v[j];
        v[j+1] = v[j];
        v[j+2] = v[j];
    }
}