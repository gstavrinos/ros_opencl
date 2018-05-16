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

kernel void frameDiffFloatFloat(global float* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffFloatDouble(global float* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleChar(global double* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleInt(global double* v, global int* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleFloat(global double* v, global float* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffDoubleDouble(global double* v, global double* v2){
    unsigned int i = get_global_id(0);
    if (abs((int)(v[i] - v2[i])) < 20) {
        v[i] = 0;
    }
}

kernel void frameDiffSmallerSecondVector(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if (abs(v[i] - v2[i/4]) < 20) {
        v[i] = 0;
    }
}

kernel void doubleGrayScale(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatFloat(global float* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleFloatDouble(global float* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleChar(global double* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleInt(global double* v, global int* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleFloat(global double* v, global float* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleDoubleDouble(global double* v, global double* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[i] = v2[i];
        v2[i+1] = v2[i];
        v2[i+2] = v2[i];
    }
}

kernel void doubleGrayScaleSmallerSecondVector(global char* v, global char* v2){
    unsigned int i = get_global_id(0);
    if(i % 3 == 0){
        unsigned int ii = i / 4;
        v[i] = v[i];
        v[i+1] = v[i];
        v[i+2] = v[i];
        v2[ii] = v2[ii];
        v2[ii+1] = v2[ii];
        v2[ii+2] = v2[ii];
    }
}