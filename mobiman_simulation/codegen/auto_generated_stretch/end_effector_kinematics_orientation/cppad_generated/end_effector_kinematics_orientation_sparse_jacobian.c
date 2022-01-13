#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void end_effector_kinematics_orientation_sparse_jacobian(double const *const * in,
                                                         double*const * out,
                                                         struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[43];

   v[0] = cos(x[10]);
   v[1] = -1 * -1 * (1 - v[0]) + v[0];
   v[2] = -3.67320510363811e-06 * v[1];
   v[3] = -0.999999999993254 * v[1];
   v[1] = 2.6413435398108e-15 * v[1];
   v[4] = -3.67320510194614e-06 * v[2] + 0.999999999993254 * v[3] + -5.94478323314349e-15 * v[1];
   v[5] = cos(x[2]);
   v[6] = sin(x[2]);
   v[7] = 0 - v[6];
   v[8] = 6.08637295559855e-15 * v[5] + 2.05465555721919e-15 * v[7];
   v[9] = -1 * v[5] + -1.55778168142717e-14 * v[7];
   v[7] = -1.55778168142717e-14 * v[5] + v[7];
   v[10] = -3.67320510399553e-06 * v[8] + 3.67320510303933e-06 * v[9] + -0.999999999986508 * v[7];
   v[11] = 0.999999999993254 * v[8] + 1.34922947695824e-11 * v[9] + -3.67320510371798e-06 * v[7];
   v[7] = 3.19069361016218e-17 * v[8] + -0.999999999993254 * v[9] + -3.67320510331688e-06 * v[7];
   v[9] = v[10] + -2.91385731014123e-16 * v[11] + 4.44300759504685e-17 * v[7];
   v[8] = 2.91385731014123e-16 * v[10] + v[11] + -3.25691080516352e-16 * v[7];
   v[7] = -4.44300759504684e-17 * v[10] + 3.25691080516352e-16 * v[11] + v[7];
   v[11] = v[9] + -7.41596560408007e-17 * v[8] + 1.33876788296791e-16 * v[7];
   v[10] = 7.41596560408007e-17 * v[9] + v[8] + 2.44545873596083e-16 * v[7];
   v[7] = -1.33876788296791e-16 * v[9] + -2.44545873596083e-16 * v[8] + v[7];
   v[8] = v[11] + 3.05317390622457e-16 * v[10] + -4.23236224076729e-16 * v[7];
   v[9] = -1 * sin(x[10]);
   v[12] = -4.2595e-14 * v[0] + 0.999999999993254 * v[9];
   v[13] = -3.05317390622457e-16 * v[11] + v[10] + 2.6842513522921e-17 * v[7];
   v[14] = 2.6415e-15 * v[0] + -3.67320510363811e-06 * v[9];
   v[7] = 4.23236224076729e-16 * v[11] + -2.68425135229209e-17 * v[10] + v[7];
   v[10] = v[0] + 4.25950097024839e-14 * v[9];
   v[11] = v[8] * v[12] + v[13] * v[14] + v[7] * v[10];
   v[15] = 6.08637295559855e-15 * v[6] + 2.05465555721919e-15 * v[5];
   v[16] = -1 * v[6] + -1.55778168142717e-14 * v[5];
   v[6] = -1.55778168142717e-14 * v[6] + v[5];
   v[5] = -3.67320510399553e-06 * v[15] + 3.67320510303933e-06 * v[16] + -0.999999999986508 * v[6];
   v[17] = 0.999999999993254 * v[15] + 1.34922947695824e-11 * v[16] + -3.67320510371798e-06 * v[6];
   v[6] = 3.19069361016218e-17 * v[15] + -0.999999999993254 * v[16] + -3.67320510331688e-06 * v[6];
   v[16] = v[5] + -2.91385731014123e-16 * v[17] + 4.44300759504685e-17 * v[6];
   v[15] = 2.91385731014123e-16 * v[5] + v[17] + -3.25691080516352e-16 * v[6];
   v[6] = -4.44300759504684e-17 * v[5] + 3.25691080516352e-16 * v[17] + v[6];
   v[17] = v[16] + -7.41596560408007e-17 * v[15] + 1.33876788296791e-16 * v[6];
   v[5] = 7.41596560408007e-17 * v[16] + v[15] + 2.44545873596083e-16 * v[6];
   v[6] = -1.33876788296791e-16 * v[16] + -2.44545873596083e-16 * v[15] + v[6];
   v[15] = v[17] + 3.05317390622457e-16 * v[5] + -4.23236224076729e-16 * v[6];
   v[9] = 0 - v[9];
   v[16] = -4.2595e-14 * v[9] + 0.999999999993254 * v[0];
   v[18] = -3.05317390622457e-16 * v[17] + v[5] + 2.6842513522921e-17 * v[6];
   v[19] = 2.6415e-15 * v[9] + -3.67320510363811e-06 * v[0];
   v[6] = 4.23236224076729e-16 * v[17] + -2.68425135229209e-17 * v[5] + v[6];
   v[9] = v[9] + 4.25950097024839e-14 * v[0];
   v[0] = v[15] * v[16] + v[18] * v[19] + v[6] * v[9];
   v[5] = -3.67320510194614e-06 * v[16] + 0.999999999993254 * v[19] + -5.94478323314349e-15 * v[9];
   v[17] = v[15] * v[2] + v[18] * v[3] + v[6] * v[1];
   v[20] = v[5] - v[17];
   v[21] = v[8] * v[2] + v[13] * v[3] + v[7] * v[1];
   v[22] = -3.67320510194614e-06 * v[12] + 0.999999999993254 * v[14] + -5.94478323314349e-15 * v[10];
   v[23] = v[21] - v[22];
   if( v[11] > v[0] ) {
      v[24] = v[20];
   } else {
      v[24] = v[23];
   }
   v[25] = 0 - v[0];
   v[26] = v[15] * v[12] + v[18] * v[14] + v[6] * v[10];
   v[27] = v[8] * v[16] + v[13] * v[19] + v[7] * v[9];
   v[28] = v[26] - v[27];
   if( v[11] > v[0] ) {
      v[29] = 1 + v[11] - v[0] - v[4];
   } else {
      v[29] = 1 + v[0] - v[11] - v[4];
   }
   v[30] = 0 - v[0];
   if( v[11] < v[30] ) {
      v[31] = 1 + v[4] - v[11] - v[0];
   } else {
      v[31] = 1 + v[11] + v[0] + v[4];
   }
   if( v[4] < 0 ) {
      v[31] = v[29];
   } else {
      v[31] = v[31];
   }
   if( v[11] < v[25] ) {
      v[29] = v[28];
   } else {
      v[29] = v[31];
   }
   if( v[4] < 0 ) {
      v[29] = v[24];
   } else {
      v[29] = v[29];
   }
   v[24] = sqrt(v[31]);
   v[32] = 0.5 / v[24];
   v[33] = -1 * sin(x[2]);
   v[34] = cos(x[2]);
   v[35] = - v[34];
   v[36] = 6.08637295559855e-15 * v[33] + 2.05465555721919e-15 * v[35];
   v[37] = -1 * v[33] + -1.55778168142717e-14 * v[35];
   v[35] = -1.55778168142717e-14 * v[33] + v[35];
   v[38] = -3.67320510399553e-06 * v[36] + 3.67320510303933e-06 * v[37] + -0.999999999986508 * v[35];
   v[39] = 0.999999999993254 * v[36] + 1.34922947695824e-11 * v[37] + -3.67320510371798e-06 * v[35];
   v[35] = 3.19069361016218e-17 * v[36] + -0.999999999993254 * v[37] + -3.67320510331688e-06 * v[35];
   v[37] = v[38] + -2.91385731014123e-16 * v[39] + 4.44300759504685e-17 * v[35];
   v[36] = 2.91385731014123e-16 * v[38] + v[39] + -3.25691080516352e-16 * v[35];
   v[35] = -4.44300759504684e-17 * v[38] + 3.25691080516352e-16 * v[39] + v[35];
   v[39] = v[37] + -7.41596560408007e-17 * v[36] + 1.33876788296791e-16 * v[35];
   v[38] = 7.41596560408007e-17 * v[37] + v[36] + 2.44545873596083e-16 * v[35];
   v[35] = -1.33876788296791e-16 * v[37] + -2.44545873596083e-16 * v[36] + v[35];
   v[36] = v[39] + 3.05317390622457e-16 * v[38] + -4.23236224076729e-16 * v[35];
   v[37] = -3.05317390622457e-16 * v[39] + v[38] + 2.6842513522921e-17 * v[35];
   v[35] = 4.23236224076729e-16 * v[39] + -2.68425135229209e-17 * v[38] + v[35];
   v[38] = v[36] * v[12] + v[37] * v[14] + v[35] * v[10];
   v[39] = 6.08637295559855e-15 * v[34] + 2.05465555721919e-15 * v[33];
   v[40] = -1 * v[34] + -1.55778168142717e-14 * v[33];
   v[34] = -1.55778168142717e-14 * v[34] + v[33];
   v[33] = -3.67320510399553e-06 * v[39] + 3.67320510303933e-06 * v[40] + -0.999999999986508 * v[34];
   v[41] = 0.999999999993254 * v[39] + 1.34922947695824e-11 * v[40] + -3.67320510371798e-06 * v[34];
   v[34] = 3.19069361016218e-17 * v[39] + -0.999999999993254 * v[40] + -3.67320510331688e-06 * v[34];
   v[40] = v[33] + -2.91385731014123e-16 * v[41] + 4.44300759504685e-17 * v[34];
   v[39] = 2.91385731014123e-16 * v[33] + v[41] + -3.25691080516352e-16 * v[34];
   v[34] = -4.44300759504684e-17 * v[33] + 3.25691080516352e-16 * v[41] + v[34];
   v[41] = v[40] + -7.41596560408007e-17 * v[39] + 1.33876788296791e-16 * v[34];
   v[33] = 7.41596560408007e-17 * v[40] + v[39] + 2.44545873596083e-16 * v[34];
   v[34] = -1.33876788296791e-16 * v[40] + -2.44545873596083e-16 * v[39] + v[34];
   v[39] = v[41] + 3.05317390622457e-16 * v[33] + -4.23236224076729e-16 * v[34];
   v[40] = -3.05317390622457e-16 * v[41] + v[33] + 2.6842513522921e-17 * v[34];
   v[34] = 4.23236224076729e-16 * v[41] + -2.68425135229209e-17 * v[33] + v[34];
   v[33] = v[39] * v[16] + v[40] * v[19] + v[34] * v[9];
   if( v[11] > v[0] ) {
      v[41] = v[38] - v[33];
   } else {
      v[41] = v[33] - v[38];
   }
   if( v[11] < v[30] ) {
      v[33] = 0 - v[38] - v[33];
   } else {
      v[33] = v[38] + v[33];
   }
   if( v[4] < 0 ) {
      v[33] = v[41];
   } else {
      v[33] = v[33];
   }
   v[41] = ((- v[32]) * (v[33] / 2.) / v[24]) / v[24];
   v[38] = v[39] * v[2] + v[40] * v[3] + v[34] * v[1];
   v[42] = 0 - v[38];
   v[1] = v[36] * v[2] + v[37] * v[3] + v[35] * v[1];
   if( v[11] > v[0] ) {
      v[3] = v[42];
   } else {
      v[3] = v[1];
   }
   v[34] = v[39] * v[12] + v[40] * v[14] + v[34] * v[10];
   v[35] = v[36] * v[16] + v[37] * v[19] + v[35] * v[9];
   v[37] = v[34] - v[35];
   if( v[11] < v[25] ) {
      v[36] = v[37];
   } else {
      v[36] = v[33];
   }
   if( v[4] < 0 ) {
      v[36] = v[3];
   } else {
      v[36] = v[36];
   }
   v[36] = v[29] * v[41] + v[36] * v[32];
   v[27] = v[26] + v[27];
   if( v[11] > v[0] ) {
      v[26] = v[27];
   } else {
      v[26] = v[31];
   }
   v[3] = 0 - v[0];
   v[17] = v[5] + v[17];
   if( v[11] < v[3] ) {
      v[23] = v[17];
   } else {
      v[23] = v[23];
   }
   if( v[4] < 0 ) {
      v[23] = v[26];
   } else {
      v[23] = v[23];
   }
   v[35] = v[34] + v[35];
   if( v[11] > v[0] ) {
      v[34] = v[35];
   } else {
      v[34] = v[33];
   }
   if( v[11] < v[3] ) {
      v[26] = v[38];
   } else {
      v[26] = v[1];
   }
   if( v[4] < 0 ) {
      v[26] = v[34];
   } else {
      v[26] = v[26];
   }
   v[26] = v[23] * v[41] + v[26] * v[32];
   if( v[11] > v[0] ) {
      v[27] = v[31];
   } else {
      v[27] = v[27];
   }
   v[22] = v[21] + v[22];
   if( v[11] < v[25] ) {
      v[20] = v[22];
   } else {
      v[20] = v[20];
   }
   if( v[4] < 0 ) {
      v[20] = v[27];
   } else {
      v[20] = v[20];
   }
   if( v[11] > v[0] ) {
      v[35] = v[33];
   } else {
      v[35] = v[35];
   }
   if( v[11] < v[25] ) {
      v[42] = v[1];
   } else {
      v[42] = v[42];
   }
   if( v[4] < 0 ) {
      v[42] = v[35];
   } else {
      v[42] = v[42];
   }
   v[42] = v[20] * v[41] + v[42] * v[32];
   if( v[11] > v[0] ) {
      v[22] = v[22];
   } else {
      v[22] = v[17];
   }
   if( v[11] < v[25] ) {
      v[31] = v[31];
   } else {
      v[31] = v[28];
   }
   if( v[4] < 0 ) {
      v[31] = v[22];
   } else {
      v[31] = v[31];
   }
   if( v[11] > v[0] ) {
      v[1] = v[1];
   } else {
      v[1] = v[38];
   }
   if( v[11] < v[25] ) {
      v[37] = v[33];
   } else {
      v[37] = v[37];
   }
   if( v[4] < 0 ) {
      v[37] = v[1];
   } else {
      v[37] = v[37];
   }
   v[37] = v[31] * v[41] + v[37] * v[32];
   jac[0] = v[36] * x[11] + v[26] * x[13] - x[14] * v[42] - v[37] * x[12];
   v[41] = -1 * sin(x[10]);
   v[1] = -1 * cos(x[10]);
   v[33] = -4.2595e-14 * v[41] + 0.999999999993254 * v[1];
   v[38] = 2.6415e-15 * v[41] + -3.67320510363811e-06 * v[1];
   v[22] = v[41] + 4.25950097024839e-14 * v[1];
   v[28] = v[8] * v[33] + v[13] * v[38] + v[7] * v[22];
   v[1] = - v[1];
   v[17] = -4.2595e-14 * v[1] + 0.999999999993254 * v[41];
   v[35] = 2.6415e-15 * v[1] + -3.67320510363811e-06 * v[41];
   v[1] = v[1] + 4.25950097024839e-14 * v[41];
   v[27] = v[15] * v[17] + v[18] * v[35] + v[6] * v[1];
   v[41] = -1 * -1 * (- v[41]) + v[41];
   v[21] = -3.67320510363811e-06 * v[41];
   v[34] = -0.999999999993254 * v[41];
   v[41] = 2.6413435398108e-15 * v[41];
   v[5] = -3.67320510194614e-06 * v[21] + 0.999999999993254 * v[34] + -5.94478323314349e-15 * v[41];
   if( v[11] > v[0] ) {
      v[9] = v[28] - v[27] - v[5];
   } else {
      v[9] = v[27] - v[28] - v[5];
   }
   if( v[11] < v[30] ) {
      v[5] = v[5] - v[28] - v[27];
   } else {
      v[5] = v[28] + v[27] + v[5];
   }
   if( v[4] < 0 ) {
      v[5] = v[9];
   } else {
      v[5] = v[5];
   }
   v[24] = ((- v[32]) * (v[5] / 2.) / v[24]) / v[24];
   v[9] = -3.67320510194614e-06 * v[17] + 0.999999999993254 * v[35] + -5.94478323314349e-15 * v[1];
   v[27] = v[15] * v[21] + v[18] * v[34] + v[6] * v[41];
   v[28] = v[9] - v[27];
   v[41] = v[8] * v[21] + v[13] * v[34] + v[7] * v[41];
   v[34] = -3.67320510194614e-06 * v[33] + 0.999999999993254 * v[38] + -5.94478323314349e-15 * v[22];
   v[21] = v[41] - v[34];
   if( v[11] > v[0] ) {
      v[30] = v[28];
   } else {
      v[30] = v[21];
   }
   v[22] = v[15] * v[33] + v[18] * v[38] + v[6] * v[22];
   v[1] = v[8] * v[17] + v[13] * v[35] + v[7] * v[1];
   v[35] = v[22] - v[1];
   if( v[11] < v[25] ) {
      v[17] = v[35];
   } else {
      v[17] = v[5];
   }
   if( v[4] < 0 ) {
      v[17] = v[30];
   } else {
      v[17] = v[17];
   }
   v[17] = v[29] * v[24] + v[17] * v[32];
   v[1] = v[22] + v[1];
   if( v[11] > v[0] ) {
      v[22] = v[1];
   } else {
      v[22] = v[5];
   }
   v[27] = v[9] + v[27];
   if( v[11] < v[3] ) {
      v[21] = v[27];
   } else {
      v[21] = v[21];
   }
   if( v[4] < 0 ) {
      v[21] = v[22];
   } else {
      v[21] = v[21];
   }
   v[21] = v[23] * v[24] + v[21] * v[32];
   if( v[11] > v[0] ) {
      v[1] = v[5];
   } else {
      v[1] = v[1];
   }
   v[34] = v[41] + v[34];
   if( v[11] < v[25] ) {
      v[28] = v[34];
   } else {
      v[28] = v[28];
   }
   if( v[4] < 0 ) {
      v[28] = v[1];
   } else {
      v[28] = v[28];
   }
   v[28] = v[20] * v[24] + v[28] * v[32];
   if( v[11] > v[0] ) {
      v[34] = v[34];
   } else {
      v[34] = v[27];
   }
   if( v[11] < v[25] ) {
      v[35] = v[5];
   } else {
      v[35] = v[35];
   }
   if( v[4] < 0 ) {
      v[35] = v[34];
   } else {
      v[35] = v[35];
   }
   v[35] = v[31] * v[24] + v[35] * v[32];
   jac[1] = v[17] * x[11] + v[21] * x[13] - x[14] * v[28] - v[35] * x[12];
   jac[2] = v[36] * x[12] + v[37] * x[11] - x[14] * v[26] - v[42] * x[13];
   jac[3] = v[17] * x[12] + v[35] * x[11] - x[14] * v[21] - v[28] * x[13];
   jac[4] = v[36] * x[13] + v[42] * x[12] - x[14] * v[37] - v[26] * x[11];
   jac[5] = v[17] * x[13] + v[28] * x[12] - x[14] * v[35] - v[21] * x[11];
}

