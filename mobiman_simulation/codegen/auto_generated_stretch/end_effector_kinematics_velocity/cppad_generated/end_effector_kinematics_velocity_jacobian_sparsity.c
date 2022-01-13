void end_effector_kinematics_velocity_jacobian_sparsity(unsigned long const** row,
                                                        unsigned long const** col,
                                                        unsigned long* nnz) {
   static unsigned long const rows[42] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[42] = {2,5,6,7,8,9,10,11,12,15,16,17,18,19,2,5,6,7,8,9,10,11,12,15,16,17,18,19,2,5,6,7,8,9,10,11,12,15,16,17,18,19};
   *row = rows;
   *col = cols;
   *nnz = 42;
}
