void end_effector_kinematics_orientation_jacobian_sparsity(unsigned long const** row,
                                                           unsigned long const** col,
                                                           unsigned long* nnz) {
   static unsigned long const rows[6] = {0,0,1,1,2,2};
   static unsigned long const cols[6] = {2,10,2,10,2,10};
   *row = rows;
   *col = cols;
   *nnz = 6;
}
