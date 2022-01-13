void dynamics_jump_map_jacobian_sparsity(unsigned long const** row,
                                         unsigned long const** col,
                                         unsigned long* nnz) {
   static unsigned long const rows[11] = {0,1,2,3,4,5,6,7,8,9,10};
   static unsigned long const cols[11] = {1,2,3,4,5,6,7,8,9,10,11};
   *row = rows;
   *col = cols;
   *nnz = 11;
}
