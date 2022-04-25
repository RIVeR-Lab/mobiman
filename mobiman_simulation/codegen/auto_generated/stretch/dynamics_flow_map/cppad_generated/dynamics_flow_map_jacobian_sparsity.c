void dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                         unsigned long const** col,
                                         unsigned long* nnz) {
   static unsigned long const rows[13] = {0,0,1,1,2,3,4,5,6,7,8,9,10};
   static unsigned long const cols[13] = {3,12,3,12,13,14,15,16,17,18,19,20,21};
   *row = rows;
   *col = cols;
   *nnz = 13;
}
