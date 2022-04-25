void dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                         unsigned long const** col,
                                         unsigned long* nnz) {
   static unsigned long const rows[11] = {0,0,1,1,2,3,4,5,6,7,8};
   static unsigned long const cols[11] = {3,10,3,10,11,12,13,14,15,16,17};
   *row = rows;
   *col = cols;
   *nnz = 11;
}
