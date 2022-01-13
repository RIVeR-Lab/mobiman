void self_collision_distance_intermediate_jacobian_sparsity(unsigned long const** row,
                                                            unsigned long const** col,
                                                            unsigned long* nnz) {
   static unsigned long const rows[31] = {0,0,0,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5,5};
   static unsigned long const cols[31] = {0,1,2,0,1,2,3,4,0,1,2,5,6,0,1,2,3,5,0,1,2,3,5,6,0,1,2,3,5,6,7};
   *row = rows;
   *col = cols;
   *nnz = 31;
}
