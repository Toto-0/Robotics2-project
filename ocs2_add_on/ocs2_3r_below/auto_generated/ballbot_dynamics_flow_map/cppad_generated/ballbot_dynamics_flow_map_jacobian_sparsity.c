void ballbot_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                 unsigned long const** col,
                                                 unsigned long* nnz) {
   static unsigned long const rows[27] = {0,1,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5};
   static unsigned long const cols[27] = {4,5,6,2,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9,2,3,4,5,6,7,8,9};
   *row = rows;
   *col = cols;
   *nnz = 27;
}
