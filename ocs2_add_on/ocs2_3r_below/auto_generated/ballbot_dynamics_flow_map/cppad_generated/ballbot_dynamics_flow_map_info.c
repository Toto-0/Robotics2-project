void ballbot_dynamics_flow_map_info(const char** baseName,
                                    unsigned long* m,
                                    unsigned long* n,
                                    unsigned int* indCount,
                                    unsigned int* depCount) {
   *baseName = "double  d";
   *m = 6;
   *n = 10;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}

