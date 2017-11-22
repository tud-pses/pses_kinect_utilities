   void kernel test2(global const int* A, global const int* B, global int* C,
                          const int n) {
       int workgroup, n_groups, job_size, start, stop;

       workgroup = get_global_id(0);
       n_groups = get_global_size(0);
       job_size = n/n_groups;

       start = workgroup*job_size;
       stop = (workgroup+1)*job_size;

       for (int i=start; i<stop; i++)
          C[i] = A[i] + B[i] + i;
   }
