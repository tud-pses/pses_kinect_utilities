   void kernel test1(global const int* A, global const int* B, global int* C,
                          const int n) {
       int ID, Nthreads, ratio, start, stop;

       ID = get_global_id(0);
       Nthreads = get_global_size(0);

       ratio = (n / Nthreads);
       start = ratio * ID;
       stop  = ratio * (ID + 1);

       for (int i=start; i<stop; i++)
          C[i] = A[i] + B[i] + i;
   }
