typedef struct
{
  unsigned int width;
  unsigned int height;
  unsigned int n_pixels;
  float depth_scaling;
  unsigned int invalid_depth;
  float max_depth;
  float NaN;
} meta_data;

typedef struct
{
  float cx;
  float cy;
  float fx;
  float fy;
} transform;

typedef struct
{
  float x;
  float y;
  float z;
  float w;
} pixel;

#define u(i, w) (i%w)

#define v(i, w) (i/w)

#define invalid_depth(d, inv) (d==inv)

void kernel depth_to_pcl(global const unsigned short* img, global pixel* pcl,
                         const meta_data md, const transform tf)
{
  int workgroup, n_groups, job_size, start, stop;
  float z;

  workgroup = get_global_id(0);
  n_groups = get_global_size(0);
  job_size = md.n_pixels / n_groups;

  start = workgroup * job_size;
  stop = (workgroup + 1) * job_size;

  for (int i = start; i < stop; i++){

    z = img[i]*md.depth_scaling;

    if(invalid_depth(img[i], md.invalid_depth)){
      /*
      if(md.max_depth!=0.0f){
        z = md.max_depth;
      }else{
        pcl[i].x=md.NaN;
        pcl[i].y=md.NaN;
        pcl[i].z=md.NaN;
      }
      */
      pcl[i].x=md.NaN;
      pcl[i].y=md.NaN;
      pcl[i].z=md.NaN;
      pcl[i].w=1.0f;
      continue;
    }

    pcl[i].x=(u(i,md.width) - tf.cx)*z/tf.fx;
    pcl[i].y=(md.height - v(i,md.width) - tf.cy)*z/tf.fy;
    pcl[i].z=z;
    pcl[i].w=1.0f;



  }
}
