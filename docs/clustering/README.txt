struct Point {
    float x, y, z;
};



thrust::device_vector<Point> d_points = h_points;



struct is_1
{
  __host__ __device__
  bool operator()(const Point p)
  {
    dist = p.x * p.x + p.y * p.y;
                    
    if(dist > 10)
        return true;
    else 
        return false;
  }
};

// thrust::transform(thrust::device, input.begin(), input.begin()+size, out.begin(),
//                 [dist_div]__device__ (Point p){
//                     dist = p.x * p.x + p.y * p.y;
                    
//                     if(dist > dist_div)
//                         return 1;
//                     else 
//                         return 0;

//                 }


    thrust::copy_if(d_points.begin(), d_points.end(), d_is_ground.begin(), is_1());
    thrust::remove_copy_if(d_points.begin(), d_points.end(), d_is_ground.begin(), is_1());

thrust::raw_pointer_cast(d_points.data())
