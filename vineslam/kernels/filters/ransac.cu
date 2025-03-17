#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <curand_kernel.h>

namespace vineslam
{
__global__ void generate_random_numbers(int* numbers, int size, int max_n)
{
  int i = threadIdx.x + blockIdx.x * blockDim.x;

  if (i < size)
  {
    curandState state;
    curand_init(clock64(), i, 0, &state);
    numbers[i] = ceilf(curand_uniform(&state) * max_n);
  }
}

struct sum_functor
{
  int R;
  int C;
  int* arr;

  sum_functor(int _R, int _C, int* _arr) : R(_R), C(_C), arr(_arr){};

  __host__ __device__ int operator()(int myC)
  {
    int sum = 0;
    for (int i = 0; i < R; i++)
      sum += arr[i * C + myC];
    return sum;
  }
};

// Global function means it will be executed on the device (GPU)
__global__ void loop(float* xx, float* yy, float* zz, int* rand_numbers, int* inliers_map, int size, int n_iters,
                     float dist_threshold)
{
  int iteration_idx = threadIdx.x + blockIdx.x * blockDim.x;
  int point_idx = threadIdx.y + blockIdx.y * blockDim.y;

  if (iteration_idx < n_iters)
  {
    if (point_idx < size)
    {
      // Get random indexes
      int r1 = rand_numbers[iteration_idx * 3];
      int r2 = rand_numbers[iteration_idx * 3 + 1];
      int r3 = rand_numbers[iteration_idx * 3 + 2];

      /*
      // Check if we have an inlier
      atomicAdd(
          &inliers_map[iteration_idx],
          (fabs(((yy[r2] - yy[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (yy[r3] - yy[r1])) * xx[point_idx] +
                (-((xx[r2] - xx[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (xx[r3] - xx[r1]))) * yy[point_idx] +
                ((xx[r2] - xx[r1]) * (yy[r3] - yy[r1]) - (yy[r2] - yy[r1]) * (xx[r3] - xx[r1])) * zz[point_idx] +
                (-(((yy[r2] - yy[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (yy[r3] - yy[r1])) * xx[r1] +
                   (-((xx[r2] - xx[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (xx[r3] - xx[r1]))) * yy[r1] +
                   ((xx[r2] - xx[r1]) * (yy[r3] - yy[r1]) - (yy[r2] - yy[r1]) * (xx[r3] - xx[r1])) * zz[r1])))) /
                  sqrt(((yy[r2] - yy[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (yy[r3] - yy[r1])) *
                           ((yy[r2] - yy[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (yy[r3] - yy[r1])) +
                       ((xx[r2] - xx[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (xx[r3] - xx[r1])) *
                           ((xx[r2] - xx[r1]) * (zz[r3] - zz[r1]) - (zz[r2] - zz[r1]) * (xx[r3] - xx[r1])) +
                       ((xx[r2] - xx[r1]) * (yy[r3] - yy[r1]) - (yy[r2] - yy[r1]) * (xx[r3] - xx[r1])) *
                           ((xx[r2] - xx[r1]) * (yy[r3] - yy[r1]) - (yy[r2] - yy[r1]) * (xx[r3] - xx[r1]))) <
              dist_threshold);
      */

      // The above code abbreviates the following one
      float x21 = xx[r2] - xx[r1];
      float y21 = yy[r2] - yy[r1];
      float z21 = zz[r2] - zz[r1];
      float x31 = xx[r3] - xx[r1];
      float y31 = yy[r3] - yy[r1];
      float z31 = zz[r3] - zz[r1];
      float l_a = +(y21 * z31 - z21 * y31);
      float l_b = -(x21 * z31 - z21 * x31);
      float l_c = +(x21 * y31 - y21 * x31);
      float l_d = -(l_a * xx[r1] + l_b * yy[r1] + l_c * zz[r1]);

      // Check if we have an inlier
      float norm = sqrt(l_a * l_a + l_b * l_b + l_c * l_c);
      float pt_xx = xx[point_idx];
      float pt_yy = yy[point_idx];
      float pt_zz = zz[point_idx];

      atomicAdd(&inliers_map[iteration_idx],
                (fabs(l_a * pt_xx + l_b * pt_yy + l_c * pt_zz + l_d) / norm) < dist_threshold);
    }
  }
}

int singleIteration(float* xx, float* yy, float* zz, float* out_xx, float* out_yy, float* out_zz, int* rand_numbers,
                     int pos, int size, float dist_threshold)
{
  // Get points
  float xx1 = xx[rand_numbers[pos * 3 + 0]];
  float yy1 = yy[rand_numbers[pos * 3 + 0]];
  float zz1 = zz[rand_numbers[pos * 3 + 0]];
  float xx2 = xx[rand_numbers[pos * 3 + 1]];
  float yy2 = yy[rand_numbers[pos * 3 + 1]];
  float zz2 = zz[rand_numbers[pos * 3 + 1]];
  float xx3 = xx[rand_numbers[pos * 3 + 2]];
  float yy3 = yy[rand_numbers[pos * 3 + 2]];
  float zz3 = zz[rand_numbers[pos * 3 + 2]];

  // Extract the plane hessian coefficients
  float x21 = xx2 - xx1;
  float y21 = yy2 - yy1;
  float z21 = zz2 - zz1;
  float x31 = xx3 - xx1;
  float y31 = yy3 - yy1;
  float z31 = zz3 - zz1;
  float l_a = +(y21 * z31 - z21 * y31);
  float l_b = -(x21 * z31 - z21 * x31);
  float l_c = +(x21 * y31 - y21 * x31);
  float l_d = -(l_a * xx1 + l_b * yy1 + l_c * zz1);
  float norm = sqrt(l_a * l_a + l_b * l_b + l_c * l_c);

  // Check if we have an inlier
  int j = 0;
  for (int i = 0; i < size; i++)
  {
    float pt_xx = xx[i];
    float pt_yy = yy[i];
    float pt_zz = zz[i];
    if (fabs(l_a * pt_xx + l_b * pt_yy + l_c * pt_zz + l_d) / norm < dist_threshold)
    {
      out_xx[j] = xx[i];
      out_yy[j] = yy[i];
      out_zz[j] = zz[i];
      j++;
    }
  }

  return j;
}

void ransac(float* xx, float* yy, float* zz, float* out_xx, float* out_yy, float* out_zz, int& size, int n_iters,
            float dist_threshold)
{
  if (size == 0)
  {
    return;
  }

  // Error code to check return values for CUDA calls
  cudaError_t err = cudaSuccess;

  int n_threads = n_iters;
  int ssize = size * sizeof(float);

  // Allocate memory for the device vector of points
  float *d_xx = NULL, *d_yy = NULL, *d_zz = NULL;

  err = cudaMalloc((void**)&d_xx, ssize);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector xx (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  err = cudaMalloc((void**)&d_yy, ssize);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector yy (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  err = cudaMalloc((void**)&d_zz, ssize);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device vector zz (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Inliers array
  int *d_inliers_map, *inliers_map;
  inliers_map = (int*)malloc(n_iters * sizeof(int));
  for (int i = 0; i < n_iters; i++)
  {
    inliers_map[i] = 0;
  }
  err = cudaMalloc((void**)&d_inliers_map, n_iters * sizeof(int));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device inliers_map vector (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Random numbers array
  int *d_rand_numbers, *rand_numbers;
  int n_rands = n_iters * 3;
  rand_numbers = (int*)malloc(n_rands * sizeof(int));
  err = cudaMalloc((void**)&d_rand_numbers, n_rands * sizeof(int));
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to allocate device rand_numbers vector (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Copy all the data to the device
  err = cudaMemcpy(d_xx, xx, ssize, cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector xx from host to device (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  err = cudaMemcpy(d_yy, yy, ssize, cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector yy from host to device (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_zz, zz, ssize, cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector zz from host to device (error code %s)!\n", cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  err = cudaMemcpy(d_inliers_map, inliers_map, n_iters * sizeof(int), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector inliers_map from host to device (error code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Generate all the random numbers required for ransac
  generate_random_numbers<<<(n_rands + n_threads - 1) / n_threads, n_threads>>>(d_rand_numbers, n_rands, size);
  err = cudaMemcpy(rand_numbers, d_rand_numbers, n_rands * sizeof(int), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector rand_numbers from device to host (error code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }

  // Call ransac
  int block_dim_per_component = 8;
  dim3 block_dim(block_dim_per_component, block_dim_per_component);
  dim3 grid_dim((n_iters + block_dim.x - 1) / block_dim.x, (size + block_dim.y - 1) / block_dim.y);
  loop<<<grid_dim, block_dim>>>(d_xx, d_yy, d_zz, d_rand_numbers, d_inliers_map, size, n_iters, dist_threshold);
  cudaDeviceSynchronize();

  // Get iteration with most inliers
  err = cudaMemcpy(inliers_map, d_inliers_map, n_iters * sizeof(int), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
    fprintf(stderr, "Failed to copy vector inliers_map from host to device (error code %s)!\n",
            cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
  int max = 0;
  int pos = -1;
  for (int i = 0; i < n_iters; i++)
  {
    if (inliers_map[i] > max)
    {
      max = inliers_map[i];
      pos = i;
    }
  }

  // Compute the inliers given the iteration number
  if (pos > 0)
  {
    int res = singleIteration(xx, yy, zz, out_xx, out_yy, out_zz, rand_numbers, pos, size, dist_threshold);
    if (res != max)
    {
      size = 0;
#if VERBOSE == 1
      std::cout << "ERROR (Ransac GPU) - singleIteration does not match kernel result (" << max << "," << res << ") ... \n" << std::flush;
#endif
    }
    else
    {
      size = max;
    }
  }
  else
  {
    size = 0;
  }

  cudaFree(d_rand_numbers);
  cudaFree(d_inliers_map);
  cudaFree(d_xx);
  cudaFree(d_yy);
  cudaFree(d_zz);
  free(rand_numbers);
  free(inliers_map);
}
}  // namespace vineslam