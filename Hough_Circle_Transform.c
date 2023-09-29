#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#pragma warning(disable : 4996)

#define NAME "Test_img_CV_HW4_396x400.yuv"

//#define channel 1  // 1-channel(Grayscale) RAW file
#define width 396
#define height 400

#define gauss 5             // Size of Gaussian Filter Mask

// Thresholds
#define UPPER 380            // Upper threshold
#define LOWER 190            // Lower threshold
#define MIN 25               // Minimun radius
#define MAX 150              // Maximum radius
#define HOUGH_TH 90          // Hough transform threshold

const int SIZE = width*height;  // Total Size
const float RAD2DEG = 57.29578f;  // 180/π

unsigned char blur[height][width];
unsigned short norm[height][width];
float gradient[height][width];

void Track(int h, int w);   // Use DFS for Edge Tracking
void Canny(unsigned char input[][width], unsigned char canny[][width]);  // Canny edge detection
void Hough(unsigned char input[][width], unsigned char canny[][width]);  // Circle detection using Hough transform

clock_t start, end;

int main()
{
  unsigned char input[height][width], canny[height][width];
  FILE* origin, *output;
  clock_t s, e;
  
  // Read img
  origin = fopen(NAME,"rb");
  if(!origin)
  {
    puts("FILE OPEN ERROR\n");
    return -1;
  }
  fread(input,sizeof(char),SIZE,origin);
  fclose(origin);

  s = clock();
  Canny(input, canny);  // Canny edge detection
  e = clock();
  printf("Canny edge detection execution time : %ldms\n\n", e-s);
  
  // Return edge detection img
  char name[50];
  sprintf(name, "gray_canny_%dx%d_%d_%d_%d.raw", width, height, LOWER, UPPER, HOUGH_TH);
  output = fopen(name,"wb");
  fwrite(canny,sizeof(char),SIZE,output);
  fclose(output);

  s = clock();
  Hough(input, canny);  // Hough transform
  e = clock();
  printf("Hough transform execution time : %ldms\n", e-s);

  sprintf(name, "Hough_transform_%dx%d_%d_%d_%d.raw", width, height, LOWER, UPPER, HOUGH_TH);
  output = fopen(name,"wb");
  fwrite(input,sizeof(char),SIZE,output);
  fclose(output);
  return 0;
}

void Canny(unsigned char input[][width], unsigned char canny[][width])
{
  // direction : 0~3 -> 0˚~135˚
  unsigned char direction[height][width];
  int sum, h, w, i, j, X, Y;
  float setha, s;
  
  // Gaussian filter mask : sum = 256 (for unsigned short)
  unsigned short gaussian[gauss][gauss] = {{1,4,6,4,1}, {4,16,24,16,4}, {6,24,36,24,6}, {4,16,24,16,4}, {1,4,6,4,1}};

  // Apply Gaussian filter
  start = clock();
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
    {
      // Convolution
      for(sum = i = 0; i < gauss; i++)
        for(j = 0; j < gauss; j++)
          sum += gaussian[i][j]*input[h+i<2 ? 0 : h+i-2>=height ? height-1 : h+i-2][w+j<2 ? 0 : w+j-2>=width ? width-1 : w+j-2];
      blur[h][w] = sum+128>>8;  // blur[h][w]=sum/256
    }
  end = clock();
  printf("Gaussian filter execution time : %ldms\n", end-start);

  short Sobel_X[3][3] = {{-1,0,1}, {-2,0,2}, {-1,0,1}}, Sobel_Y[3][3] = {{1,2,1}, {0,0,0}, {-1,-2,-1}};

  // Apply sobel operator for calculate norm & direction of gradient
  start = clock();
  for(h = 0; h < height; h++)
  {
    for(w = 0; w < width; w++)
    {
      // Convolution
      for(X = Y = i = 0; i < 3; i++)
        for(j = 0; j < 3; j++)
        {
          X += Sobel_X[i][j]*blur[h+i<1 ? 0 : h+i-1>=height ? height-1 : h+i-1][w+j<1 ? 0 : w+j-1>=width ? width-1 : w+j-1];
          Y += Sobel_Y[i][j]*blur[h+i<1 ? 0 : h+i-1>=height ? height-1 : h+i-1][w+j<1 ? 0 : w+j-1>=width ? width-1 : w+j-1];
        }

      s = Y>0 ? 1.0 : -1.0;  // Sign value for 0˚~180˚ arctan
      gradient[h][w] = X ? -1.0f*Y/X : INFINITY;  // Save gradient for Hough transform
      setha = X ? atan2f(s*Y,s*X)*RAD2DEG : 90.0f;
      direction[h][w] = setha>=22.5 && setha<67.5 ? 1 : setha>=67.5 && setha<112.5 ? 2 : setha>=112.5 && setha<157.5 ? 3 : 0;
      norm[h][w] = abs(X) + abs(Y);  // 1D distance
      //norm[h][w] = sqrtf(1.0f*(X*X+Y*Y));  // 2D distance
    }
  }
  end = clock();
  printf("Sobel operator execution time : %ldms\n", end-start);
  
  // NMS(Non-Maximum Suppression)
  start = clock();
  memset(blur, 0, sizeof(blur));  // Reuse for check local maximum
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
    {
      switch(direction[h][w])
      {
        case 0:    // Direction : →
        {
          X = w ? norm[h][w-1] : 0;
          Y = w<width-1 ? norm[h][w+1] : 0;
          break;
        }
        case 1:    // Direction : ↗
        {
          X = h && w<width-1 ? norm[h-1][w+1] : 0;
          Y = h<height-1 && w ? norm[h+1][w-1] : 0;
          break;
        }
        case 2:    // Direction : ↑
        {
          X = h ? norm[h-1][w] : 0;
          Y = h<height-1 ? norm[h+1][w] : 0;
          break;
        }
        case 3:    // Direction : ↖
        {
          X = h && w ? norm[h-1][w-1] : 0;
          Y = h<height-1 ? norm[h+1][w+1] : 0;
          break;
        }
        default:
        {
          puts("INVALID DIRECTION OCCUR\n");
          exit(-1);
        }
      }
      
      blur[h][w] = norm[h][w]>=X && norm[h][w]>=Y;  // Local maximum truth value
    }
  
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
      norm[h][w] = blur[h][w] ? norm[h][w] : 0;  // Keep as border only when local maximum
  end = clock();
  printf("Non-maximum suppression execution time : %ldms\n", end-start);

  // Hysteresis edge tracking (Two-level threshold)
  start = clock();
  memset(blur,0,sizeof(blur));  // Reuse for check visited
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
      if(norm[h][w] >= UPPER && !blur[h][w])  // Strong edge
        Track(h,w);
  
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
      canny[h][w] = blur[h][w] ? 255 : 0;  // Binaryization
  end = clock();
  printf("Hysteresis edge tracking(Two-level thresholding) execution time : %ldms\n", end-start);

  return;
}

void Track(int h, int w)    // Use DFS for Edge Tracking
{
  blur[h][w] = 1;
  
  if(h && !blur[h-1][w] && norm[h-1][w] >= LOWER)  // DOWN
    Track(h-1,w);
  if(h<height-1 && !blur[h+1][w] && norm[h+1][w] >= LOWER)  // UP
    Track(h+1,w);
  if(w && !blur[h][w-1] && norm[h][w-1] >= LOWER)  // LEFT
    Track(h,w-1);
  if(w<width-1 && !blur[h][w+1] && norm[h][w+1] >= LOWER)  // RIGHT
    Track(h,w+1);
  if(h && w && !blur[h-1][w-1] && norm[h-1][w-1] >= LOWER)  // LEFT-UP
    Track(h-1,w-1);
  if(h && w<width-1 && !blur[h-1][w+1] && norm[h-1][w+1] >= LOWER)  // RIGHT-UP
    Track(h-1,w+1);
  if(h<height-1 && w && !blur[h+1][w-1] && norm[h+1][w-1] >= LOWER)  // LEFT-DOWN
    Track(h+1,w-1);
  if(h<height-1 && w<width-1 && !blur[h+1][w+1] && norm[h+1][w+1] >= LOWER)  // RIGHT-DOWN
    Track(h+1,w+1);
  
  return;
}

void Hough(unsigned char input[][width], unsigned char canny[][width])
{
  int h, w, i, j, sgn;
  unsigned char hough[height][width] = {0,};

  start = clock();
  // Line Hough transform
  for(h = 0; h < height; h++)
    for(w = 0; w < width; w++)
      if(canny[h][w] == 255)   // Edge pixel
      {
        if(gradient[h][w] == INFINITY)  // Gradient = inf
          for(i = 0; i < height; i++)
            hough[i][w] += hough[i][w] < 255;  // Voting
        else                            // Gradient ≠ inf
          for(j = 0, sgn = gradient[h][w] < 0; j < width; j++)
          {
            int start = gradient[h][w]*(j-w+sgn)+h, end = gradient[h][w]*(j-w+(1-sgn))+h;  // Consider both when the slope is +/-
            if((sgn==1 && end<0) || (sgn==0 && start>=height))  // Invalid region only
              break;
            for(i = start; i <= end; i++)  // Normal line : y = grad(h,w)*(x-w)+h
              if(i>=0 && i<height)
                hough[i][j] += hough[i][j] < 255;  // Voting
          }
      }
  end = clock();
  printf("Line Hough transform execution time : %ldms\n", end-start);
  
  // Find all center pixels
  start = clock();
  memset(blur,0,sizeof(blur));  // Reuse for check visited
  int center[21][2] = {0,}, count = 0;
  for(i = 0; i < height; i++)
    for(j = 0; j < width; j++)
    {
      if(hough[i][j] >= HOUGH_TH && !blur[i][j])    // Probably center of a circle
      {
        int max = 0;
        // Search 15*20 rectangular area
        for(h = i; h < i+15; h++)
          for(w = j-10; w < j+11; w++)
            if(h>=0 && h<height && w>=0 && w<width && !blur[h][w])  // Valid pixel
            {
              if(max < hough[h][w])  // Highest voted pixel
              {
                // Update
                center[count][0] = w;
                center[count][1] = h;
                max = hough[h][w];
              }
              blur[h][w] = 1;     // Visited
            }
        
        // Don't count if already chosen the center for nearby pixel
        for(h = 0; h < count; h++)
        {
          int a = center[h][0]-center[count][0], b = center[h][1]-center[count][1];
          h = a*a+b*b < 450 ? count : h;  // 450 = (15^2)*2 -> (15,15) nearby area
        }
        count += count < 20 && h == count;          // Circle count up
      }
      blur[i][j] = 1;     // Visited
    }
  end = clock();
  printf("Find center pixels execution time : %ldms\n", end-start);

  // Find optimal radius
  start = clock();
  for(i = 0; i < count; i++)
  {
    int max = 0, R = 0;
    // Choose the radius (maximum votes)
    for(int r = MIN; r <= MAX; r++)
    {
      int c = 0, start = center[i][1]>r ? center[i][1]-r : 0, end = center[i][1]+r<height ? center[i][1]+r : height-1;
      for(h = start; h <= end; h++)  // Fixed the y-axis
      {
        int a = center[i][0], b = center[i][1];
        float x1 = a+sqrtf(r*r-(h-b)*(h-b)), x2 = a-sqrtf(r*r-(h-b)*(h-b));  // x1 : right arc, x2 : left arc
        int x1_l = floorf(x1), x1_h = ceilf(x1), x2_l = floorf(x2), x2_h = ceilf(x2);
        
        c += x1_l>=0 && x1_l<width && canny[h][x1_l]>0;
        c += x1_h>=0 && x1_h<width && canny[h][x1_h]>0;
        c += x2_l>=0 && x2_l<width && canny[h][x2_l]>0;
        c += x2_h>=0 && x2_h<width && canny[h][x2_h]>0;
      }
      
      if(c > max)  // Update
      {
        R = r;
        max = c;
      }
    }

    // Final result print
    printf("No.%d circle : center (a,b) = (%u,%u), radius = %d\n", i+1, center[i][0], center[i][1], R);
    input[center[i][1]][center[i][0]] = 255;
    for(h = center[i][1]-R; h <= center[i][1]+R; h++)  // Paint circle on input img
    {
      if(h < 0)
        continue;
      if(h >= height)
        break;
      
      int a = center[i][0], b = center[i][1];
      float x1 = a+sqrtf(R*R-(h-b)*(h-b)), x2 = a-sqrtf(R*R-(h-b)*(h-b));  // x1 : right arc, x2 : left arc
      int x1_l = floorf(x1), x1_h = ceilf(x1), x2_l = floorf(x2), x2_h = ceilf(x2);

      // Paint circle
      if(x1_l>=0 && x1_l<width)
        input[h][x1_l] = 255;
      if(x1_h>=0 && x1_h<width)
        input[h][x1_h] = 255;
      if(x2_l>=0 && x2_l<width)
        input[h][x2_l] = 255;
      if(x2_h>=0 && x2_h<width)
        input[h][x2_h] = 255;
    }
  }
  end = clock();
  printf("Find optimal radius execution time : %ldms\n", end-start);
  return;
}