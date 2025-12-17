#pragma once

#include <stddef.h>

template <size_t N>
struct MovingAverage
{
  float  buffer[N];
  size_t index;
  size_t count;
  float  sum;

  MovingAverage() : index(0), count(0), sum(0.0f)
  {
    for (size_t i = 0; i < N; ++i)
    {
      buffer[i] = 0.0f;
    }
  }

  float add(float value)
  {
    if (count < N)
    {
      buffer[index] = value;
      sum += value;
      ++count;
      index = (index + 1) % N;
    }
    else
    {
      sum -= buffer[index];
      buffer[index] = value;
      sum += value;
      index = (index + 1) % N;
    }
    return sum / count;
  }
};
