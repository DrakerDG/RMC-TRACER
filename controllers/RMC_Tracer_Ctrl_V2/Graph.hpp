#pragma once

#include <vector>                                         // For handling vectors with dynamic data structures

struct Point {                                            // xy structure
    int x;
    int y;
};

class Graph {                                                
public:
  Graph(int size, double min_val, double max_val);        // Graph class

  void addValue(double v);                                // Function to add data to the vector
  std::vector<Point> getPoints(int x, int y,              // Function to process and obtain the vector data
                               int width, int height);

private:
  std::vector<double> buffer;
  int head;
  int maxSize;

  double minVal;
  double maxVal;
};
