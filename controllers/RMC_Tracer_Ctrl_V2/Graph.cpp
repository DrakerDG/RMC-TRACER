#include "Graph.hpp"                                            // Header of Graph

Graph::Graph(int size, double min_val, double max_val)          // Graph class
    : buffer(size, 0.0), head(0), maxSize(size),
      minVal(min_val), maxVal(max_val) {}

void Graph::addValue(double v) {                                // Function to add data to the vector
  buffer[head] = v;
  head = (head + 1) % maxSize;
}

std::vector<Point> Graph::getPoints(int x, int y,               // Function to process and obtain the vector data
                                    int width, int height) {

  std::vector<Point> points;

  for (int i = 0; i < maxSize; i++) {

    int idx = (head + i) % maxSize;
    double v = buffer[idx];

    double n = (v - minVal) / (maxVal - minVal);

    int px = x + (i * width) / maxSize;
    int py = y + height - (int)(n * height);

    points.push_back({px, py});
  }

  return points;
}
