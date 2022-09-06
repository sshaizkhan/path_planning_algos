#include <bits/stdc++.h>

#define PLOT

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif


double MAX_ITERATIONS = 2000;
double MAX_DISTANCE = 2;
double GOAL;

// Obstacles for RRT
std::vector<double> obsXmin {3, 11};
std::vector<double> obsYmin {3, 6};
std::vector<double> obsXmax {7, 18};
std::vector<double> obsYmax {10, 17};

// ---------------------------------------------------------------

struct Node
{
  double posX;
  double poxY;
  Node* prev;
  Node* next;
};

// ----------------------------------------------------------------

class RRT {
  private:
    
};


