#pragma once
#include <vector>
#include <random>

struct Particle {
  float x, y;
  double w;
  Particle(float x, float y, double w);
};

struct Data {
  double h, d1, d2, d3, d4;
  Data(double h, double d1, double d2, double d3, double d4);
};

struct pData {
  double d, x, y;
  pData(double d, double x, double y);
};

struct Wall {
  double x1, y1, x2, y2;
  Wall(double x1, double y1, double x2, double y2);
};

struct Deadzone {
  double x, y, std, min, max;
  Deadzone(double x, double y, double std, double min, double max);
};

struct MCL {
  std::vector<Particle> particles;
  std::mt19937 gen;
  std::vector<Wall*> map;
  std::vector<Deadzone*> zones;
  int count;
  double inertSTD;
  pData p1, p2, p3, p4;

  MCL(int count, double x, double y, double inertSTD);

  void addWall(double x1, double y1, double x2, double y2);
  void addDz(double x, double y, double std, double min, double max);
  pData raycast(double x, double y, double h);

  void predict(double x, double y, double h);
  double guassLn(double d_exp, double d_act);
  void weigh(Data& data, double x, double y, double h);
  void resample();
};