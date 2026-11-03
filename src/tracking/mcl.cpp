#include "tracking/mcl.h"
#include  "utils.h"
#include <cmath>
#include <limits>
#include <algorithm>

Particle::Particle(float x, float y, double w): x(x), y(y), w(w) {}
Data::Data(double h, double d1, double d2, double d3, double d4): h(h), d1(d1), d2(d2), d3(d3), d4(d4) {}
pData::pData(double d, double x, double y): d(d), x(x), y(y) {}
Wall::Wall(double x1, double y1, double x2, double y2): x1(x1), y1(y1), x2(x2), y2(y2) {}
Deadzone::Deadzone(double x, double y, double std, double min, double max): x(x), y(y), std(std), min(min), max(max) {}

MCL::MCL(int count, double x, double y, double inertSTD):
  count(count), inertSTD(inertSTD),
  p1(0, 0, 0), p2(0, 0, 0), p3(0, 0, 0), p4(0, 0, 0)
{
  gen.seed(45);
  std::normal_distribution<> pos(x, 10.0);
  for (int i = 0; i < count; i++) {
    particles.push_back(Particle(pos(gen), pos(gen), 1.0 / count));
  }
}

void MCL::addWall(double x1, double y1, double x2, double y2) {
  map.push_back(new Wall(x1, y1, x2, y2));
}

void MCL::addDz(double x, double y, double std, double min, double max) {
  zones.push_back(new Deadzone(x, y, std, min, max));
}

pData MCL::raycast(double x, double y, double h) {
  double rad = utils::degToRad(h);
  double dx, dy;
  __builtin_sincos(rad, &dx, &dy);
  double dist = std::numeric_limits<double>::infinity();
  for (Wall* wall : map) {
    Wall& w = *wall;

    // wall vector
    double v2x = w.x2 - w.x1;
    double v2y = w.y2 - w.y1;

    // determinant
    double det = v2x*dy - v2y*dx;
    if (std::abs(det) < 1e-6) {continue;}

    // wall start to particle vector
    double v1x = x - w.x1;
    double v1y = y - w.y1;
    
    double t = (v1x*v2y - v1y*v2x) / det;
    double u = (v1x*dy - v1y*dx) / det;

    if (t >= 0 && u >= 0 && u <= 1) {
      if (t < dist) {dist = t;}
    }
  }
  if (dist == std::numeric_limits<double>::infinity()) {return pData(-1, 0, 0);}
  double x_o = dist*dx;
  double y_o = dist*dy;
  return pData(dist, x_o, y_o);
}

void MCL::predict(double x, double y, double h) {
  p1 = raycast(x, y, h);
  p2 = raycast(x, y, h + 90);
  p3 = raycast(x, y, h + 180);
  p4 = raycast(x, y, h - 90);
}

double MCL::guassLn(double d_exp, double d_act) {
  double diff = d_act - d_exp;
  return diff*diff / inertSTD;
}

void MCL::weigh(Data& data, double x, double y, double h) {
  predict(x, y, h);
  double min = std::numeric_limits<double>::infinity();
  for (Particle& p : particles) {
    p.w = 0;
    p.w += guassLn(p1.d, data.d1);
    p.w += guassLn(p2.d, data.d2);
    p.w += guassLn(p3.d, data.d3);
    p.w += guassLn(p4.d, data.d4);
    min = std::min(p.w, min);
  }
  double sum = 0;
  for (Particle& p : particles) {
    p.w = std::exp(min-p.w);
    sum += p.w;
  }
  for (Particle& p : particles) {
    p.w /= sum;
  }
}