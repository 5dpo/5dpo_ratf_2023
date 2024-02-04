#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
using namespace std;

class Location{
    
    
  public:
    Location(){};
    Location(int id, double x, double y);
    void setLocation(double x, double y);
    double getX() const;
    double getY() const;
    int getID() const;
    int id;
    double x, y;
    
    
};


class Line{
    double xi, yi , xf , yf;
    Eigen::Vector2d v;

  public:
    Line(Location i, Location f);
    Line(){};
    Location dist2Line(Location p);
    double getXi();
    double getYi();
    double getXf();
    double getYf();
    double getDist2End(Location p);
    Eigen::Vector2d getVec();
    double orientation;
    
};

bool operator==(const Location& lhs, const Location& rhs);
bool compareId(const Location& lhs, const Location& rhs);
double getDistBetweenLocations(const Location origin, const Location dest);

vector<Line> generateLines(vector<Location> path);
vector<Line> trimLines(vector<Line> path);



