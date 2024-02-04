#include "Location.h"
#include <iostream>
#define k_speed 1
Location::Location(int id, double x, double y) : id(id) , x(x),y(y)  {
}

double Location::getX() const{
    return this->x;
}

double Location::getY() const{
    return this->y;
}

int Location::getID() const{
    return this->id;
}

bool operator==(const Location& lhs, const Location& rhs){
    if(lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY())
        return true;
    else
        return false;
}

bool compareId(const Location& lhs, const Location& rhs){
    if(lhs.getID() == rhs.getID())
        return true;
    else
        return false;
}

double getDistBetweenLocations(const Location origin, const Location dest){
    double result = sqrt(pow((origin.getX()- dest.getX()), 2) + pow((origin.getY() - dest.getY()), 2));
    return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

double Line::getDist2End(Location p)
{
    return sqrt(pow(xf-p.getX(),2) + pow(yf-p.getY(),2));
}

Line::Line(Location i, Location f)
{
    this->xi= i.getX();
    this->yi= i.getY();
    this->xf= f.getX();
    this->yf= f.getY();
    this->orientation=atan2(this->yf-this->yi,this->xf-this->xi);
    Eigen::Vector2d vec(this->xf-this->xi,this->yf-this->yi);
    this->v=vec.normalized();
    
}

Location Line::dist2Line(Location p)
{
    double ux,uy,kl,pix,piy,xr,yr;
    xr =p.getX();
    yr= p.getY();
    ux = (xf - xi)/sqrt(pow((xf - xi),2) + pow((yf - yi),2));
    uy = (yf - yi)/sqrt(pow((xf - xi),2) + pow((yf - yi),2));
    kl = (xr*uy - yr*ux - xi*uy + yi*ux)/(pow(ux,2) + pow(uy,2));
    pix = -kl*uy + xr;
    piy = kl*ux + yr;

    return Location(1,pix,piy);
}



double Line::getXi()
{
    return this->xi;
}

double Line::getYi()
{
    return this->yi;
}

double Line::getXf()
{
    return this->xf;
}

double Line::getYf()
{
    return this->yf;
}

Eigen::Vector2d Line::getVec(){
    return this->v;
}




void Location::setLocation(double x, double y){
    this->x=x;
    this->y=y;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Line> generateLines(vector<Location> path)
{   
    vector<Line> line_list;
    bool first=true;
    Location l1(0,0,0);
    for (auto i: path){
        if (first){
            l1= i;
            first=false;
        }
        else{
            Line l=Line(l1,i);
            l1=i;
            line_list.push_back(l);
        }
        
    }
    return line_list;
               
}


vector<Line> trimLines(vector<Line> path){
    vector<Line> trimmed_list;
    Location dummy_loc(1,1,0);
    Line l1=Line(dummy_loc,dummy_loc);
    Line l2=Line(dummy_loc,dummy_loc);
    int n=0;
    for (auto i: path){
        if (n==0){
            l1=i;
            l2= i;                   
        }
        else{
            if(l1.getVec()!=(i.getVec())){
                Location loc_ini =Location(0,l1.getXi(),l1.getYi());
                Location loc_fin =Location(0,i.getXi(),i.getYi());
                Line new_line = Line(loc_ini,loc_fin);
                trimmed_list.push_back(new_line);
                l1=i;
                l2=i;
            }
        }
        n++;
        if (n==path.size()){
            Location loc_ini =Location(0,l1.getXi(),l1.getYi());
            Location loc_fin =Location(0,i.getXf(),i.getYf());
            Line new_line = Line(loc_ini,loc_fin);
            trimmed_list.push_back(new_line);

        }
        
    }

    return trimmed_list;



}