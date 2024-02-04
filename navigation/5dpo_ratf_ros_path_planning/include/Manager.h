#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>



using namespace std;
class Box{
    
public:
    Box(char box_type,int pos);
    char box_type;
    int location;
    int destiny;
    int priority;
    int operation;
    bool completed;
    

};

class Manager{
    
public:
    
    vector<Box> box_list;
    void get_boxes(string boxes);
    bool calculate_destinies(int box_id);
    vector<int> occupied_locations;
    bool is_occupied(int loc);
    bool free_location(int loc);
    bool complete_operation();
    int current_box=0;
    bool mach_b=false;
    
    size_t findPriorityBox();
    bool checkAllBoxesCompleted();
};



