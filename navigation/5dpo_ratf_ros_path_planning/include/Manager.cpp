#include "Manager.h"
#define IN_WARE 50
#define OUT_WARE 62
#define Mach_A 56
#define Mach_B 54
Box::Box(char box_type,int pos){

    this->box_type=box_type;
    this->location=IN_WARE+pos;
    this->destiny=OUT_WARE+pos;
    this->priority=pos;
    this->operation=0;
    if ( !(box_type=='R' || box_type=='G' || box_type=='B')){
        this->completed=true;
    }

}

void Manager::get_boxes(string srv_msg){
    int i=0;
    
    for(i=0;i<4;i++){
        Box new_box=Box(srv_msg[i],i);

        switch (srv_msg[i]) {
        case 'R': new_box.priority = 0; break;
        case 'G': new_box.priority = 1; break;
        case 'B': new_box.priority = 4; break;
        }

        this->box_list.push_back(new_box);
        this->occupied_locations.push_back(new_box.location);
    }
    
}

bool Manager::calculate_destinies(int box_id){

    Box b = this->box_list[box_id];
    int i = 0;
    

    if ((b.box_type=='B')){
        while(i<4){
            if(!is_occupied(OUT_WARE+i)){
                box_list[box_id].destiny=OUT_WARE+i;
                this->occupied_locations.push_back(OUT_WARE+i);
                return true;
            }
            i++;
        }
    }

    
    if ((b.box_type=='G' && b.operation==1) || (b.box_type=='R' && b.operation==2) ){
        while(i<4){
            if(!is_occupied(OUT_WARE+3-i)){
                box_list[box_id].destiny=OUT_WARE+3-i;
                this->occupied_locations.push_back(OUT_WARE+3-i);
                return true;
            }
            i++;
        }
    }

    else if ((b.box_type=='G' || b.box_type=='R') && b.operation==0)
    {
        while(i<2){
            if ((b.box_type=='G' and mach_b))
            {
                if(!is_occupied(Mach_B+i*4))
                {
                    box_list[box_id].destiny=Mach_B+i*4;
                    this->occupied_locations.push_back(Mach_B+i*4);
                    return true;
                }
            }
            else
            {
                if(!is_occupied(Mach_A+i*4))
                {
                    box_list[box_id].destiny=Mach_A+i*4;
                    this->occupied_locations.push_back(Mach_A+i*4);
                    return true;
                }
            }
            i++;
        }
    }

    else if ((b.box_type=='R') && b.operation==1){
        while(i<2){
            if(!is_occupied(Mach_B+i*4)){
                box_list[box_id].destiny=Mach_B+i*4;
                this->occupied_locations.push_back(Mach_B+i*4);
                return true;
            }
            i++;
        }
    }

    return false;

}

bool Manager::is_occupied(int loc)
{
    for (auto i: this->occupied_locations){
        if (i==loc)
            return true;       
        }
    return false;
}

bool Manager::free_location(int loc)
{
    std::cout << "SIZE 2  " << occupied_locations.size() <<std::endl;
    occupied_locations.erase(remove(occupied_locations.begin(), occupied_locations.end(), loc), occupied_locations.end());
    std::cout << "SIZE  2" << occupied_locations.size() << "  "<< loc << std::endl;
    return true;
}

bool Manager::complete_operation(){

    this->box_list[current_box].operation+=1;
    this->free_location(this->box_list[current_box].location);
    if ((box_list[current_box].box_type== 'R' || box_list[current_box].box_type=='G') && box_list[current_box].operation>1){
        this->free_location(this->box_list[current_box].location-1);
    }
    if (box_list[current_box].box_type== 'B')
    {   

        this->box_list[current_box].location=this->box_list[current_box].destiny;
        occupied_locations.push_back(this->box_list[current_box].location);
        box_list[current_box].completed=true;

        switch (box_list[current_box].priority) {
        case 4:  box_list[current_box].priority = 10; break;
        default: box_list[current_box].priority = 10; break;
        }
    }
    else if (box_list[current_box].box_type== 'G')
    {   
        //this->free_location(this->box_list[current_box].destiny);
        this->box_list[current_box].location=this->box_list[current_box].destiny+1;  
        if (box_list[current_box].operation==2)
        {
            box_list[current_box].completed=true;
            this->occupied_locations.push_back(this->box_list[current_box].destiny);
        }

        switch (box_list[current_box].priority) {
        case 1:  box_list[current_box].priority = 2;  break;
        case 2:  box_list[current_box].priority = 10; break;
        default: box_list[current_box].priority = 10; break;
        }
        
    }
    else if (box_list[current_box].box_type== 'R')
    {   
        //this->free_location(this->box_list[current_box].destiny);
        this->box_list[current_box].location=this->box_list[current_box].destiny+1; 

        if (box_list[current_box].operation==3)
        {
            box_list[current_box].completed=true;
            this->occupied_locations.push_back(this->box_list[current_box].destiny);
        }

        switch (box_list[current_box].priority) {
        case 0:  box_list[current_box].priority = 3;  break;
        case 3:  box_list[current_box].priority = 5;  break;
        case 5:  box_list[current_box].priority = 10; break;
        default: box_list[current_box].priority = 10; break;
        }
        
    }
    else
    {
        return false;
    }


    // Update current box by finding the higher priority box
    current_box = findPriorityBox();

    // Check if all completed
    if (checkAllBoxesCompleted()) {
        current_box = 4;
    }

    // if( box_list[current_box].completed)
    // {
    //     current_box++;
    // }
    return true;
}

size_t Manager::findPriorityBox() {
    size_t box_idx = 0;
    for (size_t i = 1; i < 4; i++) {
        if (box_list[i].priority < box_list[box_idx].priority) {
            box_idx = i;
        }
    }
    return box_idx;
}

bool Manager::checkAllBoxesCompleted() {
    for (size_t i = 0; i < 4; i++) {
        if (box_list[i].priority < 10) {
            return false;
        }
    }
    return true;
}
