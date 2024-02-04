#include "Pid.h"
#include "Location.h"
#include "Manager.h"
#include "Graph.h"
#include <string>
#include "udpClient.h"

class Robot{
    
public:

    Robot(){};  
    Robot(double k_speed, double k_corr, double k_w, std::string udp_ip, int udp_port,double kd_v, double kd_w); 

    Location closest_id(); 

    Manager man = Manager();
    Graph<Location> graph;
    float normalizeAngle(float angle);

    udp::UdpClient *comms;

    //Controllers
    PID pid_x;
    PID pid_y;
    PID pid_theta;
    void follow(Line l);
    void rotate();
    void transformToRobRef();
    void updatePosition(double x, double y, double theta);
    

    //path planning
    void create_path(int loc_id);
    vector<Line> line_list;
    vector<Line> trimmed_list;
    vector<Location> path;

    //gains
    double k_speed=0.4;
    double k_corr=1;
    double k_w=2;
    
    //State Variables
    Location position;
    double theta;
    int line_state=0;
    int current_box=0;
    bool connected_server=false;
    bool magnet =false;
    bool switch_on=false;
    bool following= true;
    bool fetching= true;
    bool can_rotate=true;


    //Outputs
    double v_x;
    double v_y;
    double V;
    double Vn;
    double omega;

    //References
    double theta_ref=0;

   
};