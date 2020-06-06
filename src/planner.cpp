#include "ros/ros.h"
#include "tf/tf.h"
#include "motion_planner/generate_plan.h"
#include "motion_profile/MotionProfile.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

//---Get params from ros parameter server
void load_param( string & p, string def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( double & p, double def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}
//---------------------------------


Matrix3d QuatToMat(Vector4d Quat) {
    Matrix3d Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
}


Vector3d R2XYZ(Matrix3d R) {
    double phi=0.0, theta=0.0, psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    theta = asin(R(0,2));
    
    if(fabsf(cos(theta))>pow(10.0,-10.0))
    {
        phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
        psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
    }
    else
    {
        if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
        {
            psi = 0.0;
            phi = atan2(R(1,0), R(2,0));
            theta = M_PI/2.0;
        }
        else
        {
            psi = 0.0;
            phi = atan2(-R(1,0), R(2,0));
            theta = -M_PI/2.0;
        }
    }
    
    XYZ << phi,theta,psi;
    return XYZ;
}

class TPlanner {

    public:
        TPlanner();
        void run();
        bool trajectory( motion_planner::generate_plan::Request &req, motion_planner::generate_plan::Response & res);

    private:
        ros::NodeHandle _nh;
        ros::ServiceServer _gen_path_srv;
        double _V_max;
        double _A_max;
        double _J_max;

        double _aV_max;
        double _aA_max;
        double _aJ_max;

};

TPlanner::TPlanner() {

    _gen_path_srv = _nh.advertiseService("/generate_tarjectory", &TPlanner::trajectory, this);
    
    load_param( _V_max, 0.5, "V_max");
    load_param( _A_max, 0.25, "A_max");
    load_param( _J_max, 0.125, "J_max");


    load_param( _aV_max, 1.5, "aV_max");
    load_param( _aA_max, 1.25, "aA_max");
    load_param( _aJ_max, 1.125, "aJ_max");


}


bool TPlanner::trajectory( motion_planner::generate_plan::Request &req, motion_planner::generate_plan::Response & res ) {

    float t0_x, t0_y, t0_z, t0_yaw;
    double px, vx, ax, jx;
    double py, vy, ay, jy;
    double pz, vz, az, jz;
    double pyaw, vyaw, ayaw, jyaw;
    tf::Quaternion q;
    float count_loop;

    MotionProfile traj_x;
    MotionProfile traj_y;
    MotionProfile traj_z;
    MotionProfile traj_yaw;

    px = req.p_i.position.x;
    py = req.p_i.position.y;
    pz = req.p_i.position.z;

    Eigen::Vector3d prpy = R2XYZ ( QuatToMat( Vector4d( req.p_i.orientation.w, req.p_i.orientation.x, req.p_i.orientation.y, req.p_i.orientation.z )));
    pyaw = prpy(2);
    vx = vy = vz = vyaw = 0;
    
    double old_ref[4];
    old_ref[0] = px; 
    old_ref[1] = py; 
    old_ref[2] = pz; 
    old_ref[3] = pyaw;

    double new_ref[4];
    new_ref[0] = req.p_f.position.x; 
    new_ref[1] = req.p_f.position.y; 
    new_ref[2] = req.p_f.position.z;
    
    Eigen::Vector3d rpy = R2XYZ ( QuatToMat( Vector4d( req.p_f.orientation.w, req.p_f.orientation.x, req.p_f.orientation.y, req.p_f.orientation.z )));
    new_ref[3] = rpy(2);
    
    traj_x.setParam(old_ref[0], new_ref[0], _V_max, _A_max, _J_max);
    traj_y.setParam(old_ref[1], new_ref[1], _V_max, _A_max, _J_max);
    traj_z.setParam(old_ref[2], new_ref[2], _V_max, _A_max, _J_max);
    traj_yaw.setParam(old_ref[3], new_ref[3], _aV_max, _aA_max, _aJ_max);

    float max_time = max( float(traj_x.Duration()),  max( float( traj_y.Duration() ), max( float( traj_z.Duration() ), float( traj_yaw.Duration() ) ) ) ); 
    
    
    ros::Rate r(100);

    res.traj.header.stamp = ros::Time::now();
    res.traj.header.frame_id = "world";
    

    double t = 0.0;
    bool done = false;
    vector<float> p;
    vector<float> dp;
    vector<float> ddp;
    res.traj.points.resize(4);

    while (!done) {

        if( t <= traj_x.Duration() ) 
            traj_x.Compute(t, px, vx, ax, jx);
        if( t <= traj_y.Duration() ) 
            traj_y.Compute(t, py, vy, ay, jy);
        if( t <= traj_z.Duration() ) 
            traj_z.Compute(t, pz, vz, az, jz);
        if( t <= traj_yaw.Duration() ) 
            traj_yaw.Compute(t, pyaw, vyaw, ayaw, jyaw);

        res.traj.points[0].positions.push_back( px );
        res.traj.points[0].velocities.push_back( vx );
        res.traj.points[0].accelerations.push_back( ax );

        res.traj.points[1].positions.push_back( py );
        res.traj.points[1].velocities.push_back( vy );
        res.traj.points[1].accelerations.push_back( ay );

        res.traj.points[2].positions.push_back( pz );
        res.traj.points[2].velocities.push_back( vz );
        res.traj.points[2].accelerations.push_back( az );


        res.traj.points[3].positions.push_back( pyaw );
        res.traj.points[3].velocities.push_back( vyaw );
        res.traj.points[3].accelerations.push_back( ayaw );

        t += 1.0/100.0;
        if ( t > max_time ) done = true;

    }

    return true;

}

int main(int argc, char** argv ) {
    ros::init(argc, argv, "motion_planner");
    TPlanner tp;
    ros::spin();
    return 0;
}