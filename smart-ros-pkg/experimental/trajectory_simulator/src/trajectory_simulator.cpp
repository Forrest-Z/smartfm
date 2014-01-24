#include <stage.hh>
#include <golfcar_ppc/golfcar_purepursuit.h>
#include <sys/stat.h>
using namespace std;

struct poseVec{
    double x, y, a;
};

class CarModel{
public:
  double wheelbase_;
  double time_step_;
  
  inline double normalize( double a )
  {
    while( a < -M_PI ) a += 2.0*M_PI;
    while( a >  M_PI ) a -= 2.0*M_PI;	 
    return a;
  }
  
  void SetSpeed(double x, double y, double a){
    
    vel_.x = x * cos(a);
    vel_.a = x * sin(a)/wheelbase_;
    
    
    double dx = vel_.x * time_step_;
    pose_.a = normalize(pose_.a + vel_.a * time_step_);
    double cosa = cos(pose_.a);
    double sina = sin(pose_.a);
    pose_.x += dx * cosa;
    pose_.y += dx * sina;
  }
  
private:
  poseVec vel_, pose_;
  
  
  
};

class TrajectorySimulator{
  Stg::World *world_;
  golfcar_purepursuit::PurePursuit *pp_;
  Stg::ModelPosition *model_position_;
  static bool s_update(Stg::World* world, TrajectorySimulator *ts);
  static void ghfunc(Stg::Model* mod, TrajectorySimulator *ts);
    
  
public:
  TrajectorySimulator(int argc, char** argv){
    char* world_file = argv[1];
    struct stat s;
    if(stat(world_file, &s) != 0){
      cout<<"World file does not exist"<<endl;
      return;
    }
    ros::init(argc, argv, "trajectory_simulator");
    Stg::Init( &argc, &argv );
    world_ = new Stg::World();
    world_->Load(world_file);
    double min_look_ahead_dist = 4.0;
    double forward_achor_pt_dist = 1.0;
    double car_length = 2.55;
    string frame_id = string("");
    pp_ = new golfcar_purepursuit::PurePursuit(frame_id, min_look_ahead_dist, forward_achor_pt_dist, car_length);
    world_->AddUpdateCallback((Stg::world_callback_t)s_update, this);
    world_->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
    
    for(int i=0; i<20; i++){
      model_position_->SetSpeed(0.2, 0.0, 0.0);
      world_->Update();
    }
      
  }
};
void TrajectorySimulator::ghfunc(Stg::Model* mod, TrajectorySimulator *ts){
  if (dynamic_cast<Stg::ModelPosition *>(mod)){
    cout<<"Position exist"<<endl;
    ts->model_position_ = dynamic_cast<Stg::ModelPosition *>(mod);
    ts->model_position_->Subscribe();
    for(int i=0; i<4; i++){
      cout<<ts->model_position_->velocity_bounds[i].min<<","<<ts->model_position_->velocity_bounds[i].max<<" ";
    }
    cout<<endl;
  }
}
bool TrajectorySimulator::s_update(Stg::World* world, TrajectorySimulator *ts){
  Stg::Pose gpose = ts->model_position_->GetGlobalPose();
  Stg::Velocity gvel = ts->model_position_->GetGlobalVelocity();
  double x = gpose.y;
  double y = -gpose.x;
  double d = (gpose.a-M_PI/2.0)/M_PI*180;
  double vx = gvel.y;
  double vy = -gvel.x;
  double vd = (gvel.a-M_PI/2.0)/M_PI*180;
  
  printf("%lf: xyz: %lf %lf %lf, vxyz: %lf %lf %lf\n", world->SimTimeNow()/1e6, x, y, d, vx, vy, vd);
  return false;
}
int main(int argc, char** argv){
  TrajectorySimulator ts(argc, argv);
  return 0;
}