#include <planning/moving_grasp.hpp>
#include <traj_opt/poly_traj_utils.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace planning::moving_grasp;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void near(double a, double b, const char* message, double eps=1e-8) {
  require(std::abs(a-b)<eps,message);
}
int main() {
  try {
    Sequence sequence;
    require(!sequence.ready(10,true,0.15),"Gate needs a stable dwell");
    require(!sequence.ready(10.1,false,0.15),"Broken gate resets dwell");
    require(!sequence.ready(10.2,true,0.15),"Gate must restart");
    require(sequence.ready(10.36,true,0.15),"Continuous dwell allows close");
    sequence.close(11.0);
    sequence.close(11.5); // Repeat command must not restart the timer.
    require(!sequence.update(11.849,0.85,2),"No early lift");
    require(sequence.phase()==Phase::CLOSING,"Closing still follows at 0.849 s");
    sequence.update(11.851,0.85,2);
    require(sequence.phase()==Phase::LIFTING,"Lift after first close + 0.85 s");
    require(!sequence.update(13.8,0.85,2),"Lift gets its full duration");
    require(sequence.update(13.852,0.85,2),"Lift can finish");
    sequence.brake(true); sequence.stopped();
    require(sequence.phase()==Phase::ABORTED && sequence.closeSent(),"Abort preserves closed command");
    sequence.update(99,0.85,2);
    require(sequence.phase()==Phase::ABORTED,"Elapsed time cannot cancel abort");
    sequence.reset();
    require(!sequence.closeSent() && sequence.phase()==Phase::APPROACH,"New trigger resets sequence");

    require(fresh(10,10.1,0.1,0.2),"Fresh snapshot");
    require(!fresh(10,10.3,0.01,0.2),"Old measurement with fresh receipt rejected");
    require(!fresh(10,10.01,0.3,0.2),"Stopped input rejected");
    require(!fresh(10.2,10,0.0,0.2),"Future stamp rejected");
    require(!fresh(0,0,0,0.2),"Zero stamp rejected");

    VelocityFilter velocity;
    const Eigen::Vector3d seed(0.3,-0.2,0.1), step(0.6,0.1,-0.2);
    require(velocity.update(seed,10,20,3,0.2),"First velocity accepted");
    require((velocity.value()-seed).norm()<1e-12,"First moving sample must not ramp from zero");
    const double tau=1.0/(2*kPi*3.0);
    velocity.update(step,10+tau,20.001,3,0.2);
    require((velocity.value()-(seed+(1-std::exp(-1.0))*(step-seed))).norm()<1e-12,
            "3 Hz uses 2*pi and measurement dt even when callbacks are batched");
    const Eigen::Vector3d last=velocity.value();
    require(!velocity.update(seed,10+tau,20.01,3,0.2),"Duplicate velocity cannot refresh filter");
    require(!velocity.update(seed,9,20.01,3,0.2),"Rollback velocity cannot refresh filter");
    require(!velocity.update({NAN,0,0},10.1,20.01,3,0.2),"Invalid velocity rejected");
    require(!velocity.update(seed,10.1,20.01,-1,0.2),"Negative cutoff rejected");
    require(!velocity.update(seed,10.1,20.01,std::numeric_limits<double>::infinity(),0.2),
            "Infinite cutoff rejected");
    require((velocity.value()-last).norm()<1e-12,"Rejected samples preserve filter state");
    velocity.update(step,10.1,20.3,3,0.2);
    require((velocity.value()-step).norm()<1e-12,"Receipt gap discards stale history");
    velocity.update(seed,11,20.31,3,0.2);
    require((velocity.value()-seed).norm()<1e-12,"Measurement gap discards stale history");
    velocity.update(step,11.01,20.32,0,0.2);
    require((velocity.value()-step).norm()<1e-12,"Zero Hz bypasses filtering");
    velocity.reset();
    velocity.update(seed,1,30,3,0.2);
    require((velocity.value()-seed).norm()<1e-12,"Reset seeds from next measurement");
    double input_energy=0.0, output_energy=0.0;
    for(int i=1;i<=3000;++i) {
      const double t=i*0.001, input=std::sin(2*kPi*3*t);
      velocity.update({input,0,0},1+t,30+t,3,0.2);
      if(i>1000) { input_energy+=input*input; output_energy+=velocity.value().x()*velocity.value().x(); }
    }
    near(std::sqrt(output_energy/input_energy),1/std::sqrt(2.0),
         "Configured 3 Hz attenuates a 3 Hz sine by approximately 3 dB",0.002);

    State start,end;
    start.p={1,2,0.8}; start.v={0.2,-0.1,0}; start.a={0.02,0.03,-0.01};
    end.p={1.15,1.95,0.95}; end.v={0.18,0.05,0.1}; end.a={0.01,0.02,0.03};
    Piece segment(0.8,quintic(start,end,0.8));
    require((segment.getPos(0)-start.p).norm()<1e-9,"Position continuity");
    require((segment.getVel(0)-start.v).norm()<1e-9,"Velocity continuity");
    require((segment.getAcc(0)-start.a).norm()<1e-9,"Acceleration continuity");
    require((segment.getPos(0.8)-end.p).norm()<1e-8,"Position endpoint");
    require((segment.getVel(0.8)-end.v).norm()<1e-8,"Velocity endpoint");
    require((segment.getAcc(0.8)-end.a).norm()<1e-8,"Acceleration endpoint");
    require(lift(0,2,0.2).norm()<1e-9,"Lift starts without a step");
    require((lift(2,2,0.2)-Eigen::Vector3d(0.2,0,0)).norm()<1e-9,"Lift stops smoothly");
    near(lift(1,2,0.2)[0],0.1,"Half lift height");

    const Eigen::Vector3d grasp(2,3,0.9);
    near(initialApproachYaw({1,3,0.2}, grasp, -0.7),0,"Initial direction points from aircraft to target");
    near(initialApproachYaw({2,3,0}, grasp, -0.7),-0.7,"Coincident horizontal position uses yaw fallback");
    require((pregraspPoint(grasp,0,0.25)-Eigen::Vector3d(1.75,3,0.9)).norm()<1e-12,
            "Pregrasp is 0.25 m behind direction at grasp height");
    require((pregraspPoint(grasp,kPi/2,0.25)-Eigen::Vector3d(2,2.75,0.9)).norm()<1e-12,
            "Turning direction rotates pregrasp position");

    const double direction_cutoff_hz = 2.0 / kPi; // Previous 0.25 s time constant.
    Direction dir;
    dir.reset(0.7);
    require(!dir.hasConfirmedDirection(),"Initial bearing is not a confirmed movement direction");
    auto update = [&](double vx, double vy, double time) {
      dir.update(vx,vy,0.05,0.08,0,time,direction_cutoff_hz,0.25,0.6);
    };
    update(0.001,-0.001,10.0);
    require(!dir.readyForGrasp(0.002,0.05,0.08),"One stationary sample cannot enable grasp");
    for (int i=1; i<=500; ++i) {
      // Single-sample spikes above the start threshold and rapid reversals
      // represent the stationary mocap noise seen in the recorded data.
      const double sign = i%2 ? 1.0 : -1.0;
      update(i%31==0 ? 0.13*sign : 0.03*sign, 0.025*sign, 10.0+0.01*i);
    }
    near(dir.yaw(),0.7,"Low-speed noise must not rotate the aircraft");
    require(!dir.readyForGrasp(0.03,0.05,0.08),"Noise spike restarts stationary confirmation");
    for (int i=1; i<=30; ++i) update(0.01,0.01,15.0+0.01*i);
    require(dir.readyForGrasp(0.03,0.05,0.08),"Stationary target may use held yaw");
    update(0,0.2,15.31);
    require(!dir.readyForGrasp(0.2,0.05,0.08),"A single moving sample cannot enable grasp");
    for (int i=32; i<=180; ++i) update(0,0.2,15.0+0.01*i);
    near(dir.yaw(),kPi/2,"Use confirmed filtered car velocity direction",1e-3);
    require(dir.readyForGrasp(0.2,0.05,0.08),"Stable movement enables heading gate");
    require(dir.hasConfirmedDirection(),"Confirmed motion direction is retained across a stop");
    const double last_reliable_yaw=dir.yaw();
    update(0.2,0,16.81);
    require(!dir.readyForGrasp(0.2,0.05,0.08),"A sudden direction change is not immediately confirmed");
    near(dir.yaw(),last_reliable_yaw,"A sudden direction change holds yaw until consistent");
    update(0,0,16.82);
    near(dir.yaw(),last_reliable_yaw,"Stopped car retains last direction");
    near(dir.rate(),0,"Stopped direction has no angular feedforward");
    require(dir.hasConfirmedDirection(),"Stop keeps the last confirmed motion direction");
    require(!dir.readyForGrasp(0.0,0.05,0.08),"Stop must be confirmed after movement");
    for (int i=183; i<=430; ++i) update(-0.2,0,15.0+0.01*i);
    near(std::abs(dir.yaw()),kPi,"Reverse follows velocity, not body nose",1e-4);

    Direction restarted;
    restarted.reset(0.0);
    for (int i=0; i<=10; ++i)
      restarted.update(0.2,0,0.05,0.08,0,100+i*0.05,1.0,0.25,0.6);
    require(restarted.readyForGrasp(0.2,0.05,0.08),"Motion confirmed before retrigger");
    restarted.resetEvidence();
    require(restarted.hasConfirmedDirection(),"Retrigger preserves last confirmed direction");
    near(restarted.yaw(),0.0,"Retrigger preserves yaw");
    near(restarted.rate(),0.0,"Retrigger clears direction rate");
    require(!restarted.readyForGrasp(0.2,0.05,0.08),"Retrigger revokes movement readiness");
    restarted.update(-0.2,0,0.05,0.08,0,101,1.0,0.25,0.6);
    near(restarted.yaw(),0.0,"First reversed sample cannot change held yaw");
    require(!restarted.readyForGrasp(0.2,0.05,0.08),"First reversed sample cannot enable grasp");
    for (int i=1; i<=4; ++i)
      restarted.update(-0.2,0,0.05,0.08,0,101+i*0.05,1.0,0.25,0.6);
    near(restarted.yaw(),0.0,"Reverse must persist through confirmation dwell");
    require(!restarted.readyForGrasp(0.2,0.05,0.08),"Reverse not ready before dwell");
    restarted.update(-0.2,0,0.05,0.08,0,101.30,1.0,0.25,0.6);
    near(std::abs(restarted.yaw()),kPi,"Stable reverse eventually replaces held yaw");
    require(restarted.readyForGrasp(0.2,0.05,0.08),"Stable reverse eventually becomes ready");
    restarted.update(0.2,0,0.05,0.08,0,104,1.0,0.25,0.6);
    near(std::abs(restarted.yaw()),kPi,"Long sample gap preserves the previous yaw");
    require(!restarted.readyForGrasp(0.2,0.05,0.08),"Long sample gap revokes readiness");
    for (int i=1; i<=6; ++i)
      restarted.update(0.2,0,0.05,0.08,0,104+i*0.05,1.0,0.25,0.6);
    near(angleError(restarted.yaw(),0),0,"Motion can be reconfirmed after a long gap");
    restarted.resetEvidence();
    restarted.update(0,0,0.05,0.08,0,105,1.0,0.25,0.6);
    require(!restarted.readyForGrasp(0,0.05,0.08),"Restart also revokes stationary readiness");
    for (int i=1; i<=6; ++i)
      restarted.update(0,0,0.05,0.08,0,105+i*0.05,1.0,0.25,0.6);
    require(restarted.readyForGrasp(0,0.05,0.08),"Stationary confirmation can recover");

    // Direction uses the configured Hz in alpha=1-exp(-2*pi*f*dt).
    for (const double cutoff : {direction_cutoff_hz, 2.0}) {
      Direction frequency_check;
      frequency_check.reset(0.0);
      for (int i=0; i<=20; ++i)
        frequency_check.update(0.2,0,0.05,0.08,0,70.0+i*0.05,cutoff,0.25,0.6);
      frequency_check.update(0.2,0.1,0.05,0.08,0,71.05,cutoff,0.25,0.6);
      const double alpha = -std::expm1(-2*kPi*cutoff*0.05);
      near(frequency_check.yaw(),std::atan2(0.1*alpha,0.2),
           "Direction filter uses Hz and sample interval",1e-10);
    }

    Direction slow;
    Sequence slow_sequence;
    slow.reset(0.0);
    for (int i=0; i<=10; ++i) {
      const double time=20.0+i*0.05;
      slow.update(0,0.075,0.05,0.08,0,time,direction_cutoff_hz,0.25,0.6);
      const bool ready=slow.readyForGrasp(0.075,0.05,0.08);
      const bool aligned=std::abs(angleError(slow.yaw(),0.0))<=0.174533;
      require(!slow_sequence.ready(time,ready && aligned,0.15),
              "A 0.075 m/s car at 90 degrees must not close before yaw alignment");
    }
    // Once the slower direction has persisted long enough, the target yaw is
    // exposed; the aircraft's actual yaw must still pass the planner gate.
    require(slow.readyForGrasp(0.075,0.05,0.08),"Stable slow direction eventually becomes usable");
    near(slow.yaw(),kPi/2,"Stable slow car points target heading along velocity",1e-8);
    require(!slow_sequence.ready(20.51,false,0.15),"Misaligned yaw keeps close gate closed");

    Direction transition;
    transition.reset(0.0);
    for (int i=0;i<=6;++i)
      transition.update(0,0,0.05,0.08,0,25.0+i*0.05,direction_cutoff_hz,0.25,0.6);
    require(transition.readyForGrasp(0,0.05,0.08),"Sustained zero velocity confirms stationary");
    transition.update(0,0.075,0.05,0.08,0,25.31,direction_cutoff_hz,0.25,0.6);
    require(!transition.readyForGrasp(0.075,0.05,0.08),
            "New low-speed motion immediately revokes stationary readiness");
    for(int i=1;i<=25;++i)
      transition.update(0,0.075,0.05,0.08,0,25.31+i*0.05,direction_cutoff_hz,0.25,0.6);
    require(transition.readyForGrasp(0.075,0.05,0.08),"Slow motion after stationary is eventually confirmed");
    near(transition.yaw(),kPi/2,"Slow motion after stationary sets moving direction",1e-8);

    Direction duplicate;
    duplicate.reset(0.0);
    for(int i=0;i<20;++i) duplicate.update(0,0,0.05,0.08,0,27.0,direction_cutoff_hz,0.25,0.6);
    require(!duplicate.readyForGrasp(0,0.05,0.08),"Repeated sample timestamps must not confirm stationary");

    Direction flicker;
    flicker.reset(0.3);
    for (int i=0; i<=80; ++i) {
      // Oscillation around the lower threshold must not establish either
      // stationary or motion without an uninterrupted evidence window.
      const double speed=i%2 ? 0.049 : 0.051;
      flicker.update(speed,0,0.05,0.08,0,30.0+i*0.05,direction_cutoff_hz,0.25,0.6);
      require(!flicker.readyForGrasp(speed,0.05,0.08),"Threshold flicker must not satisfy grasp readiness");
    }

    Direction wrap_direction;
    wrap_direction.reset(179*kPi/180);
    const double wrap_target=-179*kPi/180;
    for (int i=0;i<=20;++i)
      wrap_direction.update(0.2*std::cos(wrap_target),0.2*std::sin(wrap_target),
                            0.05,0.08,0,40.0+i*0.05,direction_cutoff_hz,0.25,0.6);
    near(angleError(wrap_direction.yaw(),wrap_target),0,"Direction filter unwraps across 180 degrees");
    require(std::abs(wrap_direction.yaw()-179*kPi/180)<0.05,"Direction filter turns along the short arc");

    Direction velocity_rate;
    velocity_rate.reset(0.0);
    for (int i=0;i<=30;++i)
      velocity_rate.update(0.15,0.0,0.05,0.08,0,50.0+i*0.05,direction_cutoff_hz,0.25,0.6);
    near(velocity_rate.rate(),0,"Straight velocity has zero yaw feedforward");
    for (int i=1;i<=80;++i) {
      const double a=0.2*i*0.05;
      velocity_rate.update(0.15*std::cos(a),0.15*std::sin(a),
                           0.05,0.08,0,51.5+i*0.05,direction_cutoff_hz,0.25,0.6);
    }
    near(velocity_rate.rate(),0.2,"Velocity direction turn rate follows constant turn",0.01);
    velocity_rate.update(0,0,0.05,0.08,0,55.55,direction_cutoff_hz,0.25,0.6);
    near(velocity_rate.rate(),0,"Unreliable or low speed disables yaw feedforward");

    Direction low_rate;
    low_rate.reset(0.0);
    for (int i=0;i<=40;++i) {
      const double a=0.2*i*0.05;
      low_rate.update(0.075*std::cos(a),0.075*std::sin(a),
                      0.05,0.08,0,60.0+i*0.05,direction_cutoff_hz,0.25,0.6);
    }
    require(low_rate.readyForGrasp(0.075,0.05,0.08),"Slow direction remains usable");
    near(low_rate.rate(),0,"Below high-speed threshold no yaw-rate feedforward");
    velocity_rate.update(0.15,0,0.05,0.08,0,56.0,direction_cutoff_hz,0.25,0.6);
    near(velocity_rate.rate(),0,"Long sample gap disables yaw-rate feedforward");

    auto wrapped=heading(179*kPi/180,0,-179*kPi/180,0,1,0.6,1.2);
    near(angleError(sampleHeading(wrapped,1)[0],-179*kPi/180),0,"Shortest-path wrap endpoint");
    require(std::abs(sampleHeading(wrapped,0.5)[0]-179*kPi/180)<0.05,"Do not turn 358 degrees");
    auto turning=heading(0,0.1,0,0.1,1,0.6,1.2);
    near(sampleHeading(turning,1)[0],0.1,"Predict endpoint direction at trajectory end");
    auto bounded=heading(0,0,kPi,0,0.5,0.6,1.2);
    for(const auto& piece:bounded) require(yawWithinLimits(piece,0.6,1.2),"Infeasible turn stays within limits");
    auto prior=sampleHeading(bounded,0.05);
    auto next=heading(prior[0],prior[1],-kPi+0.1,0,0.5,0.6,1.2);
    near(next.front().pos(0),prior[0],"Replanning yaw continuity");
    near(next.front().vel(0),prior[1],"Replanning yaw-rate continuity");
    std::cout << "Moving-grasp sequence, freshness, trajectory and heading checks passed\n";
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
