#include "main.h"
#include "gfr/api.hpp"
#include "gfr/controller/BoomerangControllerBuilder.hpp"
#include "gfr/controller/PIDControllerBuilder.hpp"
#include "gfr/controller/SwingControllerBuilder.hpp"
#include "gfr/localizer/ADILocalizerBuilder.hpp"
#include "gfr/utils/flags.hpp"

#define LEFT_MOTORS                                                            \
    { 8,-9,-10}
#define RIGHT_MOTORS                                                           \
    { -2,1,3}
pros::Motor intake1(21);
pros::Motor intake2(21);
auto odom = gfr::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_right_motor(-2)
                .with_left_motor(8)
                .with_track_width(15)
                .with_left_right_tpi(23.8732414638)
                .with_imu(7)
                .build();

auto pid = gfr::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(18, 0.02, 169)
                    .with_angular_constants(110, 0.05, 160)
               .with_min_error(5)
               .with_min_vel_for_thru(100)
               .build();

auto boomerang = gfr::controller::BoomerangControllerBuilder::new_builder(odom)
                     .with_linear_constants(18, 0.02, 169)
                    .with_angular_constants(110, 0.05, 160)
                     .with_lead_pct(0.3)
                     .with_min_vel_for_thru(70)
                     .with_min_error(5)
                     .build();
auto boomerang2 = gfr::controller::BoomerangControllerBuilder::new_builder(odom)
                     .with_linear_constants(30, 0.02, 169)
                    .with_angular_constants(50, 0.05, 160)
                     .with_lead_pct(0.1)
                     .with_min_vel_for_thru(100)
                     .with_min_error(5)
                     .build();
auto swing = gfr::controller::SwingControllerBuilder::new_builder(odom)
                 .with_angular_constants(30, 0.05, 180)
                 .build();

auto arc = gfr::controller::ArcPIDControllerBuilder(odom)
               .with_track_width(11)
                .with_linear_constants(30, 0.02, 169)
                .with_angular_constants(50, 0.05, 160)
               .with_min_error(5)
               .with_slew(8)
               .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);
auto ec = gfr::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 1.0, 200)
              .add_timeout(22800)
              .add_thru_smoothness(2)
              .build() -> exit_if([]() {
                  return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
              });

auto chassis = gfr::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec,
                                          pros::E_MOTOR_BRAKE_COAST);

pros::IMU imu(16);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    odom->begin_localization();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // auto odom = gfr::localizer::ADILocalizerBuilder::new_builder().build();
    // auto pid =
    // gfr::controller::BoomerangControllerBuilder::new_builder(odom)
    //                .with_lead_pct(60)
    //                .build();

    // // auto pid2 =
    // // gfr::controller::BoomerangControllerBuilder::new_builder(odom)
    // //                 .with_lead_pct(65)
    // //                 .build();

    // auto pid2 = gfr::controller::ControllerCopy(pid).modify_lead_pct(65);
     odom->set_pose({0.0, 0.0, 90});
     for( int i= 0; i < 20; i++){
    /*
        // circular movemnt
           chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);

            */

            //figure eight
            chassis.move({-30,40, 90},boomerang , 100, gfr::Flags::THRU);
            /*
            chassis.move({0, 40, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,0, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,40, 90},boomerang , 100, gfr::Flags::THRU);
            chassis.move({0, 40, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,0, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,40, 90},boomerang , 100, gfr::Flags::THRU);
            chassis.move({0, 40, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,0, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,40, 90},boomerang , 100, gfr::Flags::THRU);
            chassis.move({0, 40, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({-30,0, 0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
*/
            /*

            chassis.move({28,12,-100}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({5,3, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU);
        */
            /*
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU); 
             chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({20,12,-90}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU); 
             chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({20,12,-90}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU); 
             chassis.move({12,30,80}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({30,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({20,12,-90}, boomerang, 100, gfr::Flags::THRU); 
            chassis.move({0,0,90}, boomerang, 100, gfr::Flags::THRU); */
            
            /*chassis.move({10,30,0}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({13, 32, -30}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({18, 30, -90}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({19, -2, 180}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({6, -3, 130}, boomerang, 100, gfr::Flags::THRU);
            chassis.move({2, 0, 93}, pid, 100, gfr::Flags::THRU);
            */

   chassis.move({0,0,100}, boomerang, 100, gfr::Flags::NONE);  
}
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
bool isonepressed;
bool istwopressed;
void opcontrol() {

    while (true) {
        gfr::Pose p = odom->get_pose();

        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            isonepressed = !isonepressed;
            if(isonepressed){
                intake1.move(127);
                intake2.move(-127);
            }else{
                intake1.move(-127);
                intake2.move(127);

            }
        }
        
    }
}