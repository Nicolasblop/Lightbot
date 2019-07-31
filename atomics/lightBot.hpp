/**
* ARSLab - Carleton University
*
* Lightbot:
* This model will drive towards a bright source of light using a Seed Bot Shield.


*/
#ifndef BOOST_SIMULATION_PDEVS_LIGHTBOT_HPP
#define BOOST_SIMULATION_PDEVS_LIGHTBOT_HPP

//used libraries and headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <limits>
#include <math.h> 
#include <assert.h>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>


using namespace cadmium;
using namespace std;



enum DriveState {right = 0, straight = 1, left = 2, stop = 3};



//Port definition
    struct lightBot_defs {
        //Output ports
        struct rightMotor1 : public out_port<float> { };
        struct rightMotor2 : public out_port<bool> { };
        struct leftMotor1 : public out_port<float> { };
        struct leftMotor2 : public out_port<bool> { };
        //Input ports
        struct rightLightSens : public in_port<float> { }; //analogic sensor => float
        struct centerIR : public in_port<bool> { }; // digital sensor => bool
        struct leftLightSens : public in_port<float> { };
    };


    template<typename TIME>



    class LightBot {
        using defs=lightBot_defs; // putting definitions in context
        public:
            //Parameters to be overwriten when instantiating the atomic model
            TIME   slowToggleTime;
            TIME   fastToggleTime;

            // default constructor
            LightBot() noexcept{
              state.dir = straight;
            }
            
            // state definition
            struct state_type{
              DriveState dir;
              bool prop;
            }; 
            state_type state;


            // ports definition
            using input_ports=std::tuple<typename defs::rightLightSens, typename defs::leftLightSens, typename defs::centerIR>;
            using output_ports=std::tuple<typename defs::rightMotor1, typename defs::rightMotor2, typename defs::leftMotor1, typename defs::leftMotor2>;


            // internal transition
            void internal_transition() {
              state.prop = false;
            }


            // external transition
            void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) { 
              float lightRight = 0;
              float lightLeft = 0;
              bool centerIR = false;

              for(const auto &x : get_messages<typename defs::centerIR>(mbs)){
                state.centerIR = !x;
              }
              
              for(const auto &x : get_messages<typename defs::rightLightSens>(mbs)){
                lightRight = x;
              }

              for(const auto &x : get_messages<typename defs::leftLightSens>(mbs)){
                lightLeft = x;
              }              

              if(state.centerIR) {
                //if centerIR doesn't see the ground, bot stops
                state.dir = DriveState::stop;
              } else if ((lightLeft-lightRight)<>0.1) { //10% difference between left and right sensor
                state.dir = DriveState::right;
              } else if ((lightRight-lightLeft)>0.1) {
                state.dir = DriveState::left;
              } else {
                state.dir = DriveState::straight;
              }
              state.prop = true;
            }

            // confluence transition
            void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
              internal_transition();
              external_transition(TIME(), std::move(mbs));
            }

            // output function
            typename make_message_bags<output_ports>::type output() const {
              typename make_message_bags<output_ports>::type bags;
              float rightMotorOut1;
              bool rightMotorOut2;
              float leftMotorOut1;
              bool leftMotorOut2;  

              switch(state.dir){
                case DriveState::right:
                  rightMotorOut1 = 0;
                  rightMotorOut2 = 0.5;
                  leftMotorOut1 = 0;
                  leftMotorOut2 = 1;                
                break;

                case DriveState::left:
                  rightMotorOut1 = 0;
                  rightMotorOut2 = 1;
                  leftMotorOut1 = 0;
                  leftMotorOut2 = 0.5;
                break;

                case DriveState::straight:
                  rightMotorOut1 = 0;
                  rightMotorOut2 = 1;
                  leftMotorOut1 = 0;
                  leftMotorOut2 = 1;
                break;

                case DriveState::stop:
                default:
                  rightMotorOut1 = 0;
                  rightMotorOut2 = 0;
                  leftMotorOut1 = 0;
                  leftMotorOut2 = 0;
                break;
              }

              get_messages<typename defs::rightMotor1>(bags).push_back(rightMotorOut1);
              get_messages<typename defs::rightMotor2>(bags).push_back(rightMotorOut2);
              get_messages<typename defs::leftMotor1>(bags).push_back(leftMotorOut1);
              get_messages<typename defs::leftMotor2>(bags).push_back(leftMotorOut2);
                
              return bags;
            }

            // time_advance function
            TIME time_advance() const { 
            	if(state.prop){
                	return TIME("00:00:00");
                }else{
              		return std::numeric_limits<TIME>::infinity();
          		}
            }

            friend std::ostringstream& operator<<(std::ostringstream& os, const typename LightBot<TIME>::state_type& i) {
              os << "Current state: " << i.dir; 
              return os;
            }
        };

#endif // BOOST_SIMULATION_PDEVS_LIGHTBOT_HPP