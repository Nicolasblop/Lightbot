/**
* By: Ben Earle
* ARSLab - Carleton University
*
* Analog Input:
* This main file constructs the do simple line following project, using a Seed Bot Shield.
* Its purpose is to demonstrate how to use all of the port IO models in ECADMIUM.
*/

#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>

#include <cadmium/modeling/coupled_model.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
#include <cadmium/modeling/dynamic_coupled.hpp>
#include <cadmium/modeling/dynamic_atomic.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/tuple_to_ostream.hpp>
#include <cadmium/logger/common_loggers.hpp>
#include <cadmium/io/iestream.hpp>


#include <NDTime.hpp>

#include <cadmium/embedded/io/digitalInput.hpp>
#include <cadmium/embedded/io/analogInput.hpp>
#include <cadmium/embedded/io/pwmOutput.hpp>
#include <cadmium/embedded/io/digitalOutput.hpp>

#include "../atomics/lightBot.hpp"

#ifdef ECADMIUM
  #include "../mbed.h"
#else
  const char* A2  = "./inputs/A2_CenterIR_In.txt";
  const char* A4  = "./inputs/A4_leftLightSens_In.txt";
  const char* A5  = "./inputs/A5_rightLightSens_In.txt";
  const char* D8  = "./outputs/D8_RightMotor1_Out.txt";
  const char* D11 = "./outputs/D11_RightMotor2_Out.txt";
  const char* D12 = "./outputs/D12_LeftMotor1_Out.txt";
  const char* D13 = "./outputs/D13_LeftMotor2_Out.txt";
#endif

using namespace std;

using hclock=chrono::high_resolution_clock;
using TIME = NDTime;

int main(int argc, char ** argv) {

  //This will end the main thread and create a new one with more stack.
  #ifdef ECADMIUM
    //Logging is done over cout in ECADMIUM
    struct oss_sink_provider{
      static std::ostream& sink(){
        return cout;
      }
    };
  #else
    // all simulation timing and I/O streams are ommited when running embedded

    auto start = hclock::now(); //to measure simulation execution time

    /*************** Loggers *******************/

    static std::ofstream out_data("seeed_bot_test_output.txt");
    struct oss_sink_provider{
      static std::ostream& sink(){
        return out_data;
      }
    };
  #endif

  using info=cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using debug=cadmium::logger::logger<cadmium::logger::logger_debug, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using state=cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_messages=cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using routing=cadmium::logger::logger<cadmium::logger::logger_message_routing, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using global_time=cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using local_time=cadmium::logger::logger<cadmium::logger::logger_local_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_all=cadmium::logger::multilogger<info, debug, state, log_messages, routing, global_time, local_time>;
  using logger_top=cadmium::logger::multilogger<log_messages, global_time>;


/*******************************************/


/********************************************/
/*********** APPLICATION GENERATOR **********/
/********************************************/
  using AtomicModelPtr=std::shared_ptr<cadmium::dynamic::modeling::model>;
  using CoupledModelPtr=std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>>;

/********************************************/
/********** LightBot ************************/
/********************************************/

  AtomicModelPtr lightBot = cadmium::dynamic::translate::make_dynamic_atomic_model<LightBot, TIME>("lightBot");

/********************************************/
/****************** Input *******************/
/********************************************/

  AtomicModelPtr centerIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("centerIR", A2);
  
  AtomicModelPtr rightLightSens = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("rightLightSens", A5);
  AtomicModelPtr leftLightSens = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("leftLightSens", A4);
 
/********************************************/
/***************** Output *******************/
/********************************************/

  AtomicModelPtr rightMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("rightMotor1", D8);
  AtomicModelPtr rightMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("rightMotor2", D11);
  AtomicModelPtr leftMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("leftMotor1", D12);
  AtomicModelPtr leftMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("leftMotor2", D13);


/************************/
/*******TOP MODEL********/
/************************/
  cadmium::dynamic::modeling::Ports iports_TOP = {};
  cadmium::dynamic::modeling::Ports oports_TOP = {};

  cadmium::dynamic::modeling::Models submodels_TOP =  {rightLightSens, leftLightSens, lightBot, centerIR, rightMotor1, rightMotor2, leftMotor1, leftMotor2};

  cadmium::dynamic::modeling::EICs eics_TOP = {};
  cadmium::dynamic::modeling::EOCs eocs_TOP = {};
  cadmium::dynamic::modeling::ICs ics_TOP = {
     cadmium::dynamic::translate::make_IC<lightBot_defs::rightMotor1, pwmOutput_defs::in>("lightBot","rightMotor1"),
     cadmium::dynamic::translate::make_IC<lightBot_defs::rightMotor2, digitalOutput_defs::in>("lightBot","rightMotor2"),
     cadmium::dynamic::translate::make_IC<lightBot_defs::leftMotor1, pwmOutput_defs::in>("lightBot","leftMotor1"),
     cadmium::dynamic::translate::make_IC<lightBot_defs::leftMotor2, digitalOutput_defs::in>("lightBot","leftMotor2"),

     cadmium::dynamic::translate::make_IC<analogInput_defs::out, lightBot_defs::rightLightSens>("rightLightSens", "lightBot"),
     cadmium::dynamic::translate::make_IC<analogInput_defs::out, lightBot_defs::leftLightSens>("leftLightSens", "lightBot"),

     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, lightBot_defs::centerIR>("centerIR", "lightBot")
  };
  CoupledModelPtr TOP = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
   "TOP",
   submodels_TOP,
   iports_TOP,
   oports_TOP,
   eics_TOP,
   eocs_TOP,
   ics_TOP
   );

///****************////

  #ifdef ECADMIUM
    //Enable the motors:
    DigitalOut rightMotorEn(D9);
    DigitalOut leftMotorEn(D10);
    rightMotorEn = 1;
    leftMotorEn = 1;
  #endif

  // Logs are currently blocking opperations. It is recommended to turn them off when embedding your application.
  // They can be used for testing; however, keep in mind they will add extra delay to your model.

  cadmium::dynamic::engine::runner<NDTime, cadmium::logger::not_logger> r(TOP, {0});

  //cadmium::dynamic::engine::runner<NDTime, log_all> r(TOP, {0});

  r.run_until(NDTime("00:10:00:000"));

  #ifndef ECADMIUM
    return 0;
  #endif
}
