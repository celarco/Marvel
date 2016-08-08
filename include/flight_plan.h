/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/ 
    
#ifndef FLIGHT_PLAN_HEADER
#define FLIGHT_PLAN_HEADER

#include <string>
#include <iostream>
#include "guidance_pack.h"

#define MAX_BLOCK_COUNT 20
#define MAX_FUNCTION_COUNT 100
#define MAX_FUNCTION_ARG_COUNT 5

enum function_type { take_off = 1,
                     hold_position,
                     set_heading,
                     heading_lock,
                     heading_unlock,
                     rotate,
                     move_oa,
                     move,
                     go,
                     go_oa

};

struct function {
    function_type type;
    float arg[MAX_FUNCTION_ARG_COUNT];
    bool done = false;
};

struct block {
    unsigned short int start_function_number;
    std::string name;
    unsigned short int id;
};


#endif
