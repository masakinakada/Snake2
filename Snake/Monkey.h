//
//  Monkey.h
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "Genome.h"
#include "Snake.h"
#ifndef test_Monkey_h
#define test_Monkey_h

class Monkey
{
public:
    Monkey();
    void set_number(int n);
    int get_number();
    void increase_generation();
    int get_generation();
    void set_generation(int n);
    void set_runNumber(int n);
    int get_runNumber();
    void init();
    void randomize();
    Genome get_genome();
    void mutate_genome(int num, int generation);
    void bread_monkeys(Monkey otherMonkey, Monkey child, int runN);
    void control_robot(Snake Snake, float time, float dt);
    void set_distance(float value);
    float get_distance();
private:
    
    float distanceTraveled;
    int number, generation, runNumber;
    Genome *genome;
};


#endif
