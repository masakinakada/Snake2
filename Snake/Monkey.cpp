//
//  Monkey.cpp
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "Monkey.h"
#define SPEED_K 1
#define MUSCLE_NUM 2

Monkey::Monkey()
{
    init();
}

int Monkey::get_number()
{
    return number;
}

void Monkey::set_number(int n)
{
    number = n;
}

void Monkey::increase_generation()
{
    generation++;
}

void Monkey::set_generation(int n)
{
    generation = n;
}

int Monkey::get_generation()
{
    return generation;
}

void Monkey::set_runNumber(int n)
{
    runNumber = n;
}

int Monkey::get_runNumber()
{
    return runNumber;
}

void Monkey::init(){
    genome = new Genome();
    randomize();
    set_generation(1);
    set_runNumber(1);
}

void Monkey::randomize()
{
    genome->randomize();
}

Genome Monkey::get_genome(){
    return *genome;
}

void Monkey::mutate_genome(int num, int generation)
{
    genome->mutate(num, generation);
}

void Monkey::bread_monkeys(Monkey otherMonkey, Monkey child, int runN){
    int otherNumber, childNumber;
    otherNumber = otherMonkey.get_number();
    childNumber = child.get_number();
    
    child.get_genome().crossover(otherMonkey.get_genome(), get_genome());
    child.increase_generation();
    child.set_runNumber(runN);
}

void Monkey::control_robot(Snake Snake, float time, float dt, float alpha)
{
    for(int k=0; k<MUSCLE_NUM; k++)
    {
        Snake.set_joint_velocity(k, SPEED_K*get_genome().calculate_torque(k, time), 0, dt, alpha);
    }
}

void Monkey::set_distance(float value)
{
    distanceTraveled = value;
}

float Monkey::get_distance()
{
    return distanceTraveled;
}