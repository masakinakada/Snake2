//
//  GA.cpp
//  Snake
//
//  Created by Masaki Nakada on 2/13/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "GA.h"

using namespace std;

GA::GA(Creature *wCreature, World* wWorld){
    gaCreature = wCreature;
    gaWorld = wWorld;
    init();
}

void GA::init(){
    // The "Monkeys" are the individuals that will control the Creature.
    // Create the Monkeys and assign them numbers.
    for(int i=0;i<MONKEY_NUM;i++){
        monkeys[i] = new Monkey;
        monkeys[i]->set_number(i);
    }
    /*
    for(int i=0;i<SEATS_NUM;i++){
        seats[i] = new Monkey;
    }*/
    pickDrivers();
    bestDistance = 0.0;
    bestDriver = new Monkey;
    bestGeneration = 1;
    bestRun = 1;
    runCount = 1;
}

int GA::iRand(int floor, int ceiling)
{
    int dif = ceiling - floor;
    int v1 = rand() % dif + floor;
    return v1;
}

int* GA::shuffleCard()
{
    for (int i=0; i<MONKEY_NUM; i++) {
        card[i] = i;  // fill the array in order
    }
   
    for(int i=0;i<MONKEY_NUM; i++)
    {
        int target = iRand(0, MONKEY_NUM);
        int temp = card[i]; card[i] = card[target]; card[target] = temp;
    }
    return card;
}

void GA::pickDrivers(){
    
    cout << "selected Monkeys are"<<endl;
    for(int i=0;i<SEATS_NUM;i++){
        shuffleCard();
        seats[i] = monkeys[card[i]];
        //seats[i]->set_number(monkeys[i]->get_number());
        cout<<card[i]<<endl;
       
    }
    currentSeat = 0;
    
    cout<<"Next drivers are"<<endl;
    for(int i=0; i<SEATS_NUM;i++){
        cout<<"Moneky # " <<seats[i]->get_number()<<endl;
    }
}

void GA::iterate(float time, float dt)
{
    static float accumulated_time = 0.0;
    accumulated_time+=dt;
    seats[currentSeat]->control_robot(*gaCreature, time, dt, 1.0);
    
    int driver_num = seats[currentSeat]->get_number();
    float driven_distance = seats[currentSeat]->get_distance();
    
    cout << "Seat is # "<< driver_num <<endl;
    cout << "distance is " << driven_distance << endl;
    if(accumulated_time>10.0)
    {
        changeDrivers();
        accumulated_time = 0.0;
        cout << "Chaning Driver to the next monkey....." << endl;
        cout << "next driver is monkey # " <<seats[currentSeat]->get_number()<<endl;
    }
}

//this function should be called every 20 msec
void GA::changeDrivers()
{
    seats[currentSeat]->set_distance(gaCreature->getDistance());
   
    if(seats[currentSeat]->get_distance()>bestDistance){
        bestDistance = seats[currentSeat]->get_distance();
        bestDriver = seats[currentSeat];
        bestGeneration = seats[currentSeat]->get_generation();
        bestRun = runCount;
    }
    
    //get the creature back to the origin and let the new driver to handle the control
    gaCreature->to_center();
    currentSeat++;
    
    if(currentSeat>SEATS_NUM)
    {
        cout << "breading monkeys, and generate new generation" <<endl;
        breadMonkeys(runCount);
        pickDrivers();
        runCount++;
    }
    
    
    
}

Monkey* GA::sortByDistance(Monkey *seats)
{
    int max = SEATS_NUM;
    for(int i = 0; i < max; i++)
    {
        for(int j = 0; j < max; j++)
        {
            if(seats[i].get_distance() < seats[j].get_distance())
            {
                Monkey temp = seats[i];
                seats[i] = seats[j];
                seats[j] = temp;
            }
        }
    }
    
    return seats;
    
}

Monkey* GA::breadMonkeys(int runN)
{
    Monkey *sortedSeats = sortByDistance(*seats);
    sortedSeats[0].bread_monkeys(sortedSeats[1], *seats[2], runN);
    sortedSeats[1].bread_monkeys(sortedSeats[0], *seats[3], runN);
    ;
    seats[2]->mutate_genome(runN, runN);
    seats[3]->mutate_genome(runN, runN);
    return sortedSeats;
}
