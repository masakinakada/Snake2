//
//  GA.cpp
//  Snake
//
//  Created by Masaki Nakada on 2/13/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "GA.h"

using namespace std;

GA::GA(Snake *wSnake, World* wWorld){
    gaSnake = wSnake;
    gaWorld = wWorld;
    init();
}

void GA::init(){
    // The "Monkeys" are the individuals that will control the Snake.
    // Create the Monkeys and assign them numbers.
    for(int i=0;i<MONKEY_NUM;i++){
        monkeys[i] = new Monkey;
        monkeys[i]->set_number(i);
    }

    pickDrivers();
    bestDistance = 0.0;
    bestGeneration = 0;
    bestRun = 0;
    runCount = 0;
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
        int target = iRand(0, MONKEY_NUM-1);
        int temp = card[i]; card[i] = card[target]; card[target] = temp;
    }
    return card;
}

void GA::pickDrivers(){
    
    cout << "selected Monkeys are"<<endl;
    shuffleCard();
    for(int i=0;i<SEATS_NUM;i++){
        seats[i] = monkeys[card[i]];
        //seats[i]->set_number(monkeys[i]->get_number());
        cout<<"Monkey #" <<card[i]<<endl;
       
    }
    currentSeat = 0;
    cout<<"First driver is Monkey #"<<seats[currentSeat]->get_number()<<endl;
}

void GA::iterate(float time, float dt)
{
    static float accumulated_time = 0.0;
    accumulated_time+=dt;
    seats[currentSeat]->control_robot(*gaSnake, time, dt, 1.0);
    
    
    // cout << "distance is " << driven_distance << endl;
    if(accumulated_time>10.0)
    {
        changeDrivers();
        accumulated_time = 0.0;

    }
}

//this function should be called every 20 msec
void GA::changeDrivers()
{
    seats[currentSeat]->set_distance(gaSnake->getDistance());
    cout<<"Monkey #"<<seats[currentSeat]->get_number()<<" moved " <<seats[currentSeat]->get_distance()<<endl;
    
    if(seats[currentSeat]->get_distance()>bestDistance){
        bestDistance = seats[currentSeat]->get_distance();
        bestDriver = seats[currentSeat];
        bestGeneration = seats[currentSeat]->get_generation();
        bestRun = runCount;
        
        cout<<"new Best Distance:" <<bestDistance<<endl;
        cout<<"new Best Driver: Monkey #"<<bestDriver->get_number()<<endl;
        cout<<"new Best Generation: "<<bestGeneration<<endl;
        cout<<"new Best Run count: "<<bestRun<<endl;
    }
    
	gaSnake->initPhysics();
    
    if(currentSeat>SEATS_NUM-2)
    {
        cout << "breading monkeys, and generate new generation" <<endl;
        breadMonkeys(runCount);
        pickDrivers();
        cout<<"Run Count: "<<runCount++<<endl;
    } else{
        cout << "Chaning Driver to the next monkey....." << endl;
    
        currentSeat++;
        //get the Snake back to the origin and let the new driver to handle the control
        int driver_num = seats[currentSeat]->get_number();
        cout << "next driver is monkey # "<< driver_num <<endl;
    }
}

void GA::sortByDistance()
{
    int max = SEATS_NUM;
    for(int i = 0; i < max; i++)
    {
        for(int j = 0; j < max; j++)
        {
            if(seats[i]->get_distance() > seats[j]->get_distance())
            {
                Monkey* temp = seats[i];
                seats[i] = seats[j];
                seats[j] = temp;
            }
        }
    }
    
    cout<<"sorted order is"<<endl;
    for(int i=0;i<max;i++)
    {
        cout<<"Monkey #" <<seats[i]->get_number()<<endl;;
    }
}

void GA::breadMonkeys(int runN)
{
    sortByDistance();
    cout <<"bread Monkey#"<<seats[0]->get_number()<<" and Monkey #" <<seats[1]->get_number()<<", set the breaded genome to Monkey #"<< seats[2]->get_number()<<endl;
    cout <<"bread Monkey#"<<seats[1]->get_number()<<" and Monkey #" <<seats[0]->get_number()<<", set the breaded genome to Monkey #"<< seats[3]->get_number()<<endl;
    seats[0]->bread_monkeys(*seats[1], *seats[2], runN);
    seats[1]->bread_monkeys(*seats[0], *seats[3], runN);
    ;
    
    cout<<"Mutate Monkey #" <<seats[2]->get_number()<<endl;
    cout<<"Mutate Monkey #" <<seats[3]->get_number()<<endl;
    seats[2]->mutate_genome(seats[2]->get_number(), runN);
    seats[3]->mutate_genome(seats[3]->get_number(), runN);
}
