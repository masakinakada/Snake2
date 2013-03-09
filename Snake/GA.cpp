//
//  GA.cpp
//  Snake
//
//  Created by Masaki Nakada on 2/13/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "GA.h"
#include <iostream>
#include <fstream>
using namespace std;

GA::GA(World* wWorld){
    for(int i=0;i<SEATS_NUM;i++){
        gaSnake[i] = new Snake(MUSCLE_NUM+1, i);
        wWorld->Add_Object(gaSnake[i]);
    }
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
    bestDistance = -100.0;
    bestGeneration = 0;
    bestRun = 0;
    runCount = 0;
}

void GA::writeBest(){
    
    if (runCount>0) {
       
        ofstream myfile;
        myfile.open ("bestPopulation.txt");
        myfile << "Writing best population.\n";
        myfile << "best distance: ";
        myfile << bestDistance << "\n";
        myfile << "best generation: ";
        myfile << bestGeneration <<"\n";
        myfile << "best Run: ";
        myfile << bestRun <<"\n";
        myfile <<"best Genome" <<"\n";
        for (int i=0; i<GENOME_NUM; i++) {
            
            myfile << bestDriver->get_genome().get_genomeData(i) <<", ";
        }
        myfile << "\n";
        
        myfile.close();
    }
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
}

void GA::iterate(float time, float dt)
{
    static float accumulated_time = 0.0;
    accumulated_time+=dt;
    for(int i=0;i<SEATS_NUM;i++)
    {
        seats[i]->control_robot(*gaSnake[i], accumulated_time, dt, 2.0);
    }
    if(accumulated_time>10.0)
    {
        changeDrivers();
        accumulated_time = 0.0;

    }
}

//this function should be called every 20 msec
void GA::changeDrivers()
{
    
    for(int i=0; i<SEATS_NUM; i++){
    	
        seats[i]->set_distance(gaSnake[i]->getDistance(i));
        cout<<"Monkey #"<<seats[i]->get_number()<<" moved " <<seats[i]->get_distance()<<endl;
        
        if(seats[i]->get_distance()>bestDistance){
            bestDistance = seats[i]->get_distance();
            bestDriver = seats[i];
            bestGeneration = seats[i]->get_generation();
            bestRun = runCount;
        
            cout<<"new Best Distance:" <<bestDistance<<endl;
            cout<<"new Best Driver: Monkey #"<<bestDriver->get_number()<<endl;
            cout<<"new Best Generation: "<<bestGeneration<<endl;
            cout<<"new Best Run count: "<<bestRun<<endl;
        }
        
        //termination condition. if they move more than 300. we stop the simulation and return the best individual
        if(seats[i]->get_distance()>100){
            writeBest();
            exit(0);
        }
        
        gaSnake[i]->Reinit(i);
    }
    
      
    cout << "breading monkeys, and generate new generation" <<endl;
    breadMonkeys(runCount);
    pickDrivers();
    cout<<"Run Count: "<<runCount++<<endl;
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
    for (int i=0; i<SEATS_NUM/2; i++) {
        
        if(i<SEATS_NUM/2-1){
            cout <<"bread Monkey#"<<seats[i]->get_number()<<" and Monkey #" <<seats[i+1]->get_number()<<", set the breaded genome to Monkey #"<< seats[SEATS_NUM/2+i]->get_number()<<endl;
            seats[i]->bread_monkeys(*seats[i+1], *seats[SEATS_NUM/2+i], runN);
            
            cout<<"Mutate Monkey #" <<seats[SEATS_NUM/2+i]->get_number()<<endl;
            seats[SEATS_NUM/2+i]->mutate_genome(seats[SEATS_NUM/2+i]->get_number(), seats[SEATS_NUM/2+i]->get_generation());
            
            seats[SEATS_NUM/2+i]->increase_generation();
            cout<< "evolve monkey # " << seats[SEATS_NUM/2+i]->get_number()<< " in generation #"<< seats[SEATS_NUM/2+i]->get_generation() <<endl;
        }else{
            cout <<"bread Monkey#"<<seats[i]->get_number()<<" and Monkey #" <<seats[0]->get_number()<<", set the breaded genome to Monkey #"<< seats[SEATS_NUM/2+i]->get_number()<<endl;
            seats[i]->bread_monkeys(*seats[0], *seats[SEATS_NUM/2+i], runN);
            
            cout<<"Mutate Monkey #" <<seats[SEATS_NUM/2+i]->get_number()<<endl;
            seats[SEATS_NUM/2+i]->mutate_genome(seats[SEATS_NUM/2+i]->get_number(), seats[SEATS_NUM/2+i]->get_generation());
            seats[SEATS_NUM/2+i]->increase_generation();
            cout<< "evolve monkey # " << seats[SEATS_NUM/2+i]->get_number()<< " in generation #"<< seats[SEATS_NUM/2+i]->get_generation() <<endl;
        }
    }
    
}
