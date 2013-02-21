//
//  Genome.cpp
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

//caution
// need to chack raodome generator funciton. could be wrong. check outputs

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include "Genome.h"
#include <array>

//MUSCLE_NUM * 2 + 1
#define NUMBER_OF_GENE 5


using namespace std;

Genome::Genome(){
}

float Genome::get_genomeData(int num)
{
    return genomeData[num];
}

float Genome::fRand(float floor, float ceiling)
{
    float rnd = floor + (float)rand()/((float)RAND_MAX/(ceiling-floor));
    return rnd;
}

int Genome::iRand(int floor, int ceiling)
{
    int dif = ceiling - floor;
   int v1 = rand() % dif + floor;
    return v1;
}

void Genome::randomize()
{
    static int monkey_number = 0;
  
    cout<<"Genome for monkey #" << monkey_number<<endl;
    //for phase and velocity of sinosoid
    for (int i=0; i<MUSCLE_NUM; i++) {
        genomeData[i] = fRand(1.0, 2.0);
        genomeData[i+MUSCLE_NUM] = fRand(3.5, 6.3);
        cout<<"new genome Data"<<"(w"<<i<<") =" << genomeData[i]<< endl;
        cout<<"new genome Data"<<"(phase"<<i<<") =" << genomeData[i+MUSCLE_NUM]<< endl;
       
    }
    //this is for amplitude of sinosoid
    genomeData[2*MUSCLE_NUM] = fRand(-4.0, 4.0);
     cout<<"new genome Data"<<"(amplitude) =" << genomeData[2*MUSCLE_NUM]<< endl;
    
    cout<<"Control sinosoidal function for Monkey #"<<monkey_number<<" is "<<endl;
    cout<<"First segment:"<<genomeData[2*MUSCLE_NUM] <<" * sin("<<genomeData[0]<<"*t + "<<genomeData[1]<<")"<<endl;
    cout<<"Second segment:"<<genomeData[2*MUSCLE_NUM] <<" * sin("<<genomeData[2]<<"*t + "<<genomeData[3]<<")"<<endl;
    
    monkey_number++;
}

int Genome::calculate_torque(int k, float time)
{
    float sin_value = 2*(sin(genomeData[2*MUSCLE_NUM] * (time+genomeData[k*MUSCLE_NUM]) - (genomeData[k])));
    if(sin_value>1){
        return 2;
    }
    else if(sin_value>0){
        return 1;
    }else if(sin_value>-1){
        return -1;
    }else{
        return -2;
    }
}

void Genome::mutate(int num, int generation)
{
    int n = iRand(0,NUMBER_OF_GENE);
    if(n<MUSCLE_NUM) genomeData[n] = fRand(1.0, 2.0);
    else if(n<MUSCLE_NUM*2) genomeData[n] = fRand(3.5, 6.3);
    else genomeData[MUSCLE_NUM*2] = fRand(-4.0, 4.0);
    
    cout<<"gene # " << n << "mutated for Monkey #" << num <<" of generation " << generation << endl;
}

void Genome::crossover(Genome parent1, Genome parent2)
{
    int n, crossoverPoint;
    crossoverPoint = iRand(0, MUSCLE_NUM*2-1);
    
    if(crossoverPoint==0){
        Genome tmp;
        tmp = parent2;
        parent2 = parent1;
        parent1 = tmp;
    }
    
    n = 0;
    while(n<sizeof(genomeData) / sizeof(float))
    {
        if(n<crossoverPoint)
            genomeData[n] = parent1.get_genomeData(n);
        else
            genomeData[n] = parent2.get_genomeData(n);
        
        n++;
    }
    
    
}



