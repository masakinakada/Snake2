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


#ifndef M_PI
#  define M_PI  3.14159265358979323846
#endif
//#include <array>



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

void Genome::genomeGeneration(int i)
{
    switch (i) {
        case 0:
            //frequency is optimal around 4.0. this could be chnaged, think more if necessary
            genomeData[i] = fRand(5.0, 8.5);
            break;
        case 1:
            //45 degree is the one cicle. 0-1 with sin function. adjacent one is one phase different at most, and should have same phase most of the time.
            //adjacent ones cannot have the differnece more than one phase to make the periodic pattern
            // new change: at leaset two of the adjacent ones are in the same phase
            genomeData[i] = fRand(0.0, M_PI/4);
            break;
        case 2:
            genomeData[i] = fRand(0, 2*M_PI);
            break;
            //this conmbination is proved to work well
            /*
        case 3:
            genomeData[i] = fRand(1.5, 1.9);
            break;
        case 4:
            genomeData[i] = fRand(2.0, 2.4);
            break;
           */ 
        case 3:
            genomeData[i] = fRand(1.0, 1.9);
            break;
        case 4:
            genomeData[i] = fRand(1.5, 2.4);
            break;
        default:
            break;
    }
}

void Genome::randomize()
{
    static int monkey_number = 0;
  
    cout<<"Genome for monkey #" << monkey_number<<endl;
    //for phase and velocity of sinosoid
    
    for (int i=0; i<GENOME_NUM; i++) {
        
        genomeGeneration(i);
        cout<<"new genome Data"<<"(w"<<i<<") =" << genomeData[i]<< endl;
    }
    //this is for amplitude of sinosoid
    
    cout<<"Control sinosoidal function for Monkey #"<<monkey_number<<" is "<<endl;
    for(int i=0;i<MUSCLE_NUM;i++)
    {
        
        cout<<i<<"th segment:" <<" sin("<<genomeData[0] <<"*("<<"t) + " <<genomeData[1] <<"*" <<i <<") "<<endl;
        cout<<"apha1 = " <<genomeData[3]<<", alpha2 = " << genomeData[4] <<endl;
    }

    monkey_number++;
}

void Genome::mutate(int num, int generation)
{
    int n = iRand(0,GENOME_NUM*1.1);
    genomeGeneration(n);
    
    if(n<GENOME_NUM){
        cout<<"gene # " << n << " mutated for Monkey #" << num <<" of generation " << generation << endl;
    }
}


int Genome::calculate_torqueH(int k, float time)
{
    float sin_value = 2*(sin(genomeData[0] * time+ k * genomeData[1]));
    //float sin_value = 2*(sin(5.57416 * time + k * 0.258493));
    if(sin_value>sqrt(2)){
        return 2;
    }
    else if(sin_value>0){
        return 1;
    }else if(sin_value>-sqrt(2)){
        return -1;
    }else{
        return -2;
    }
}

int Genome::calculate_torqueV(int k, float time)
{
    float sin_value = 2*(sin(genomeData[0] * time+ k * genomeData[1] + genomeData[2]));
   // float sin_value = 2*(sin(5.57416 * time + k * 0.258493 + 3.26893));
    
    if(sin_value>sqrt(2)){
        return 2;
    }
    else if(sin_value>0){
        return 1;
    }else if(sin_value>-sqrt(2)){
        return -1;
    }else{
        return -2;
    }
}


void Genome::crossover(Genome parent1, Genome parent2)
{
    int n, crossoverPoint;
    crossoverPoint = iRand(0, GENOME_NUM-1);
    
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



