//
//  Genome.h
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#ifndef test_Genome_h
#define test_Genome_h
#define MUSCLE_NUM 27 

class Genome
{
public:
    Genome();
    float genomeData[MUSCLE_NUM*2+1];
    float get_genomeData(int num);
    void randomize();
    int calculate_torque(int k, float time);
    void mutate(int num, int generation);
    void crossover(Genome parent1, Genome parent2);
    float fRand(float floor, float ceiling);
    int iRand(int floor, int ceiling);
};

#endif
