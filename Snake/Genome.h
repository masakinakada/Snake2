//
//  Genome.h
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#ifndef test_Genome_h
#define test_Genome_h
#define MUSCLE_NUM 16 
#define GENOME_NUM 3

class Genome
{
public:
    Genome();
    float genomeData[GENOME_NUM];
    float get_genomeData(int num);
    void randomize();
    int calculate_torqueH(int k, float time);
    int calculate_torqueV(int k, float time);
    void mutate(int num, int generation);
    void crossover(Genome parent1, Genome parent2);
    float fRand(float floor, float ceiling);
    int iRand(int floor, int ceiling);
};

#endif
