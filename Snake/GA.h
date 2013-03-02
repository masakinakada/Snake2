//
//  GA.h
//  Snake
//
//  Created by Masaki Nakada on 2/13/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#ifndef Snake_GA_h
#define Snake_GA_h

#include "Monkey.h"
#include "Snake.h"

#define SEATS_NUM 4
#define MONKEY_NUM 15



class GA
{
public:
    GA(World* wWorld);
    void init();
    void pickDrivers();
    int currentSeat;
   
    void iterate(float time, float dt);
    void changeDrivers();
    void getDistance();
    void compareDistance();
    void breadMonkeys( int runN);
    void sortByDistance();
    int iRand(int floor, int ceiling);
    int* shuffleCard();
    
private:
    Monkey* monkeys[15];
    Monkey* seats[SEATS_NUM];
    Snake* gaSnake[SEATS_NUM];
    int card[MONKEY_NUM];
    float bestDistance;
    Monkey* bestDriver;
    int bestGeneration;
    int bestRun;
    int runCount;

};

#endif
