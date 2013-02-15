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
#include "Creature.h"

#define SEATS_NUM 4
#define MONKEY_NUM 15



class GA
{
public:
    GA(Creature *wCreature, World* wWorld);
    void init();
    void pickDrivers();
    int currentSeat;
    Monkey* monkeys[15];
    Monkey* seats[SEATS_NUM];
    Creature* gaCreature;
    World* gaWorld;
    void iterate(float time, float dt);
    void changeDrivers();
    void getDistance();
    void compareDistance();
    void breadMonkeys( int runN);
    void sortByDistance();
    float bestDistance;
    Monkey* bestDriver;
    int bestGeneration;
    int bestRun;
    int runCount;
    int iRand(int floor, int ceiling);
    int* shuffleCard();
    int card[MONKEY_NUM];
};

#endif
