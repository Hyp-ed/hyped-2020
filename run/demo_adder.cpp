#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include <iostream>

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::Timer;
using hyped::utils::Logger;
using hyped::utils::System;

class Increment : public Thread {
  public:
  Increment(int* n);
  void run();
  Logger log_;
  int* n_;  
};

Increment::Increment(int* n)
  : log_(System::getLogger()),
    n_(n)
  {
  }

void Increment::run(){
  for(int i = 0; i < 100000; i++){
    ++(*n_);
    std::cout << *n_ << std::endl;
  }
  
}

int main(int argc, char* argv[]){
  hyped::utils::System::parseArgs(argc, argv);

  int number = 0;

  // std::cout << number << std::endl;

  Increment* t1 = new Increment(&number);
  Increment* t2 = new Increment(&number);
  Increment* t3 = new Increment(&number);
  Increment* t4 = new Increment(&number);
  Increment* t5 = new Increment(&number);
  Increment* t6 = new Increment(&number);
  // Increment* t7 = new Increment(&number);
  // Increment* t8 = new Increment(&number);

  t1->start();
  t2->start();
  t3->start();
  t4->start();
  t5->start();
  t6->start();
  // t7->start();
  // t8->start();

  // std::cout << number << std::endl;

  t1->join();
  t2->join();
  t3->join();
  t4->join();
  t5->join();
  t6->join();
  // t7->join();
  // t8->join();

  // std::cout << number << std::endl;

  delete t1;
  delete t2;
  delete t3;
  delete t4;
  delete t5;
  delete t6;
  // delete t7;
  // delete t8;

  return 0;
}