#pragma once

class MiniSumo{
    public:

    /**
     * Delcare pure virtual functions for MiniSumo class, 
     * these functions  are the foundation of MiniSumo robots
     */
    virtual void forward(int speed) = 0;
    virtual void reverse(int speed) = 0;
    virtual void right(int speed) = 0;
    virtual void left(int speed) = 0;
    virtual void rightForward(int speed) = 0;
    virtual void leftForward(int speed) = 0;
    virtual void stopMotors() = 0;


    
};