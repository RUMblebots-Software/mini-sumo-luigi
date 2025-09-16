#pragma once
#include "SumoMvmt.hpp"

class Luigi : public SumoMvmt {

    public:

    /**
 * Motor driver pins
 * 
 * @param STBYpin - 1
 * @param PWMApin - 2
 * @param PWMBpin - 3
 * @param AIN1pin - 4
 * @param AIN2pin - 5
 * @param BIN1pin - 6
 * @param BIN2pin - 7
 * 
 * 
 * Sensor pins
 * @param RIGHT_SENSORpin - 8
 * @param RIGHT_ANGLE_SENSORpin - 9
 * @param RIGHT_FRONT_SENSORpin - 10
 * @param LEFT_FRONT_SENSORpin - 11
 * @param LEFT_ANGLE_SENSORpin - 12
 * @param LEFT_SENSORpin - 13
 * @param BACK_SENSORpin - 14
 * @param RIGHT_LINE_SENSORpin - 15
 * @param LEFT_LINE_SENSORpin - 16
 * @param BACK_LINE_SENSORpin - 17
 * 
 * 
 * 
 */
    Luigi(int STBYpin, int PWMApin, int PWMBpin, int AIN1pin, int AIN2pin, int BIN1pin, int BIN2pin, 
        uint8_t RIGHT_SENSORpin, uint8_t RIGHT_ANGLE_SENSORpin, uint8_t RIGHT_FRONT_SENSORpin, uint8_t LEFT_FRONT_SENSORpin, uint8_t LEFT_ANGLE_SENSORpin, 
        uint8_t LEFT_SENSORpin, uint8_t BACK_SENSORpin, uint8_t RIGHT_LINE_SENSORpin, uint8_t LEFT_LINE_SENSORpin, uint8_t BACK_LINE_SENSORpin) { 

        *STBY = STBYpin;
        *PWMA = PWMApin;
        *PWMB = PWMBpin;
        *AIN1 = AIN1pin;
        *AIN2 = AIN2pin;
        *BIN1 = BIN1pin;
        *BIN2 = BIN2pin;

        *RIGHT_SENSOR = RIGHT_SENSORpin;
        *RIGHT_ANGLE_SENSOR = RIGHT_ANGLE_SENSORpin;
        *RIGHT_FRONT_SENSOR = RIGHT_FRONT_SENSORpin;
        *LEFT_FRONT_SENSOR = LEFT_FRONT_SENSORpin;
        *LEFT_ANGLE_SENSOR = LEFT_ANGLE_SENSORpin;
        *LEFT_SENSOR = LEFT_SENSORpin;
        *BACK_SENSOR = BACK_SENSORpin;
        *RIGHT_LINE_SENSOR = RIGHT_LINE_SENSORpin;
        *LEFT_LINE_SENSOR = LEFT_LINE_SENSORpin;
        *BACK_LINE_SENSOR = BACK_LINE_SENSORpin;
    }
    /**
     * TODO: Implement the Luigi specific behaviors and his pins
     */

    
};