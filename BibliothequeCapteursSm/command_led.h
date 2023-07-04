#ifndef COMMAND_LED_H
#define COMMAND_LED_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include <Servo.h>

class command_led
{
    public:
        command_led();
        command_led(int pinMot1, int pinMot2, int pinMot3, int pinMot4, int pinMotProf1, int pinMotProf2, int pinMotProf3, int pinMotProf4);
        virtual ~command_led();

        void initialisation(int ESCpin);
        void get_command();
        void send_manual_command(int cmd[]);

    protected:

    private:

        int pinMot1; //Pin des moteure de direction
        int pinMot2;
        int pinMot3;
        int pinMot4;

        int pinMotProf1; //Pin des moteure de profondeur
        int pinMotProf2;
        int pinMotProf3;
        int pinMotProf4;

        /*Servo Mot1;
        Servo Mot2;
        Servo Mot3;
        Servo Mot4;

        Servo MotProf1;
        Servo MotProf2;
        Servo MotProf3;
        Servo MotProf4;*/

        //int cmd_dep[];
};

#endif // COMMAND_LED_H