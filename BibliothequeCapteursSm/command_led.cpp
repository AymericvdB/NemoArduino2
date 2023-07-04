#include "command_led.h"

int cmd_dep_led[8] =  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

command_led::command_led()
{
    //ctor
}

command_led::command_led(int pinMot1, int pinMot2, int pinMot3, int pinMot4, int pinMotProf1, int pinMotProf2, int pinMotProf3, int pinMotProf4)
{
    this->pinMot1 = pinMot1; //Pin des moteure de direction
    this->pinMot2 = pinMot2;
    this->pinMot3 = pinMot3;
    this->pinMot4 = pinMot4;

    this->pinMotProf1 = pinMotProf1; //Pin des moteure de profondeur
    this->pinMotProf2 = pinMotProf2;
    this->pinMotProf3 = pinMotProf3;
    this->pinMotProf4 = pinMotProf4;

    initialisation(this->pinMot1);
    initialisation(this->pinMot2);
    initialisation(this->pinMot3);
    initialisation(this->pinMot4);

    initialisation(this->pinMotProf1);
    initialisation(this->pinMotProf2);
    initialisation(this->pinMotProf3);
    initialisation(this->pinMotProf4);

    //delay(3000);

    //cmd_dep[8] =  {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    //send_manual_command(cmd_dep);

    //send_manual_command({1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500});

    //delay(7000);
}

command_led::~command_led()
{
    //dtor
}

void command_led::initialisation(int ESCpin)
{
    digitalWrite(ESCpin, LOW);
}

void command_led::get_command()
{
    int cmd[8];

    union{
        int Consigne;
        unsigned char myChars[sizeof(double)];
    } data_byte;

    int j=0;

    while(j<1000)
    {
        if(Serial.readString() == "M")
        {
            for(int i=0; i<sizeof(cmd); i++)
            {
                cmd[i] = Serial.read();
            }

            digitalWrite(this->pinMot1, HIGH);
            digitalWrite(this->pinMot2, HIGH);
            digitalWrite(this->pinMot3, HIGH);
            digitalWrite(this->pinMot4, HIGH);

            digitalWrite(this->pinMotProf1, HIGH);
            digitalWrite(this->pinMotProf2, HIGH);
            digitalWrite(this->pinMotProf3, HIGH);
            digitalWrite(this->pinMotProf4, HIGH);

            j = 1000;
            delay(100);
        }
        else if(Serial.readString() == "U") //Arret d'urgence
        {

            digitalWrite(this->pinMot1, HIGH);
            digitalWrite(this->pinMot2, HIGH);
            digitalWrite(this->pinMot3, HIGH);
            digitalWrite(this->pinMot4, HIGH);

            digitalWrite(this->pinMotProf1, HIGH);
            digitalWrite(this->pinMotProf2, HIGH);
            digitalWrite(this->pinMotProf3, HIGH);
            digitalWrite(this->pinMotProf4, HIGH);
            j=1000;
        }
        else
        {
            j++;
        }


    }


}

void command_led::send_manual_command(int cmd[])
{
    digitalWrite(this->pinMot1, HIGH);
    digitalWrite(this->pinMot2, HIGH);
    digitalWrite(this->pinMot3, HIGH);
    digitalWrite(this->pinMot4, HIGH);

    digitalWrite(this->pinMotProf1, HIGH);
    digitalWrite(this->pinMotProf2, HIGH);
    digitalWrite(this->pinMotProf3, HIGH);
    digitalWrite(this->pinMotProf4, HIGH);
}