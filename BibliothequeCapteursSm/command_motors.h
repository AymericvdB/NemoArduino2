#ifndef COMMAND_MOTORS_H
#define COMMAND_MOTORS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include <ArduinoJson.h>

struct Cmds {
    String moteur;
    int cmd;
};

class command_motors
{
    public:
        command_motors();
        command_motors(int pinMot1, int pinMot2, int pinMot3, int pinMot4, int pinMotProf1, int pinMotProf2, int pinMotProf3, int pinMotProf4);
        virtual ~command_motors();

        void initialisation(int ESCpin, Servo servo);

        void get_and_exec_command();
        void get_command();
        void exec_command();

        void cmd_N();
        void cmd_U();
        void cmd_D();

        void set_value(String mot, int value);
        int get_value(String mot);

    protected:

    private:

        int pinMot1; //Pin des moteurs de direction
        int pinMot2;
        int pinMot3;
        int pinMot4;

        int pinMotProf1; //Pin des moteurs de profondeur
        int pinMotProf2;
        int pinMotProf3;
        int pinMotProf4;

        Servo Mot1;
        Servo Mot2;
        Servo Mot3;
        Servo Mot4;

        Servo MotProf1;
        Servo MotProf2;
        Servo MotProf3;
        Servo MotProf4;

        //Liste des modes
        //A : mode automatique (par defaut)
        //M : mode manuel
        //U : mode urgence (quand activee, verification forte en amont, raspberry)
        //N : mode precedent conserve
        char mode[2];

        //Liste des types de message
        //A : cmd d'origine automatique (par defaut)
        //M : cmd d'origine manuelle
        char type[2];

        //Liste des options
        //N : pas d'options (par defaut)
        //D : retour des commandes (port serie) pour affichage ou log
        char option[2];

        //Dictionnaire pour enregistrer les commandes
        Cmds dico[8];

        //Variables JSON
        static constexpr const int capacity_got = 12 * JSON_OBJECT_SIZE(1);

        StaticJsonDocument<capacity_got> doc_got;

        int size_buffer = 120;
        int size_msg = 120;
};

#endif // COMMAND_MOTORS_H
