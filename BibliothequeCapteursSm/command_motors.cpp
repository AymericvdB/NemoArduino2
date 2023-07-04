#include "command_motors.h"

command_motors::command_motors()
{
    //ctor
}

command_motors::command_motors(int pinMot1, int pinMot2, int pinMot3, int pinMot4, int pinMotProf1, int pinMotProf2, int pinMotProf3, int pinMotProf4)
{
    this->pinMot1 = pinMot1; //Pin des moteure de direction
    this->pinMot2 = pinMot2;
    this->pinMot3 = pinMot3;
    this->pinMot4 = pinMot4;

    this->pinMotProf1 = pinMotProf1; //Pin des moteure de profondeur
    this->pinMotProf2 = pinMotProf2;
    this->pinMotProf3 = pinMotProf3;
    this->pinMotProf4 = pinMotProf4;

    initialisation(this->pinMot1, this->Mot1);
    initialisation(this->pinMot2, this->Mot2);
    initialisation(this->pinMot3, this->Mot3);
    initialisation(this->pinMot4, this->Mot4);

    initialisation(this->pinMotProf1, this->MotProf1);
    initialisation(this->pinMotProf2, this->MotProf2);
    initialisation(this->pinMotProf3, this->MotProf3);
    initialisation(this->pinMotProf4, this->MotProf4);

    //Delai necessaire pour que les moteurs s'initialisent (7 sec)
    delay(7000);

    dico[0].moteur = "M1";
    dico[0].cmd = 1500;
    dico[1].moteur = "M2";
    dico[1].cmd = 1500;
    dico[2].moteur = "M3";
    dico[2].cmd = 1500;
    dico[3].moteur = "M4";
    dico[3].cmd = 1500;
    dico[4].moteur = "MP1";
    dico[4].cmd = 1500;
    dico[5].moteur = "MP2";
    dico[5].cmd = 1500;
    dico[6].moteur = "MP3";
    dico[6].cmd = 1500;
    dico[7].moteur = "MP4";
    dico[7].cmd = 1500;

    //Initialisation mode et option
    mode[0] = 'A';
    mode[1] = '\0';

    type[0] = 'A';
    type[1] = '\0';

    option[0] = 'N';
    option[1] = '\0';
}

command_motors::~command_motors()
{
    //dtor
}

void command_motors::initialisation(int ESCpin, Servo servo)
{
    servo.attach(ESCpin);
    servo.writeMicroseconds(1500);//Arret du moteur
    //delay(7000); //Attente de 7 secondes
    //digitalWrite(ESCpin, LOW);
}

//Le dictionnaire enegistrant les commandes passees, celles-ci sont
//continuellement executees meme en cas d'interruption de communication
void command_motors::get_and_exec_command()
{
    get_command();
    exec_command();
}

//Reception et decodage de la commande en provenance de la Raspberry
void command_motors::get_command()
{
  char read_buf[size_buffer];
  char msg[size_msg];

  memset(read_buf, '\0', sizeof read_buf);
  memset(msg, '\0', sizeof msg);
  
  unsigned long dt = 0.0;
  unsigned long start = millis();

  //Attendre d'avoir le premier caractere du message (accolade car le message est un JSON) tout en ne bloquant pas indefinimement le programme
  //si le message n'arrive pas : duree max 100ms
  while(dt < 100)
  {
    if(Serial.available() > 0 && Serial.read() == '{')
    {
        int n = Serial.readBytesUntil('}', read_buf, 200);

        //Reconstruction du message (qui a perdu ses deux accolades), pour qu'il
        //puisse etre lu par la methode deserialize (qui n'accepte que les JSON valides)
        msg[0] = '{';
        for(int i=0; i<=n; i++)
        {
            msg[i+1] = read_buf[i];
        }
        msg[n+1] = '}';

        //D'apres la doc, doc_got est "cleared" avant de recuperer le nouveau JSON
        DeserializationError err = deserializeJson(doc_got, msg); //Deserialization avec obtention d'un objet lisible type map
        if (err) 
        {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.f_str());        
        }
        else
        {
            //Mise a jour mode et option
            String mode_s = doc_got["mode"].as<String>();
            String type_s = doc_got["type"].as<String>();
            String option_s = doc_got["op"].as<String>();

            mode[0] = mode_s[0];
            type[0] = type_s[0];
            option[0] = option_s[0];
            
            //Mise a jour des commandes
            if(mode[0] == type[0])
            {
                set_value("M1", doc_got["M1"].as<int>());
                set_value("M2", doc_got["M2"].as<int>());
                set_value("M3", doc_got["M3"].as<int>());
                set_value("M4", doc_got["M4"].as<int>());

                set_value("MP1", doc_got["MP1"].as<int>());
                set_value("MP2", doc_got["MP2"].as<int>());
                set_value("MP3", doc_got["MP3"].as<int>());
                set_value("MP4", doc_got["MP4"].as<int>());
            }
        }
      
      break; //Sortie anticipee du while une fois le msg recupere
    }
    else
    {
      dt = millis() - start;
    }
  }


}

//Selection de la commande en fonction du mode et des options
void command_motors::exec_command()
{
    switch (mode[0])
    {
        case 'A': cmd_N();
            break;
        case 'M': cmd_N();
            break;
        case 'U': cmd_U();
            break;
        default: cmd_U();
    }

    switch (option[0])
    {
        case 'N':
            break;
        case 'D': cmd_D();
            break;
    }
}

//Normal motor command
void command_motors::cmd_N()
{
    Mot1.writeMicroseconds(get_value("M1"));
    Mot2.writeMicroseconds(get_value("M2"));
    Mot3.writeMicroseconds(get_value("M3"));
    Mot4.writeMicroseconds(get_value("M4"));

    MotProf1.writeMicroseconds(get_value("MP1"));
    MotProf2.writeMicroseconds(get_value("MP2"));
    MotProf3.writeMicroseconds(get_value("MP3"));
    MotProf4.writeMicroseconds(get_value("MP4"));
}

//The motors are stopped (urgence mode, brief message)
void command_motors::cmd_U()
{
    Mot1.writeMicroseconds(1500);
    Mot2.writeMicroseconds(1500);
    Mot3.writeMicroseconds(1500);
    Mot4.writeMicroseconds(1500);

    MotProf1.writeMicroseconds(1500);
    MotProf2.writeMicroseconds(1500);
    MotProf3.writeMicroseconds(1500);
    MotProf4.writeMicroseconds(1500);
}

//Display of the commands received
void command_motors::cmd_D()
{
    Serial.print("mode : "); Serial.print(mode[0]);
    Serial.print("type : "); Serial.print(type[0]);
    Serial.print("option : "); Serial.println(option[0]);

    Serial.print("M1 = "); Serial.print(get_value("M1"));
    Serial.print(", M2 = "); Serial.print(get_value("M2"));
    Serial.print(", M3 = "); Serial.print(get_value("M3"));
    Serial.print(", M4 = "); Serial.print(get_value("M4"));
    Serial.print(", MP1 = "); Serial.print(get_value("MP1"));
    Serial.print(", MP2 = "); Serial.print(get_value("MP2"));
    Serial.print(", MP3 = "); Serial.print(get_value("MP3"));
    Serial.print(", MP4 = "); Serial.println(get_value("MP4"));
}

//Attribution de la valeur en fonction du String selectionne (comportement dictionnaire)
void command_motors::set_value(String mot, int value)
{
    for(int i=0; i<sizeof(dico); i++)
    {
        if(dico[i].moteur == mot)
            dico[i].cmd = value;
    }
}

//Recuperationen fonction du String selectionne (comportement dictionnaire)
int command_motors::get_value(String mot)
{
    int value = 1500;
    for(int i=0; i<sizeof(dico); i++)
    {
        if(dico[i].moteur == mot)
            value = dico[i].cmd;
    }
    return value;
}