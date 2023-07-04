#include <Ultrasonic.h>
#include <command_motors.h>
#include <capteurs_sm.h>

/*
Les pins entres de la classe Command commandent dans l'ordre:
-Moteur principal avant droit
-Moteur principal arriere droit
-Moteur principal arriere gauche
-Moteur principal avant gauche
-Moteur de profondeur avant droit
-Moteur de profondeur arriere droit
-Moteur de profondeur arriere gauche
-Moteur de profondeur avant gauche
*/

/*
Les pins entres de la classe capteurs connectent:
-le capteur d'humidite (pin analogique)
-l'emission du capteur ultrason (pin digital)
-la reception du capteur ultrason (pin digital)
-le capteur de tension (montage de resistances) (pin analogique)
*/
//La pression et l'imu communiquent en I2C (broches A4(SDA) et A5(SCL) pour Uno, broches 20(SDA) et 21(SCL))

command_motors Command(2, 3, 4, 5, 6, 7, 8, 9);
capteurs_sm Capteurs(A0, 1, 2, A1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
}

void loop() {
  // put your main code here, to run repeatedly:
  Command.get_command();
  Capteurs.get_and_send_data();
}
