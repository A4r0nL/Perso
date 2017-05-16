#include <Ultrasonic.h>
#include <Timer.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <rgb_lcd.h>



/** [VICTORIEN] **/
#define AVANT 0
#define DROIT 1
#define ARRIERE 2
#define GAUCHE 3
#define INTERVAL_CAPTEURS 50

#define PAS_D_OBSTACLE 1
#define OBSTACLE_ALERTE 2
#define OBSTACLE_DANGER 3

Ultrasonic capteurs[4] = { Ultrasonic(A2),  Ultrasonic(A4),  Ultrasonic(A8), Ultrasonic(A12) };
int valeurCapteur[4] =   { 0, 0, 0, 0};
int ancienneValeurCapteur[4] =  { 0, 0, 0, 0 };
bool peutBouger[4] = { true, true, true, true };
Timer timerCapteurs;
int capteurUtilise = 0;

/** ALEXANDRE **/
#define CMD_MOVE 0
#define CMD_KLAXON 2
#define CMD_SPEED 3
#define CMD_MSG 4

char ssid[] = "dd-wrt"; // NOM DU ROUTEUR
char pass[] = "MOT_DE_PASSE"; // MOT DE PASSE WIFI
int status = WL_IDLE_STATUS;
unsigned int localPort = 2900;
unsigned int targetPort = 2914;
char packetBuffer[255];
char SensorsBuffer[] = "STI2D2100"; //-> [HEADER][PROJECT_ID][SENSOR_ID][STATE]-> [STI2D2][1][0][0]
WiFiUDP Udp;

/** JORDAN**/
rgb_lcd lcd;
#define BUZZER_PIN 17
#define BUTTON_PIN 15
#define PIN_BATTERIE 14

/** VALENTIN **/
#define MOVE_AV 0
#define MOVE_AV_D 1
#define MOVE_D 2
#define MOVE_AR_D 3
#define MOVE_AR 4
#define MOVE_AR_G 5
#define MOVE_G 6
#define MOVE_AV_G 7
#define STATIC 8
#define AV 1
#define AR 0
int VIT_L = 11;    int VIT_R = 3;
int DIR_L = 12;   int DIR_R = 13;
float Speed = 1;
int etat = STATIC;


void emission(void) /** [ALEXANDRE] **/
{
      
}

void reception(void)
{ 
    int taillePacket = Udp.parsePacket();
    if(taillePacket != 0)
    {
        Serial.print("Received packet of size ");
        Serial.println(taillePacket);
        Serial.print("From ");
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(", port ");
        Serial.println(Udp.remotePort());
    
        // read the packet into packetBufffer
        int len = Udp.read(packetBuffer, 255);
        if (len > 0) {
          packetBuffer[len] = 0;
        }
        Serial.println("Contents:");
        Serial.println(packetBuffer);

        //RECEPTION

        if(packetBuffer[0] == CMD_MOVE)
        {  
              int cmdDeplacement = packetBuffer[1];
              Serial.print("deplacement : ");
              Serial.println(cmdDeplacement);
              etat = cmdDeplacement;     
        }
        else if(packetBuffer[0] == CMD_MSG)
        {
              char buff[len];
              for(int i(0) ; i < len-1; i++)
                buff[i] = packetBuffer[i+1];
              buff[len] = 0;
              lcd.clear();
              lcd.setCursor(0,0);
              for(int i(0) ; i < len; i++)
              {
                  if(i == 16)
                    lcd.setCursor(0,1); 
                  lcd.print(buff[i]);  
              }
        }
        else if(packetBuffer[0] == CMD_KLAXON)
        {
              digitalWrite(BUZZER_PIN, (uint8_t) packetBuffer[1]);
              Serial.print("Klaxon : ");
              Serial.println((uint8_t) packetBuffer[1]); 
        }
        else if(packetBuffer[0] == CMD_SPEED)
        {
              Speed = (float) packetBuffer[1]/255.0f;
        }
        
    }
}

void verifierCapteurs(void) /** [VICTORIEN **/
{
    timerCapteurs.update();
    if(timerCapteurs.getTimer() > INTERVAL_CAPTEURS)
    {
        timerCapteurs.reset();
        int distance = capteurs[capteurUtilise].MeasureInCentimeters();
        peutBouger[capteurUtilise] = true;
        /** DEFINIR LES SEUILS **/
        if(capteurUtilise == AVANT || capteurUtilise == ARRIERE)
        {
          if(distance > 50) // PAS DE DANGER
              valeurCapteur[capteurUtilise] = PAS_D_OBSTACLE;
          else if(distance > 20) // ALERTE
              valeurCapteur[capteurUtilise] = OBSTACLE_ALERTE;
          else
          {
              valeurCapteur[capteurUtilise] = OBSTACLE_DANGER;
              peutBouger[capteurUtilise] = false;
            /*  if(capteurUtilise == AVANT)
                lcd.setRGB(*/
          }
        }
        else
        {
           if(distance > 20) // PAS DE DANGER
              valeurCapteur[capteurUtilise] = PAS_D_OBSTACLE;
          else if(distance > 10) // ALERTE
              valeurCapteur[capteurUtilise] = OBSTACLE_ALERTE;
          else
          {
              valeurCapteur[capteurUtilise] = OBSTACLE_DANGER;
              peutBouger[capteurUtilise] = false;
          }
        }
   
       //Serial.println();
        if(ancienneValeurCapteur[capteurUtilise] != valeurCapteur[capteurUtilise])
        {
             SensorsBuffer[7] = capteurUtilise+'0';
             SensorsBuffer[8] = valeurCapteur[capteurUtilise]+'0';
             Udp.beginPacket(Udp.remoteIP(), targetPort);
             Udp.write(SensorsBuffer);
             Udp.endPacket();
             ancienneValeurCapteur[capteurUtilise] =  valeurCapteur[capteurUtilise];
             Serial.print("Send :");
             Serial.println(SensorsBuffer);
        }
        /** CHANGEMENT DE CAPTEUR **/
        capteurUtilise += 1;
        if(capteurUtilise > 3)
            capteurUtilise = 0;
    }
}


void afficherMessage(char *message) /** [JORDAN] **/
{
    lcd.clear();
    lcd.print(message);
}

void motorisation(int etat) /** [VALENTIN] **/
{
    if((etat == MOVE_AV || etat == MOVE_AV_D ||
        etat == MOVE_AV_G) && peutBouger[AVANT] == false)
      etat = STATIC;
    else if((etat == MOVE_AV_D || etat == MOVE_AR_D ||
             etat == MOVE_D) && peutBouger[DROIT] == false)
      etat = STATIC;
    else if((etat == MOVE_AV_G || etat == MOVE_AR_G ||
            etat == MOVE_G) && peutBouger[GAUCHE] == false)
      etat = STATIC;
      if((etat == MOVE_AR || etat == MOVE_AR_D ||
          etat == MOVE_AR_G) && peutBouger[ARRIERE] == false)
      {
      etat = STATIC;
      }
    
    switch(etat) 
    {
    case STATIC : // off
       analogWrite(VIT_L ,0*Speed);      analogWrite(VIT_R ,0);
       break;
    case MOVE_AV : // avance
           analogWrite(VIT_L ,255*Speed);    analogWrite(VIT_R ,255*Speed);
           digitalWrite(DIR_L,AV);     digitalWrite(DIR_R, AV);
       break; 
    case MOVE_AV_D : // droite avant
      analogWrite(VIT_L, 255*Speed);     analogWrite (VIT_R,117*Speed);
      digitalWrite(DIR_L, AV);     digitalWrite(DIR_R, AV);
      break;
    case MOVE_D : // droite
      analogWrite(VIT_L ,255*Speed);     analogWrite (VIT_R,0);
      digitalWrite(DIR_L, AV);     digitalWrite(DIR_R,AV);
      break;
    case MOVE_AR_D : // droite arrière
      analogWrite(VIT_L,255*Speed);        analogWrite(VIT_R, 115*Speed);
      digitalWrite(DIR_L, AR);       digitalWrite(DIR_R, AR);
      break;
    case MOVE_AR : // recule
      analogWrite(VIT_L ,255*Speed);          analogWrite( VIT_R,255*Speed);
      digitalWrite(DIR_L, AR);          digitalWrite(DIR_R, AR);
      break;
    case MOVE_AR_G : // gauche arrière
      analogWrite(VIT_L ,115*Speed);        analogWrite(VIT_R ,255*Speed);
      digitalWrite(DIR_L, AR);             digitalWrite(DIR_R, AR); 
      break;
    case MOVE_G : // gauche
      analogWrite (VIT_L,0);        analogWrite (VIT_R,255*Speed);
      digitalWrite(DIR_L,AV);        digitalWrite(DIR_R,AV);
      break;  
    case MOVE_AV_G : // gauche avant
      analogWrite (VIT_L,115*Speed);      analogWrite (VIT_R,255*Speed);
      digitalWrite (VIT_L,AV);       digitalWrite (VIT_R,AV);
      break;
  }
}


void connecterWifi(void)
{
   if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      afficherMessage("Wifi shield introuvable");
      lcd.setRGB(255,0,0);
      while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
    afficherMessage("Shield wifi : mettre a jour");
    lcd.setRGB(255,0,0);
    while(1);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    afficherMessage("Connexion...");
    lcd.setRGB(225,115,11);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.print("You're connected to the network");
  lcd.clear();
  lcd.setRGB(0,255,0);
  afficherMessage("Connecte");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
}

void setup() 
{
    Serial.begin(9600);
    lcd.begin(16,2);
    
    pinMode(VIT_L,OUTPUT);          pinMode(VIT_R,OUTPUT);
    pinMode(DIR_L,OUTPUT);          pinMode(DIR_R,OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);    pinMode(BUTTON_PIN, INPUT);

    pinMode(PIN_BATTERIE, INPUT); 

    connecterWifi();
}

void loop() 
{
    verifierCapteurs();
    reception();
    motorisation(etat);
    if(digitalRead(BUTTON_PIN) == HIGH)
    {
        lcd.clear();
        float Volt = (float) analogRead(PIN_BATTERIE)/1023.0f*5.0f*2.0f;
        float pourcentage = (Volt-7)*100.0/1.5;
        lcd.setCursor(0,0);     lcd.print("Batterie : ");   lcd.print(pourcentage);    lcd.print("%");
        lcd.setCursor(0,1);     lcd.print(WiFi.localIP());
       
    }
}
