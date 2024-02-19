
//**CNC transformer winding machine**//
//**Sajy_ho**// 
//**(2022)**//

// Pins and variables:
  #include <LiquidCrystal.h>
  const int rs = 10, en = 9, d4 = 8, d5 = 7, d6 = 6, d7 = 5;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

  #define switchPin 2           //Rotary encoder Sw input pin
  #define pinB 3                //Rotary encoder Clk input pin
  #define pinA 4                //Rotary encoder Dt input pin
  #define motor1_STEP_PIN A2    //Main motor's Step pin
  #define motor1_DIR_PIN A1     //Main motor's Dir pin
  #define motor1_EN_PIN A0      //Main motor's En pin
  #define motor2_STEP_PIN A5    //Lead motor's Step pin
  #define motor2_DIR_PIN A4     //Lead motor's Dir pin
  #define motor2_EN_PIN A3      //Lead motor's En pin

  ///////////////////////////////////////////////////////////////
  //Hardware specs:
  #define motor1_SPR 400       //Main motor Steps per Revolution(Should not Exceed 400!)
  #define motor2_SPR 1600      //Lead motor Steps per Revolution
  #define lead 8               //Lead screw's "Lead" factor in mm
  ///////////////////////////////////////////////////////////////

  #include <AccelStepper.h>
  AccelStepper motor1(AccelStepper::DRIVER, motor1_STEP_PIN, motor1_DIR_PIN);
  AccelStepper motor2(AccelStepper::DRIVER, motor2_STEP_PIN, motor2_DIR_PIN);

  volatile bool switchStateCurrent = true;
  volatile bool switchStateLast = true;
  volatile bool switched = false;
  volatile bool pinAStateCurrent = false;
  volatile bool pinAStateLast = false;
  volatile bool rotationR = false;
  volatile bool rotationL = false;

  float RPM = 100;
  int Speed = RPM;
  float dia = 20;
  int Diameter = dia;
  long steps;
  long turns;
  long setTurns = 1;
  int prevTurns;
  float s1;
  float s2;
  float a1 = 400.0;
  float a2;
  float d1;
  float d2;
  bool direc;
  String direcString = "left";
  bool menuEnd;
  int8_t menuCounter;
  int8_t subMenu;
  String proceedString = "No";
  bool proceed;
//

//-----------------------------------------------------------
 
void switching(){
  
  switchStateCurrent = digitalRead(switchPin);
  if (!switchStateLast && switchStateCurrent) {
    switched = true; 
  }
  switchStateLast = switchStateCurrent; 
}

//----------------------------------------------------------

void rotation() {
  pinAStateCurrent = digitalRead(pinA);
  if (!pinAStateLast && pinAStateCurrent) {   
    if (digitalRead(pinB)) {
      rotationL = true;
    }else{
      rotationR = true;
    }   
  }
  pinAStateLast = pinAStateCurrent;
}

//----------------------------------------------------------

void menu(){

  if(subMenu==0){
    if(rotationR){
      menuCounter++;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      menuCounter--;
      rotationL = false; 
      lcd.clear();          
    }
  }
  
  //============================================== 
 //menu 1
 
  if(menuCounter==1 && subMenu==0){
    lcd.setCursor(0,0);
    lcd.print(">");
    lcd.setCursor(1,0);
    lcd.print("Turns");
    lcd.setCursor(8,0);
    lcd.print(setTurns);
    lcd.setCursor(1,1);
    lcd.print("Direc");
    lcd.setCursor(8,1);
    lcd.print(direcString);
    if(switched){
      subMenu = 1;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==1 && subMenu==1){
    lcd.setCursor(1,0);
    lcd.print("Turns");
    lcd.setCursor(7,0);
    lcd.print(">");
    lcd.setCursor(8,0);
    lcd.print(setTurns);
    lcd.setCursor(1,1);
    lcd.print("Direc");
    lcd.setCursor(8,1);
    lcd.print(direcString);
    if(rotationR){
      setTurns = setTurns + 10;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      setTurns = setTurns - 10;
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      subMenu = 2;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==1 && subMenu==2){
    lcd.setCursor(1,0);
    lcd.print("Turns");
    lcd.setCursor(8,0);
    lcd.print(setTurns);
    lcd.setCursor(11,0);
    lcd.print("<");
    lcd.setCursor(1,1);
    lcd.print("Direc");
    lcd.setCursor(8,1);
    lcd.print(direcString);
    if(rotationR){
      setTurns++;
      rotationR = false; 
      lcd.clear();           
    }
    else if(rotationL){
      setTurns--;
      rotationL = false;  
      lcd.clear();          
    }
    if(switched){
      subMenu = 0;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

 //============================================== 
 //menu 2
 
  if(menuCounter==2 && subMenu==0){
    lcd.setCursor(1,0);
    lcd.print("Turns");
    lcd.setCursor(8,0);
    lcd.print(setTurns);
    lcd.setCursor(0,1);
    lcd.print(">");
    lcd.setCursor(1,1);
    lcd.print("Direc");
    lcd.setCursor(8,1);
    lcd.print(direcString);
    if(switched){
      subMenu = 1;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==2 && subMenu==1){
    lcd.setCursor(1,0);
    lcd.print("Turns");
    lcd.setCursor(8,0);
    lcd.print(setTurns);
    lcd.setCursor(1,1);
    lcd.print("Direc");
    lcd.setCursor(7,1);
    lcd.print(">");
    lcd.setCursor(8,1);
    lcd.print(direcString);
    if(rotationR){
      direc = true;
      direcString = "Right";
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      direc = false;
      direcString = "Left";
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      subMenu = 0;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

 //============================================== 
 //menu 3
 
  if(menuCounter==3 && subMenu==0){
    lcd.setCursor(1,0);
    lcd.print("Direc");
    lcd.setCursor(8,0);
    lcd.print(direcString);
    lcd.setCursor(0,1);
    lcd.print(">");
    lcd.setCursor(1,1);
    lcd.print("Dia");
    lcd.setCursor(6,1);
    lcd.print(Diameter);
    lcd.setCursor(10,1);
    lcd.print("x.01mm");
    if(switched){
      subMenu = 1;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==3 && subMenu==1){
    lcd.setCursor(1,0);
    lcd.print("Direc");
    lcd.setCursor(8,0);
    lcd.print(direcString);
    lcd.setCursor(1,1);
    lcd.print("Dia");
    lcd.setCursor(5,1);
    lcd.print(">");
    lcd.setCursor(6,1);
    lcd.print(Diameter);
    lcd.setCursor(10,1);
    lcd.print("x.01mm");
    if(rotationR){
      dia = dia + 10;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      dia = dia - 10;
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      subMenu = 2;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==3 && subMenu==2){
    lcd.setCursor(1,0);
    lcd.print("Direc");
    lcd.setCursor(8,0);
    lcd.print(direcString);
    lcd.setCursor(1,1);
    lcd.print("Dia");
    lcd.setCursor(6,1);
    lcd.print(Diameter);
    lcd.setCursor(9,1);
    lcd.print("<");
    lcd.setCursor(10,1);
    lcd.print("x.01mm");
    if(rotationR){
      dia++;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      dia--;
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      subMenu = 0;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

 //============================================== 
 //menu 4
 
  if(menuCounter==4 && subMenu==0){
    lcd.setCursor(1,0);
    lcd.print("Dia");
    lcd.setCursor(6,0);
    lcd.print(Diameter);
    lcd.setCursor(10,0);
    lcd.print("x.01mm");
    lcd.setCursor(0,1);
    lcd.print(">");
    lcd.setCursor(1,1);
    lcd.print("Speed");
    lcd.setCursor(8,1);
    lcd.print(Speed);
    lcd.setCursor(13,1);
    lcd.print("RPM");
    if(switched){
      subMenu = 1;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==4 && subMenu==1){
    lcd.setCursor(1,0);
    lcd.print("Dia");
    lcd.setCursor(6,0);
    lcd.print(Diameter);
    lcd.setCursor(10,0);
    lcd.print("x.01mm");
    lcd.setCursor(1,1);
    lcd.print("Speed");
    lcd.setCursor(7,1);
    lcd.print(">");
    lcd.setCursor(8,1);
    lcd.print(Speed);
    lcd.setCursor(13,1);
    lcd.print("RPM");
    if(rotationR){
      RPM = RPM + 10;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      RPM = RPM - 10;
      rotationL = false; 
      lcd.clear();          
    }
    if(switched){
      subMenu = 2;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==4 && subMenu==2){
    lcd.setCursor(1,0);
    lcd.print("Dia");
    lcd.setCursor(6,0);
    lcd.print(Diameter);
    lcd.setCursor(10,0);
    lcd.print("x.01mm");
    lcd.setCursor(1,1);
    lcd.print("Speed");
    lcd.setCursor(8,1);
    lcd.print(Speed);
    lcd.setCursor(11,1);
    lcd.print("<");
    lcd.setCursor(13,1);
    lcd.print("RPM");
    if(rotationR){
      RPM++;
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      RPM--;
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      subMenu = 0;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

 //==============================================
 //menu 5
 
  if(menuCounter==5 && subMenu==0){
    lcd.setCursor(1,0);
    lcd.print("Speed");
    lcd.setCursor(8,0);
    lcd.print(Speed);
    lcd.setCursor(13,0);
    lcd.print("RPM");
    lcd.setCursor(0,1);
    lcd.print(">");
    lcd.setCursor(1,1);
    lcd.print("Proceed?");
    lcd.setCursor(11,1);
    lcd.print(proceedString);
    if(switched){
      subMenu = 1;
      delay(300);
      switched = false;
      lcd.clear();
    }
  }

  else if(menuCounter==5 && subMenu==1){
    lcd.setCursor(1,0);
    lcd.print("Speed");
    lcd.setCursor(8,0);
    lcd.print(Speed);
    lcd.setCursor(13,0);
    lcd.print("RPM");
    lcd.setCursor(1,1);
    lcd.print("Proceed?");
    lcd.setCursor(10,1);
    lcd.print(">");
    lcd.setCursor(11,1);
    lcd.print(proceedString);
    if(rotationR){
      proceed = true;
      proceedString = "Yes";
      rotationR = false;
      lcd.clear();           
    }
    else if(rotationL){
      proceed = false;
      proceedString = "No";
      rotationL = false;
      lcd.clear();           
    }
    if(switched){
      if(proceed){
        menuEnd = true;
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Turns:");
        lcd.setCursor(8, 0);
        lcd.print(turns);
        lcd.setCursor(0, 1);
        lcd.print("Direc:");
        lcd.setCursor(8, 1);
        lcd.print(direcString);
        
        motor1.disableOutputs();
        motor2.disableOutputs();
        s1 = (RPM * motor1_SPR)/60.;
        s2 = (RPM * dia / 6000.) * (motor2_SPR / lead);
        a2 = (a1 * dia / lead ) * ( motor2_SPR / (100. *  motor1_SPR));
        d1 = setTurns * motor1_SPR;
        d2 = d1 * s2 / s1;
        
        motor1.setMaxSpeed(s1);
        motor1.setAcceleration(a1);
        motor1.moveTo(d1); 
        motor2.setMaxSpeed(s2);
        motor2.setAcceleration(a2);
        if(!direc){
          motor2.moveTo(d2);
        }else{
          motor2.moveTo(-d2);
        }
      }
      subMenu = 0;
      delay(300);
      switched = false;
      if(!proceed){
        lcd.clear();
      }
    }
  }
  
 //==========================================

  if(menuCounter>5){
    menuCounter = 5;
  }

  else if(menuCounter<1){
    menuCounter = 1;
  }

  if(setTurns>999){
    setTurns = 1;
  }

  else if(setTurns<1){
    setTurns = 999;
  }

  if(dia>100){
    dia = 1;
  }

  else if(dia<1){
    dia = 100;
  }

  if(RPM>300){
    RPM = 1;
  }
  
  else if(RPM<1){
    RPM = 300;
  }

  Speed = RPM;
  Diameter = dia;
}

//----------------------------------------------------------

void setup() {
  lcd.begin(16, 2);
  motor1.setEnablePin(motor1_EN_PIN);
  motor2.setEnablePin(motor2_EN_PIN);
  pinMode (switchPin, INPUT_PULLUP);
  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(switchPin), switching, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), rotation, CHANGE);
  lcd.setCursor(0, 0);
  lcd.print("Winding Machine");
  lcd.setCursor(3, 1);
  lcd.print("-Sajad Ho-");
  delay(4000);
  lcd.clear();
}

//----------------------------------------------------------

void loop() {

  while(!menuEnd){
    menu();
  }

  motor1.run();
  motor2.run();
   
  steps = motor1.currentPosition();
  turns = steps / motor1_SPR;

  if(turns!=prevTurns){
    lcd.setCursor(8, 0);
    lcd.print("    ");
    lcd.setCursor(8, 0);
    lcd.print(turns);
    prevTurns = turns;
  }
  
  if(turns>=setTurns){
    menuEnd = false;
    while(!switched){
      if(!menuEnd){
        motor1.enableOutputs();
        motor2.enableOutputs();
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Finished!");
        lcd.setCursor(2, 1);
        lcd.print(turns);
        lcd.setCursor(8, 1);
        lcd.print("turns");
        menuEnd = true;
      }
    }
  }

  if(switched){
    motor1.enableOutputs();
    motor2.enableOutputs();
    delay(300);
    switched = false;
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Stopped!");
    lcd.setCursor(2, 1);
    lcd.print(turns);
    lcd.setCursor(8, 1);
    lcd.print("turns");
    while (!switched){
      delay(100);
    }
    menuEnd = false;
    menuCounter = 1;
    steps = 0;
    turns = 0;
    delay(300);
    switched = false;
    lcd.clear();
  }
}
