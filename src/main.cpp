#include "main.h"

float map_values(float x, float in_min, float in_max, float out_min, float out_max) {
    const float run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const float rise = out_max - out_min;
    const float delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void Encoders_Interrupt(void)
{
  byte next_state, table_input;

  // Encoder 1
  next_state     = digitalRead(ENC1_A) << 1;
  next_state    |= digitalRead(ENC1_B);
  table_input    = (encoder1_state << 2) | next_state;
  encoder1_pos  -= encoder_table[table_input];
  encoder1_state = next_state;

  // Encoder 2
  next_state     = digitalRead(ENC2_A) << 1;
  next_state    |= digitalRead(ENC2_B);
  table_input    = (encoder2_state << 2) | next_state;
  encoder2_pos  += encoder_table[table_input];
  encoder2_state = next_state;

  counterPID += 1;
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  Encoders_Interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void tuningSetupTurn()
{
  // Creats array with tuning setpoints for turn movement
  for (int c = 0; c < SETPOINT_VALUES_TURN; c++)
  {
    // map(value, fromLow, from High, toLow, toHigh)
    setpoint_values_turn[c] = map(c,0,SETPOINT_VALUES_TURN-1,setpoint_turn_min,setpoint_turn_max); 
  } 
  // Sets default value of tune counter in the middle of the number of possible tuning setpoints
  tune_counter_turn = SETPOINT_VALUES_TURN/2;
  
  // Sets default value of SETPOINT TURN in the middle of the number of possible tuning setpoints
  Setpoint_t = SETPOINT_TURN + setpoint_values_turn[tune_counter_turn];
  //Test
  Serial.print("Adding values for tunning setpoint turn:");
  for (int c= 0; c < SETPOINT_VALUES_TURN; c++){
    Serial.print(c);
    Serial.print(":");
    Serial.print(setpoint_values_turn[c]);
    Serial.print("|");
  }
  //Test
  Serial.println();
  Serial.print("Setpoint turn:");
  Serial.println(Setpoint_t);
}

void tuningSetupMove()
{
  // Creats array with tuning power of right wheel 
  for (int c = 0; c < POWER_VALUES_RUN; c++)
  {
    // map(value, fromLow, from High, toLow, toHigh)
    shift_values_move[c] = map_values(c,0,POWER_VALUES_RUN-1,shift_move_min,shift_move_max); 
  }
  // Sets default value of tune counter in the middle of the number of possible tuning setpoints
  tune_counter_move = POWER_VALUES_RUN/2;
  //@ Add to setpoint the additional value
  Shift_tunning_power = shift_values_move[tune_counter_move]; 
  //Test
  Serial.print("Adding values for tunning setpoint forward:");
  for (int c= 0; c < POWER_VALUES_RUN; c++){
    Serial.print(c);
    Serial.print(":");
    Serial.print(shift_values_move[c]);
    Serial.print("|");
  }
  Serial.println();
  Serial.print("Shift tunning power for run:");
  Serial.println(Shift_tunning_power); 
}

void startTimer()
{
  if (timer == NULL) {
    timer = timerBegin(2, 4000, true);
    timerAttachInterrupt(timer, &onTimer, true);
  }
  timerAlarmWrite(timer, 1, true);
  timerAlarmEnable(timer);
}

void stopTimer()
{
  if (timer != NULL) {
    timerAlarmDisable(timer);
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer        = NULL;
    encoder1_pos = 0;
    encoder2_pos = 0;
  }
}

void showBitmap(unsigned char bitmap_oled[]) {
  display.clearDisplay();
 
  display.drawBitmap(0, 0, bitmap_oled, bitmap_height, bitmap_width, WHITE);
  display.display();
}

void setLed(int r, int g, int b) // function to set NEO Pixel LED
{
  DEBUG_PRINTLN_FCT("exc setLED fct"); // debug print
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(r, g, b)); // set pixel color
  pixels.show(); // show pixel color
}

void read_direction_buttons() // function to read direction buttons
{
  DEBUG_PRINTLN_FCT("exc read_direction_buttons fct"); //debug print
  button_left.loop(); //read left button

  if (button_left.isPressed()) {   //check if left button is pressed
    mov = TURN_LEFT; //set mov to turn left
    DEBUG_PRINTLN_ACT("button left is pressed"); // debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      //@Sequence
      if (nr_comm > 0){
        //if previous command is equal to actual command...
        if (recorded_button[nr_comm - 1] == mov){
          //increase the value of sequence in the sequence...
          sequence_rec_button[nr_comm] = sequence_rec_button[nr_comm - 1] + 1;
        }
        //Keep the number 1, first time command
        else sequence_rec_button[nr_comm] = 1;
      }
      else{
        //First time sequence
        sequence_rec_button[nr_comm] = 1;
      }
      //@End sequence
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
  button_forwards.loop(); //read forwards button

  if (button_forwards.isPressed()) { //check if forwards button is pressed
    mov = FORWARD; //set mov to forward
    DEBUG_PRINTLN_ACT("button forward is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      //@Sequence
      if (nr_comm > 0){
        //if previous command is equal to actual command...
        if (recorded_button[nr_comm - 1] == mov){
          //increase the value of sequence in the sequence...
          sequence_rec_button[nr_comm] = sequence_rec_button[nr_comm - 1] + 1;
        }
        //Keep the number 1, first time command
        else sequence_rec_button[nr_comm] = 1;
      }
      else{
        //First time sequence
        sequence_rec_button[nr_comm] = 1;
      }
      //@End sequence
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
  button_right.loop(); //read right button

  if (button_right.isPressed()) { //check if right button is pressed
    mov = TURN_RIGHT; //set mov to turn right
    DEBUG_PRINTLN_ACT("button right is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      //@Sequence
      if (nr_comm > 0){
        //if previous command is equal to actual command...
        if (recorded_button[nr_comm - 1] == mov){
          //increase the value of sequence in the sequence...
          sequence_rec_button[nr_comm] = sequence_rec_button[nr_comm - 1] + 1;
        }
        //Keep the number 1, first time command
        else sequence_rec_button[nr_comm] = 1;
      }
      else{
        //First time sequence
        sequence_rec_button[nr_comm] = 1;
      }
      //@End sequence
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0
    button_stop.resetCount(); 
  }
  button_backwards.loop(); //read backwards button

  if (button_backwards.isPressed()) { //check if backwards button is pressed
    mov = BACKWARD; //set mov to backward
    DEBUG_PRINTLN_ACT("button backwards is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      //@Sequence
      if (nr_comm > 0){
        //if previous command is equal to actual command...
        if (recorded_button[nr_comm - 1] == mov){
          //increase the value of sequence in the sequence...
          sequence_rec_button[nr_comm] = sequence_rec_button[nr_comm - 1] + 1;
        }
        //Keep the number 1, first time command
        else sequence_rec_button[nr_comm] = 1;
      }
      else{
        //First time sequence
        sequence_rec_button[nr_comm] = 1;
      }
      //@End sequence

      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }

  button_stop.loop(); //read backwards button

  if (button_stop.isPressed()) { //check if backwards button is pressed
    mov = WAIT; //set mov to backward
    DEBUG_PRINTLN_ACT("button wait is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      //@Sequence
      if (nr_comm > 0){
        //if previous command is equal to actual command...
        if (recorded_button[nr_comm - 1] == mov){
          //increase the value of sequence in the sequence...
          sequence_rec_button[nr_comm] = sequence_rec_button[nr_comm - 1] + 1;
        }
        //Keep the number 1, first time command
        else sequence_rec_button[nr_comm] = 1;
      }
      else{
        //First time sequence
        sequence_rec_button[nr_comm] = 1;
      }
      //@End sequence
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
} // read_cmd_buttons

void init(void) // function to init the the robo
{
  DEBUG_PRINTLN_FCT("exc init fct");
  setLed(0, 0, 255); // set LED to blue

  if (button_command_count > 3) { // reset command button counter if command button is pressed more then one time
    button_command.resetCount();
  } // reset

  if (button_command_count == 1) {  // if command button is pressed once
    // Initialize state
    nr_comm = 0;                                         // start the command reading
    memset(recorded_button, 0, sizeof(recorded_button)); // initialize to zero the commands vector
    memset(sequence_rec_button,0,sizeof(sequence_rec_button)); //bis
    machine_state = READ_COMM_ST; // set machine state to read comm
  }

  if (button_command_count == 0) {
    on_execute_comm_st = 0;
  }

  if (button_stop_count == 1) //switch to tune state
  {
    setLed(255,255,255); // set LED to white
    machine_state = TUNE_ST;
    
  }

}

void readComm(void) // funciton to read movement commands
{
  DEBUG_PRINTLN_FCT("exc readComm fct"); // debug print
  setLed(255, 255, 0);               // set LED to yellow

  if (nr_comm < MAX_NR_COMMANDS) { // it only keeps the first max_nr_commands...
    read_direction_buttons(); // call read dir func
    
  }

  // -- wait for the button_command_count = 2 and nr_commands != 0
  if (button_command_count == 2 and nr_comm != 0) {
    machine_state = START_EXEC_ST;
    //Testing the sequence of commands with the number of sequence of comm. exec.
    #ifdef DEBUG_SEQ_COMMANDS
      Serial.print("(Command,Sequence):");
      for (int i=0;i<nr_comm;i++){
        Serial.print("(");
        Serial.print(recorded_button[i]);
        Serial.print(",");
        Serial.print(sequence_rec_button[i]);
        Serial.print(")");
      }
      Serial.println();
    #endif

  }

  if (button_command_count >= 2 && nr_comm == 0) { // reset command button counter if command button is pressed more then one time
    button_command.resetCount();
    machine_state= INIT_ST;
  } // reset
 
}

void startExec(void) // function to start execution of commands
{
  DEBUG_PRINTLN_FCT("exc startExec fct"); // debug print
  setLed(0, 255, 0); // set LED to green
  #ifndef DEBUG_VISUAL_MODE
  showBitmap(image_data_EYES_MIDDLE);
  #endif

  button_forwards.loop(); 

  if (button_command_count > 2) {
    if (on_execute_comm_st == 1) {
      button_command.resetCount();
    }
    button_stop_count = 0; // reset button stop counter before going into INIT_ST 
    button_stop.resetCount(); // reset button stop
    button_command_count = 0; // reset button stop counter before going into INIT_ST 
    button_command.resetCount(); // reset button stop
    machine_state = INIT_ST; // set machine state
  }

  if (button_forwards.isPressed()) { // check if button forward is pressed
    comm_index         = nr_comm; // set comm_index to number of commands
    on_execute_comm_st = 1; // executed at least once ... needed???
    machine_state      = EXEC_ST; // set machine state to exectute_state
  }
}

void stopExec(void){
  button_stop.loop(); 
  if (button_stop.isPressed()){ // check if stop button is pressed, if yes stop current run of commands and go back to START_EXEC_ST
    MotorControl.motorsStop(); // stop motors
    setLed(255, 0, 0); // set led to red

    comm_index = 0; // set comm index to 0 to restart at command 0 on the next run
    
    #ifndef DEBUG_VISUAL_MODE
    showBitmap(image_data_DISTRESSED_EYES);
    #endif
    stop_melody();

    button_forwards.loop();
    if (button_forwards.isReleased()) { // wait till button releases state
      machine_state = START_EXEC_ST;
    }
  }
}

void exec(void) // function to execute the movement commands
{
  DEBUG_PRINTLN_FCT("exc exec fct"); // debug print
  comm_index--; // comm index -1

  if (comm_index >= 0) { // avoid getting nonsense data
    setLed(255, 51, 255);               // set led to pink
    int action = recorded_button[(nr_comm - 1) - comm_index];  // get current action
    int seq_action = sequence_rec_button[(nr_comm - 1) - comm_index];
    #ifdef DEBUG_SEQ_COMMANDS
      Serial.print("Action|Seq. Action:");
      Serial.print(action);
      Serial.print("|");
      Serial.println(seq_action);
    #endif
    if (action == FORWARD) {  //set state to execute movement action
      //@Initialize PID for run
      //Setpoint_r = SETPOINT_RUN;     
      pidleft_r.Initialize();
      pidright_r.Initialize();
      //@Initialize PID for difference:
      // Only initialize PID if it is the first time 
      // the action FORWARD is executed. 
      if (seq_action == 1) { 
        //Test
        Serial.println("Initialize pid_d");
        pid_d.Initialize();
      }
      #ifdef DEBUG_SEQ_COMMANDS
      Serial.print("FORWARD sequence command number:");
      Serial.println(seq_action);
      #endif
      machine_state = FORWARD_ST; 
    } else if (action == BACKWARD) {
      //@Initialize PID for run
      //Setpoint_r = SETPOINT_RUN;     
      pidleft_r.Initialize();
      pidright_r.Initialize();
      pid_d.Initialize();
      machine_state = BACK_ST;
    } else if (action == TURN_LEFT) {
      //@Initialize PID for turn
      pidleft_t.Initialize();
      pidright_t.Initialize();
      //Set the turn setpoint
      //Setpoint_t = SETPOINT_TURN;
      machine_state = TURN_LEFT_ST;
    } else if (action == TURN_RIGHT) {
      //@Execute a first reading of PID (?):
      pidleft_t.Initialize();
      pidright_t.Initialize();
      //Set the turn setpoint
      //Setpoint_t = SETPOINT_TURN;
      machine_state = TURN_RIGHT_ST;
      } else if (action == WAIT) {
      machine_state = WAIT_ST;
    }
  }

  if (comm_index < 0) {             // no more commands
    finish_melody();
    button_forwards.loop(); 
  
    if (button_forwards.isReleased()) { // wait till button releases state
      machine_state = START_EXEC_ST;
    }
  }
}

void turnRight(void) // function to turn right
{
  DEBUG_PRINTLN_FCT("exc turnRight fct");
  DEBUG_PRINTLN_ACT("turn right");
  #ifndef DEBUG_VISUAL_MODE 
  showBitmap(image_data_EYES_LEFT);
  #endif
  if ((abs(encoder1_pos) < Setpoint_t) &&
      (abs(encoder2_pos) < Setpoint_t))
  {
    startTimer();

    //int vel = kspeed * (turnspeedL + val_outputL) + setpoint_straight_run; // setpoint_straight_run makes sure robo turns accurate
    //int ver = kspeed * (turnspeedR + val_outputR) - setpoint_straight_run;
    int vel =  val_outputL;
    int ver =  val_outputR;
    //int ver = 50;
    //int vel = 50;
    //@ Verify if value is less than 20
    if (vel < 20) { vel = 20;}
    if (ver < 20) { ver = 20;}
  //@ Changed the pins
    MotorControl.motorReverse(1, ver);
    MotorControl.motorForward(0, vel);
    
    #ifdef DEBUG_TURN_OUTPUT
      Serial.print("Setpoint_turn:");
      Serial.println(Setpoint_t);
      Serial.print("P left:");
      Serial.print(pidleft_t.GetP()*kpL_t);
      Serial.print("| P right:");
      Serial.println(pidright_t.GetP()*kpR_t);
      Serial.print("I left:");
      Serial.print(pidleft_t.GetI()*ki_t);
      Serial.print("| I right:");
      Serial.println(pidright_t.GetI()*ki_t);      
      Serial.print("D left:");
      Serial.print(pidleft_t.GetD()*kd_t);
      Serial.print("| D right:");
      Serial.println(pidright_t.GetD()*kd_t);      
    
      Serial.print("val_outputL:");
      Serial.print(val_outputL);
      Serial.print("| val_outputR:");
      Serial.println(val_outputR);
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      Serial.print("Vel. Left:");      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);

      Serial.print(vel);
      Serial.print("| Vel. Right:");
      Serial.println(ver);
    #endif

    if (counterPID > freq) { //50
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = abs(encoder1_pos);
      enc_readR = abs(encoder2_pos);
      /*
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      */
      pidleft_t.Compute();
      pidright_t.Compute();
    }    

  } else {
    // Test
    Serial.print("Left encoder:");
    Serial.print(encoder1_pos);
    Serial.print("| Right encoder:");
    Serial.println(encoder2_pos);

    stopTimer();
    time_now = millis();
 
    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
  stopExec(); // stop current execution
}

void turnLeft(void) // function to turn left
{
  DEBUG_PRINTLN_FCT("exc turnLeft fct");
  DEBUG_PRINTLN_ACT("turn left");
  #ifndef DEBUG_VISUAL_MODE
  showBitmap(image_data_EYES_RIGHT);
  #endif
  if ((abs(encoder1_pos) < Setpoint_t) &&
      (abs(encoder2_pos) < Setpoint_t))
  {
    startTimer();
    //int vel = kspeed * (turnspeedL + val_outputL) + setpoint_straight_run; // setpoint_straight_run makes sure robo turns accurate
    //int ver = kspeed * (turnspeedR + val_outputR) - setpoint_straight_run;
    //@Using PID
    int vel = val_outputL;
    int ver = val_outputR;
    //int ver = 50;
    //int vel = 50;
    //@ Verify if value is less than 20
    if (vel < 20) { vel = 20;}
    if (ver < 20) { ver = 20;}

    // @Changed the pins
    MotorControl.motorReverse(0, vel);
    MotorControl.motorForward(1, ver);
    #ifdef DEBUG_TURN_OUTPUT
      Serial.print("Setpoint_turn:");
      Serial.println(Setpoint_t);
      Serial.print("P left:");
      Serial.print(pidleft_t.GetP()*kpL_t);
      Serial.print("| P right:");
      Serial.println(pidright_t.GetP()*kpR_t);
      Serial.print("I left:");
      Serial.print(pidleft_t.GetI()*ki_t);
      Serial.print("| I right:");
      Serial.println(pidright_t.GetI()*ki_t);      
      Serial.print("D left:");
      Serial.print(pidleft_t.GetD()*kd_t);
      Serial.print("| D right:");
      Serial.println(pidright_t.GetD()*kd_t);      
    
      Serial.print("val_outputL:");
      Serial.print(val_outputL);
      Serial.print("| val_outputR:");
      Serial.println(val_outputR);
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      Serial.print("Vel. Left:");
      Serial.print(vel);
      Serial.print("| Vel. Right:");
      Serial.println(ver);
    #endif




    if (counterPID > freq) { //50
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = abs(encoder1_pos);
      enc_readR = abs(encoder2_pos);
      pidleft_t.Compute();
      pidright_t.Compute();
    }
  } else {
    // Test
    Serial.print("Left encoder:");
    Serial.print(encoder1_pos);
    Serial.print("| Right encoder:");
    Serial.println(encoder2_pos);
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  
  }
  stopExec(); // stop current execution
}

/*
Testing Forward
Notes
setpoint_straight_run is an additional value that is added to modify the distance the robot moves.
But it must be added to SETPOINT_RUN instead to of adding to the velocity. We want to change the
distance and not the velocity of the robot.
*/
void forward(void) // function to drive forwards
{

  DEBUG_PRINTLN_FCT("exc forward fct");
  DEBUG_PRINTLN_ACT("drive forward");
  #ifndef DEBUG_VISUAL_MODE
    showBitmap(image_data_EYES_DOWN);
  #endif
  //@ Eventually we will change the code to the following:
  //if ((abs(encoder1_pos) < SETPOINT_RUN + setpoint_straight_run) &&
  //    (abs(encoder2_pos) < SETPOINT_RUN + setpoint_straight_run)) {
  //
  if ((abs(encoder1_pos) < Setpoint_r) &&
      (abs(encoder2_pos) < Setpoint_r)) {
    startTimer();

    //int vel = kspeed * (speedL + val_outputL) + setpoint_straight_run; // setpoint_straight_run -> make sure robo goes straight
    //int ver = kspeed * (speedR + val_outputR) - setpoint_straight_run;
    //@
    //int vel = kspeed * (speedL + val_outputL); 
    //int ver = kspeed * (speedR + val_outputR);
    //@Using PID
    //int vel = shift_powerL + val_outputL;
    //int ver = shift_powerR + val_outputR;
    //@Constant without PID and with shift (initial shift and tunning shift) and
    // the val_output, output value from the difference PID
    int vel = power_base + shift_powerL;
    int ver = power_base + shift_powerR  + Shift_tunning_power + val_output;
    //if (vel > 50){vel = 50;}
    //if (ver > 50){ver = 50;}
    
    //Testing the values of velocity:
    #ifdef DEBUG_FORWARD_OUTPUT
      Serial.print("P left:");
      Serial.print(pidleft_r.GetP()*kpL_r);
      Serial.print("| P right:");
      Serial.println(pidright_r.GetP()*kpR_r);
      Serial.print("I left:");
      Serial.print(pidleft_r.GetI()*ki_r);
      Serial.print("| I right:");
      Serial.println(pidright_r.GetI()*ki_r);      
      Serial.print("D left:");
      Serial.print(pidleft_r.GetD()*kd_r);
      Serial.print("| D right:");
      Serial.println(pidright_r.GetD()*kd_r);      
    
      Serial.print("val_outputL:");
      Serial.print(val_outputL);
      Serial.print("| val_outputR:");
      Serial.println(val_outputR);
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      Serial.print("Vel. Left:");
      Serial.print(vel);
      Serial.print("| Vel. Right:");
      Serial.println(ver);
    #endif
    //@ Why reverse?
    //MotorControl.motorReverse(0, vel);
    //MotorControl.motorReverse(1, ver);
  
  
    MotorControl.motorForward(0, vel);  
    MotorControl.motorForward(1, ver);
    
  
    if (counterPID > freq) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);

      #ifdef COUNTING_ENCODERS
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      Serial.print("Vel. Left:");
      Serial.print(vel);
      Serial.print("| Vel. Right:");
      Serial.println(ver);
      Serial.print("diff power:");
      Serial.println(val_output);
      Serial.print("kp*P:");
      Serial.print(kp_d*pid_d.GetP());
      Serial.print("|I:");
      Serial.print(ki_d*pid_d.GetI());
      Serial.print("|kd*D:");
      Serial.print(kd_d*pid_d.GetD());
      Serial.println();

      #endif
      enc_readL = abs(encoder1_pos);
      enc_readR = abs(encoder2_pos);
      // Diff between encoders
      enc_read = enc_readR - enc_readL;
      //Test
      Serial.print("Enc_read:");
      Serial.println(enc_read);
      //pidleft_r.Compute();
      //pidright_r.Compute();
      pid_d.Compute(); //Compute PID difference
      //Serial.println("PID compute");
      //Test
      Serial.print("PID output:");
      Serial.println(val_output);
    
      #ifdef DEBUG_VISUAL_MODE
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.clearDisplay();
          // Adjusting Powerdisplay.setCursor(10, 10);
          display.print("Right Encd: ");
          display.println(encoder2_pos);
          display.setCursor(10, 20);
          display.print("Left Encd: ");
          display.print(encoder1_pos);
          display.display();
      #endif

    }
  
  } else {    
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;

  }
  stopExec(); // stop current execution
}

void back(void) // function to drive backwards
{
  DEBUG_PRINTLN_FCT("exc back fct");
  DEBUG_PRINTLN_ACT("drive back");
  #ifndef DEBUG_VISUAL_MODE 
  showBitmap(image_data_EYES_UP);
  #endif
  if ((abs(encoder1_pos) < Setpoint_r) &&
      (abs(encoder2_pos) < Setpoint_r)) {
    startTimer();

    //int vel = kspeed * (speedL + val_outputL) + setpoint_straight_run; // setpoint_straight_run -> make sure robo goes straight
    //int ver = kspeed * (speedR + val_outputR) - setpoint_straight_run;
    //@ Changed the pins to have reverse in back:
    int vel = power_base + shift_powerL;
    int ver = power_base + shift_powerR  + Shift_tunning_power + val_output;


    //int vel = shift_powerL + val_outputL;
    //int ver = shift_powerR + val_outputR;
   //Testing the values of velocity:
    #ifdef DEBUG_BACKWARD_OUTPUT
      Serial.print("P left:");
      Serial.print(pidleft_r.GetP()*kpL_r);
      Serial.print("| P right:");
      Serial.println(pidright_r.GetP()*kpR_r);
      Serial.print("I left:");
      Serial.print(pidleft_r.GetI()*ki_r);
      Serial.print("| I right:");
      Serial.println(pidright_r.GetI()*ki_r);      
      Serial.print("D left:");
      Serial.print(pidleft_r.GetD()*kd_r);
      Serial.print("| D right:");
      Serial.println(pidright_r.GetD()*kd_r);      
    
      Serial.print("val_outputL:");
      Serial.print(val_outputL);
      Serial.print("| val_outputR:");
      Serial.println(val_outputR);
      Serial.print("Left encoder:");
      Serial.print(encoder1_pos);
      Serial.print("| Right encoder:");
      Serial.println(encoder2_pos);
      Serial.print("Vel. Left:");
      Serial.print(vel);
      Serial.print("| Vel. Right:");
      Serial.println(ver);
    #endif
   
    MotorControl.motorReverse(0, vel);
    MotorControl.motorReverse(1, ver);

    if (counterPID > freq) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = abs(encoder1_pos);
      enc_readR = abs(encoder2_pos);
      // Diff between encoders
      enc_read = enc_readR - enc_readL;
      //pidleft_r.Compute();
      //pidright_r.Compute();
      pid_d.Compute(); //Compute PID difference
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
  stopExec(); // stop current execution
}

void wait(void) // function to wait
{
  DEBUG_PRINTLN_FCT("exc wait fct");
  DEBUG_PRINTLN_ACT("wait");
  #ifndef DEBUG_VISUAL_MODE
  showBitmap(image_data__EYES_BLANK);
  #endif
  if (reset_time_wait){
    time_wait = millis();
    reset_time_wait = 0;
  }

  if (millis() >= time_wait + WAIT_DELAY){
    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
    time_now = millis();
    reset_time_wait = 1;
  }

  stopExec(); // stop current execution 
}

void tune()
{
  DEBUG_PRINTLN_FCT("exc read_tune_buttons fct"); //debug print
  display.setTextSize(1);
  display.setTextColor(WHITE);
  button_right.loop(); //read right button

  if (button_right.isPressed()) {   //check if right button is pressed
    if (tune_counter_turn+1 < SETPOINT_VALUES_TURN) 
    {
      tune_counter_turn++; //add 1 to the tune_counter 
      DEBUG_PRINT_ACT("Tune Counter: ");
      DEBUG_PRINTLN_ACT(tune_counter_turn);
      
      Setpoint_t = SETPOINT_TURN + setpoint_values_turn[tune_counter_turn];
      DEBUG_PRINT_ACT("New Setpoint Value: ");
      DEBUG_PRINTLN_ACT(Setpoint_t);
      tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
      DEBUG_PRINTLN_ACT("button right is pressed"); // debug print
      int brightness = map(tune_counter_turn,0,SETPOINT_VALUES_TURN,0,255);
      pixels.setBrightness(brightness); // adjust brightness of LED for optical user feedback
      setLed(125, 255, 0);               // set LED to green
     }
  }
   
  button_left.loop(); //read left button

  if (button_left.isPressed()) {   //check if left button is pressed
    if (tune_counter_turn >=1 && tune_counter_turn <= SETPOINT_VALUES_TURN)
    {
      tune_counter_turn--; //subtract 1 from the tune_counter 
      DEBUG_PRINT_ACT("Tune Counter: ");
      DEBUG_PRINTLN_ACT(tune_counter_turn);
      Setpoint_t = SETPOINT_TURN + setpoint_values_turn[tune_counter_turn];
      DEBUG_PRINT_ACT("New Setpoint Value: ");
      DEBUG_PRINTLN_ACT(Setpoint_t);
      tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
      DEBUG_PRINTLN_ACT("button left is pressed"); // debug print
      int brightness = map(tune_counter_turn,0,SETPOINT_VALUES_TURN,0,255);
      pixels.setBrightness(brightness); // adjust brightness of LED for optical user feedback
      setLed(125, 255, 0);               // set LED to green
    }
  }

  button_forwards.loop(); //read forward button

  if (button_forwards.isPressed()) {   //check if right button is pressed
    if (tune_counter_move+1 < POWER_VALUES_RUN) 
    {
      tune_counter_move++; //subtract 1 from the tune_counter 
      DEBUG_PRINT_ACT("Tune Counter: ");
      DEBUG_PRINTLN_ACT(tune_counter_move);
      
      Shift_tunning_power = shift_values_move[tune_counter_move];
      DEBUG_PRINT_ACT("New ballance power value: ");
      DEBUG_PRINTLN_ACT(Shift_tunning_power);
      tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
      DEBUG_PRINTLN_ACT("button forwards is pressed"); // debug print
      int brightness = map(tune_counter_move,0,POWER_VALUES_RUN,0,255);
      pixels.setBrightness(brightness); // adjust brightness of LED for optical user feedback
      setLed(165,42,42);               // set LED to 

     }
  }
   
  button_backwards.loop(); //read forward button

  if (button_backwards.isPressed()) {   //check if right button is pressed
    if (tune_counter_move >=1 && tune_counter_move <= POWER_VALUES_RUN)
    {
      tune_counter_move--; //add 1 to the tune_counter 
      DEBUG_PRINT_ACT("Tune Counter: ");
      DEBUG_PRINTLN_ACT(tune_counter_move);
      
      Shift_tunning_power = shift_values_move[tune_counter_move];
      DEBUG_PRINT_ACT("New ballance power value: ");
      DEBUG_PRINTLN_ACT(Shift_tunning_power);
      tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
      DEBUG_PRINTLN_ACT("button backwards is pressed"); // debug print
      int brightness = map(tune_counter_move,0,POWER_VALUES_RUN,0,255);
      pixels.setBrightness(brightness); // adjust brightness of LED for optical user feedback
      setLed(165,42,42);               // set LED to 

     }
  }

  display.clearDisplay();
  display.setCursor(10, 10);
  display.print("Straight: ");
  display.println(Shift_tunning_power);
  display.setCursor(10, 20);
  display.print("Degree  : ");
  display.println(Setpoint_t);
  display.display();


if (button_stop_count == 2) //switch to INIT state
  {
    machine_state = INIT_ST;
    button_stop_count = 0; // reset button stop counter
    button_stop.resetCount(); // reset button stop
    pixels.setBrightness(255); // reset brightness of LED after tune state 
  }
}

void stop(void) // function that is called between movements
{
  DEBUG_PRINTLN_FCT("exc stop fct"); // debug print
  DEBUG_PRINTLN_ACT("stop"); // debug print
  MotorControl.motorsStop(); // stop motors
  
  if (millis() >= time_now + STOP_DELAY) {
    machine_state = stop_next_state; 
  }
  
  stopExec(); // stop current execution
  
}

void fsm(void) // finite state machine
{
  
  DEBUG_PRINTLN_FCT("exc fsm fct");

  button_command.loop(); // loop() for button_command
  button_command_count = button_command.getCount(); // get count of how often command button was pressed
  
  button_stop.loop();
  button_stop_count = button_stop.getCount();

  DEBUG_PRINT_VAR("button_command_count: "); // debug print
  DEBUG_PRINTLN_VAR(button_command_count); // debug print
  
  DEBUG_PRINT_VAR("nr_comm: "); // debug print
  DEBUG_PRINTLN_VAR(nr_comm); // debug print
  
  switch (machine_state) { // switch to current machine state
  case INIT_ST: // execute init state
    last_machine_state = machine_state; // set last machine state
    init();  //execute func
    break;

  case READ_COMM_ST: // execute read comment state
    last_machine_state = machine_state; // set last machine state
    readComm(); //execute func
    break;

  case START_EXEC_ST: // execute start execution state
    last_machine_state = machine_state; // set last machine state
    startExec(); //execute func
    break;

  case EXEC_ST: // execute execute state
    last_machine_state = machine_state; // set last machine state
    exec(); //execute func
    break;

  case FORWARD_ST: // execute forward state
    last_machine_state = machine_state; // set last machine state
    forward(); //execute func
    break;

  case BACK_ST: // execute forward state
    last_machine_state = machine_state; // set last machine state
    back(); //execute func
    break;

  case TURN_RIGHT_ST: // execute turn right state
    last_machine_state = machine_state; // set last machine state
    turnRight(); //execute func
    break;

  case TURN_LEFT_ST: // execute turn left state
    last_machine_state = machine_state; // set last machine state
    turnLeft(); //execute func
    break;

  case STOP_ST:  //execute stop state
    last_machine_state = machine_state; // set last machine state
    stop(); //execute func
    break;

  case TUNE_ST: // execute tune state 
    last_machine_state = machine_state; // set last machine state
    tune(); // tune func
    break;

  case WAIT_ST: // execute tune state 
    last_machine_state = machine_state; // set last machine state
    wait(); // execute func
    break;

  case VOID_ST: // execute void state 
    // put code here
    break;
  }
  
} // fsm

void show_state(void){ // show state function is used for debuging
  if (machine_state != last_machine_state){ // show new state if the state has changed
    //DEBUG_PRINTLN_STATE(last_machine_state); // print last state
    DEBUG_PRINTLN_STATE(machine_state); // print current state
  }
} // show state

void setup() // microcontroller setup runs once
{
  Serial.begin(9600); // setup serial monitor
  DEBUG_PRINTLN_FCT("exc microcontroller setup fct"); // debug print
  
  //display setup 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay(); // Clear the buffer
  showBitmap(bitmap_uac_logo); // show uac logo
  delay(500); // show UAC logo for 0.5 sec
  #ifndef DEBUG_VISUAL_MODE
  showBitmap(image_data_EYES_MIDDLE);
  #endif
  //play startup melody
  startup_melody();
  
  // setup ez buttons debounce time
  button_command.setDebounceTime(button_debounce_time); 
  button_command.setCountMode(COUNT_FALLING);
  button_left.setDebounceTime(button_debounce_time);
  button_forwards.setDebounceTime(button_debounce_time);
  button_right.setDebounceTime(button_debounce_time);
  button_backwards.setDebounceTime(button_debounce_time);
  button_stop.setDebounceTime(button_debounce_time);


  // Neopixel setup
  pixels.begin(); // INITIALIZE NeoPixel strip object
  pixels.clear(); // Set all pixel colors to 'off'
  
  // Motor setup
  // Encoders Pins
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  // Motor Pins
  //MotorControl.attachMotors(25, 26, 32, 33); //ROBOT José trocar 25 por 27
  //@Change the pins to invert the movement:
  //@Change left with right
  MotorControl.attachMotors( 26, 25, 33, 32); //ROBOT José trocar 25 por 27


  // Tuning Setup
  tuningSetupTurn();
  tuningSetupMove();

  machine_state = INIT_ST; // set machine to init state
}

void loop() // microcontroller loop function 
{
//  DEBUG_PRINTLN_FCT("exc microcontoller loop fct"); // debug print
  fsm(); // execute finite state machine
  //show_state(); // execute show state fct for debugging
}