void autosteerLoop()
{
    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    }
    else if (steerConfig.SteerButton == 1)    //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }
    else                                      // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    calcSteerAngle();
    calcSteeringPID();  //do the pid
    
    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2
      digitalWrite(DIR1_RL_ENABLE, HIGH);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
      motorDrive();       //out to motors the pwm value
    }
    else
    {

    digitalWrite(DIR1_RL_ENABLE, LOW); //IBT2

      pwmDrive = 0; //turn off steering motor
      motorDrive(); //out to motors the pwm value
      pulseCount = 0;
    } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  // Speed pulse
  /**
  if (gpsSpeedUpdateTimer < 1000)
  {
      if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
      {
          speedPulseUpdateTimer = 0;

          //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
          //gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          float speedPulse = gpsSpeed * 36.1111;

          //Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

          if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
              tone(velocityPWM_Pin, uint16_t(speedPulse));
          }
          else {
              noTone(velocityPWM_Pin);
          }
      }
  }
  else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
  {
      noTone(velocityPWM_Pin);
  }
*/
  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}
