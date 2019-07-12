#include <Platform.h>

Platform platform;

void setup() 
{

  platform.init();

  Serial.begin(230400);
  while (!Serial)
  {
  }
}

void loop() 
{
  platform.step();
}