#include <Platform.h>
#include <definitions.h>
#include <definitions_2.h>


//! 1
float Platform::map(float x, float in_min, float in_max, float out_min, float out_max)
{
  float mapping = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  return mapping<-out_min ? -out_min : (mapping>out_max ? out_max : mapping);
}
