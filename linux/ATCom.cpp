/* attempt to migrate ARduino ATcommand library to linux
 * has not been tested yet
 * By Farhan Shaukat
 * Team Vermillion Millennium Falcon
 * 5/11/2012
 */

#include <stdio.h>
#include <string.h>

int main ()
{
	char str[500];
	return 0;
}

string sendComwdg(string str)
{
	str = "AT*COMWDG=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

string sendFtrim()
{
	str = "AT*FTRIM=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

string sendConfig(string option, string value)
{
	str = "AT*CONFIG=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, ",\"");
	str = strcat(str, option);
	str = strcat(str, "\",\"");
	str = strcat(str, value);
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

string sendRef(string fs)
{
	str = "AT*REF=";
	str = strcat(str, sequenceNumber);
	if (fs == "TAKEOFF") {
		strcat(str, ",290718208\r");
	}
	else if (fs == "LANDING") {
		str = strcat(str, ",290717696\r");
	}
	sequenceNumber++;
	return str;
}

string sendPcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
	str = "AT*PCMD=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, ",");
	str = strcat(str, enable);
	str = strcat(str, ",");
	str = strcat(str, fl2int(roll));
	str = strcat(str, ",");
	str = strcat(str, fl2int(pitch));
	str = strcat(str, ",");
	str = strcat(str, fl2int(gaz));
	str = strcat(str, ",");
	str = strcat(str, fl2int(yaw));
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

/* Movement functions */
/*int moveForward(float distanceInMeters)
{
  float i = 0;
  string moveForward = sendPcmd(1, 0, 1, 0, 0);
  delay(200);
  while (i < distanceInMeters) {
    string stayForward = sendPcmd(1, 0, 0, 0, 0);
    //delay(200);
    i += 0.2;
  }
  return 1;
}

int moveRotate(float yawInDegrees)
{
  int i = 0;
  while (i < yawInDegrees) {
    string stayRotate = sendPcmd(1, 0, 0, 0, 0.17);
    //delay(150);
    i += 8;
  }
  return 1;
}*/

string makeAnim(anim_mayday_t anim, int time)
{
	str = "AT*ANIM=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, ",");
	str = strcat(str, anim);
	str = strcat(str, ",");
	str = strcat(str, time);
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

string LEDAnim(int animseq, int duration)
{
	str = "AT*LED=";
	str = strcat(str, sequenceNumber);
	str = strcat(str, ",");
	str = strcat(str, animseq);
	str = strcat(str, ",1073741824,");
	str = strcat(str, duration);
	str = strcat(str, "\r");
	sequenceNumber++;
	return str;
}

long fl2int(float value)
{
	resultint.i = 0;
	if (value < -1 || value > 1) {
		resultint.f = 1;
	} else {
		resultint.f = value;
	}
	return resultint.i;
}
