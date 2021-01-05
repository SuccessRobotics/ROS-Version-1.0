#include <ArduinoHardware.h>
#include <ros.h> 
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float32.h> 
#include <Encoder.h>

Encoder myEncL(2, 27);
Encoder myEncR(19, 23);
long oldPositionL  = 0;
long oldPositionR  = 0;
ros::NodeHandle nh;


geometry_msgs::Twist msg;
std_msgs::Float32 encL_msg;
std_msgs::Float32 encR_msg;

ros::Publisher EncL("Enc_L", &encL_msg);
ros::Publisher EncR("Enc_R", &encR_msg);

  long newPositionL;
  long newPositionR;
  
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{

	double x = cmd_vel.linear.x;
        double z = cmd_vel.angular.z;

	double moveL = x+(z/2);
	double moveR = x-(z/2);

  
if (moveL>0.0){
      analogWrite(5,max(min(moveL*100,60),35));
      digitalWrite(30,0);digitalWrite(32,1);
    }else if (moveL<0.0){
	analogWrite(5,max(min(abs(moveL)*100,60),35));
        digitalWrite(30,1);digitalWrite(32,0);
    }else{ 
	analogWrite(5,0);
        digitalWrite(30,0);digitalWrite(32,0);
	}

if (moveR>0.0){
      analogWrite(7,max(min(moveR*100,60),35));
      digitalWrite(22,0);digitalWrite(24,1);
    }else if (moveR<0.0){
	analogWrite(7,max(min(abs(moveR)*100,60),35));
        digitalWrite(22,1);digitalWrite(24,0);
    }else{ 
	analogWrite(7,0);
        digitalWrite(22,0);digitalWrite(24,0);
        }

 if(cmd_vel.linear.y>=1.0){ 
 digitalWrite(42,1);
 } else  {
 digitalWrite(42,0);
 }
}
ros::Subscriber <geometry_msgs::Twist> Motor("/cmd_vel", roverCallBack);

void setup()
{
  pinMode(5,OUTPUT);  pinMode(32,OUTPUT); pinMode(30,OUTPUT);
  pinMode(7,OUTPUT);  pinMode(22,OUTPUT); pinMode(24,OUTPUT);
  pinMode(42,OUTPUT); 
  digitalWrite(42,1); delay(100);
  digitalWrite(42,0); delay(100);
  digitalWrite(42,1); delay(100);
  digitalWrite(42,0); delay(100);
  nh.initNode();
  nh.subscribe(Motor);
  nh.advertise(EncL);
  nh.advertise(EncR);
} 

void loop()
{
    newPositionL = myEncL.read();
  newPositionR = myEncR.read()*-1;
  
  if (newPositionL != oldPositionL) {
    oldPositionL = newPositionL;
     encL_msg.data = newPositionL;
  }
  if (newPositionR != oldPositionR) {
    oldPositionR = newPositionR;
     encR_msg.data = newPositionR;
  }
  if((newPositionL>1000000)||(newPositionR>1000000)||(newPositionL<-1000000)||(newPositionR<-1000000)){
  myEncL.write(0);
  myEncR.write(0);
  }
  EncL.publish( &encL_msg );
  EncR.publish( &encR_msg );
  nh.spinOnce();
  delay(10);
}
