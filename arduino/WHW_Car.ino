#include <Servo.h>

/**
 * 1.测量左右两侧的距离
 * 2.判断轮子是否在转动，避免卡死
 * 3.边缘容易卡住，还需要更多的传感器
 *
 */

//Servo 库会使第9和第10脚上的analogWrite()(PWM)功能失效
Servo servo1;
Servo servo2;
const int TrigPin = 7;
const int EchoPin = 8;
const int left1 = 3;
const int left2 = 5;
const int servo1Pin = 9;
const int servo2Pin = 10;
const int right1 = 6;
const int right2 = 11;
float distance;
//这种微调，应该写入存储中
//微调 == 在直行时PWM减去该值
//左侧微调
int leftMinitrim = 10;
//右侧微调
int rightMinitrim = 0;
//自动运行
boolean autoRun = true;


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);

  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);

  Serial.println("begin");

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  Serial.println("servo restoration");
  servo1.write(90);
  servo2.write(90);
}
/**
 * 测量距离
*/
void measureDistance() {
  // 产生一个10us的高脉冲去触发TrigPin
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  // 检测脉冲宽度，并计算出距离
  distance = pulseIn(EchoPin, HIGH) / 58.00;
  //  Serial.print(distance);
  //  Serial.print("cm");
  //  Serial.println();
}

/**
 * 停车
 */
void stopCar() {
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
}

/**
 * 直行，正 == 前；负 == 后
 */
void pRun(int _speed) {
  if (_speed > 0) {
    //    Serial.print("forward   speed = ");
    //    Serial.println(_speed);

    digitalWrite(left2, LOW);
    analogWrite(left1, _speed - leftMinitrim);

    digitalWrite(right2, LOW);
    analogWrite(right1, _speed - rightMinitrim);
  } else {
    _speed = 0 - _speed;

    //    Serial.print("back   speed = ");
    //    Serial.println(_speed);

    digitalWrite(left1, LOW);
    analogWrite(left2, _speed - leftMinitrim);

    digitalWrite(right1, LOW);
    analogWrite(right2, _speed - rightMinitrim);
  }
}

/**
 * 转弯，正 == 右转；负 == 左转
 */
void wheel(int _speed) {
  //右转
  if (_speed > 0) {
    //Serial.print("--> right rurn   speed = ");
    //Serial.println(_speed);

    digitalWrite(left2, LOW);
    digitalWrite(right1, LOW);
    analogWrite(left1, _speed);
    analogWrite(right2, _speed);
  } else {
    //左转
    _speed = 0 - _speed;
    //Serial.print("<-- left  rurn   speed = ");
    //Serial.println(_speed);

    digitalWrite(left1, LOW);
    digitalWrite(right2, LOW);
    analogWrite(left2, _speed);
    analogWrite(right1, _speed);
  }
}

//操作类型代码常量定义
const int W_LEFTMINITRIM = 5; //左侧微调
const int W_RIGHTMINITRIM = 6; //右侧微调
const int W_RUN_MODEL = 10; //运行模式
const int W_GO_FORWARD = 11; //前进
const int W_GO_BACK = 12; //后退
const int W_LEFT_TURN = 13;//左转
const int W_RIGHT_TURN = 14;//右转
const int W_MEASURE_DISTANCE = 15;//测量距离

//输入数据,定长4个字节
byte inBytes[4];

//连续几次没有读到指令就应该停车
int readStatus = 0;
//操作精细度，数字越小越精确
const int OPERATE_ACCURACY = 4;
//休眠毫秒数，关系的操作精确性
const int DELAY_TIME =  50;

/*
 * 自动运行状态记录
 * 不能进入内部循环，否则不能及时接收到指令
 */

void loop() {

  if (Serial.available() > 0) {
    //读取传入的字节：
    int readSize = Serial.readBytes(inBytes, 4);
    readStatus = 0;

    //Serial.print("Received Size: ");
    //Serial.println(readSize, DEC);

    if (readSize ==  4) {
      int type = inBytes[0];
      switch (type) {
        case   W_LEFTMINITRIM:
          leftMinitrim = inBytes[1];
          break;
        case W_RIGHTMINITRIM:
          rightMinitrim = inBytes[1];
          break;
        case W_RUN_MODEL:
          autoRun = inBytes[1];
          break;
        case W_GO_FORWARD:
          pRun(inBytes[1]);
          break;
        case W_GO_BACK:
          pRun(0 - inBytes[1]);
          break;
        case W_LEFT_TURN:
          wheel(inBytes[1]);
          break;
        case W_RIGHT_TURN:
          wheel(0 - inBytes[1]);
          break;
        case W_MEASURE_DISTANCE:
          measureDistance();
          //应该返回距离
          break;
        default:
          //报警
          Serial.println("error order");
      }
    } else {
      //否则丢弃
      Serial.println("error serial stream");
    }
  }

  //测量距离
  //测量距离的时候应该测试一下左右两侧的距离，避免左右两侧撞上---有时间再弄
  measureDistance();

  if (autoRun) {//自动运行
    //自动运行模式不能卡死，应该读取串口数据
    if (distance < 10) {//小于10  倒车
      int backCount = 0;
      int leftWheelCount = 0;
      int rightWheelCount = 0;
      while (distance < 30) {//需要弄出一定的距离

        if (backCount > 20) { //倒车倒不动了
          //开始转弯
          if (leftWheelCount < 20) {
            leftWheelCount ++;
            wheel(-180);
          } else {
            //开始右转
            if (rightWheelCount < 20) {
              rightWheelCount ++;
              wheel(180);
            } else {
              //卡死了，停车，报警
              stopCar();
              Serial.println("stop car");
              delay(1000);
              break;
            }
          }
        } else {
          backCount++;
          pRun(-180);
        }
        delay(50);
        measureDistance();
      }
    } else if (distance < 25) {//如果距离小于25，则转弯
      wheel(180);
    } else if (distance > 100) {//全速前进
      pRun(255);
    } else if (distance > 50) {//半速前进
      pRun(225);
    } else {//慢速前进
      pRun(180);
    }
  } else {
    //如果多次收不到指令应该停止运行
    if (readStatus > OPERATE_ACCURACY) {
      stopCar();
    } else {
      readStatus ++;
    }
  }

  delay(DELAY_TIME);
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
