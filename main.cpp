#include "kinect.h"
#include "pioneer.h"
#include <time.h>
#include <pthread.h>
#include <stdio.h>

#include "Aria.h"

#include "slam.h"
#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArRobot.h"


AREXPORT Avoid::Avoid(const char *name,
                                            ArPose goal,
                                            double obstacleFrontDistance,
                                            double obstacleSideDistance,
                                            double minDistance) :
  ArAction(name, "Avoids side obstacles, ie walls")
{
  setNextArgument(ArArg("goal", &myGoal, "ArPose to go to. (ArPose)"));
  myGoal = goal;
  myOldGoal = myGoal;
  setNextArgument(ArArg("obstacle front distance", &myObsDist_front,
            "Distance at which to start avoiding (mm)"));
  myObsDist_front = obstacleFrontDistance;
  setNextArgument(ArArg("obstacle side distance", &myObsDist_side,
            "Distance at which to start avoiding (mm)"));
  myObsDist_side = obstacleSideDistance;
  setNextArgument(ArArg("turn amount", &minimum_Dist,
            "Degrees at which to turn (deg)"));
  minimum_Dist=minDistance;

  //雷达数据大突变，保持
  leftDist_before = 0;
  rightDist_before = 0;

  //标志区//
  //到达目标，停止机器人标志
  STOP_CAR_FIRST = false;

  //新定义目标点后，车体指向目标点完毕标志
  ACHIEVE_ANGLE = false;

  //转向目标点标志，
  TURN_TO_GOAL = false;

  //当前位置到目标位置角度
  poseAngle = 0;
}

AREXPORT Avoid::~Avoid()
{

}
//到达目标点方法
AREXPORT bool Avoid::haveAchievedGoal(void)
{
  if (myState == STATE_ACHIEVED_GOAL)
    return true;
  else
    return false;
}
//取消目标点方法
AREXPORT void Avoid::cancelGoal(void)
{
  myState = STATE_NO_GOAL;
}
//设置目标点方法
AREXPORT void Avoid::setGoal(ArPose goal)
{
  myState = STATE_GOING_TO_GOAL;
  myOldGoal = myGoal;
  myGoal = goal;
  //myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
}


//action 开始方法
AREXPORT ArActionDesired *Avoid::fire(
    ArActionDesired currentDesired)
{
    //设置目标点偏了，纠正
    if(myGoal.findDistanceTo(myOldGoal) > 5){
        setGoal(myGoal);
    }
    //当前位置与目标点之间的夹角
    poseAngle = myRobot->getPose().findAngleTo(myGoal);
    //重新设定目标点后进行方位对准，指向目标点
    if(!ACHIEVE_ANGLE){
        //poseAngle = myRobot->getPose().findAngleTo(myGoal);
        if(ArMath::fabs(poseAngle - myRobot->getTh())>1){
            myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
            printf("try to pose to the target1, angle is %f \n", myRobot->getPose().findAngleTo(myGoal));
        }
        else{
            ACHIEVE_ANGLE = true;
            printf("ACHIEVE_ANGLE\n");
        }
        return &myDesired;
    }

    // 到达目标点或者已经取消了目标点，什么不做返回
    if (myState == STATE_ACHIEVED_GOAL || myState == STATE_NO_GOAL){
        return NULL;
    }
  //定义左边距离与右边距离
  double leftDist, rightDist;
  //定义前方距离和探测到最小距离的声呐环上的那个声呐在声呐环坐标系下的夹角
  double dist, angle;
  //小车距离目标点的距离
  double dist_to_goal;
  //左右两边声呐测距减去侧边安全距离之后的值
  double left_err;
  double right_err;
  //车的指向和目标指向的夹角，得做处理
  double angle_attractive;
  //车左右轮速度
  double Vl, Vr;

  double X,Y;

  //重要参数，避障转向的比例系数
  double k1 = 0.2;  //0.2

  //较远距离找目标点调整参数，有关角度
  double k2 = 1.8; // 1.8
  //较远距离找目标点调整参数，有关距离
  double kp2 = 0.10; // 0.1

  //较近距离找目标点调整参数，有关角度
  //double kp3_angle = 0.8;
  //较近距离找目标点调整参数，有关距离
  double kp3_dist = 0.6;  // 0.6 //2

  //进入目标点的误差
  double err_goal = 0;

  //直线行走的小车最大速度
  double  car_val_normal = 200; //300
  //转向时用的速度
  double  car_val_turn = 100; //150;
  //探测前方距离，利用-40度到40度之间的声呐来探测
  dist = myRobot->checkRangeDevicesCurrentPolar(-40, 40, &angle);
  //计算车体位置到目标点距离
  dist_to_goal = myRobot->getPose().findDistanceTo(myGoal);
      //- myRobot->getRobotRadius());
  //计算左边距离
  leftDist = myRobot->checkRangeDevicesCurrentPolar(60, 120);
      //- myRobot->getRobotRadius());
  //计算右边距离
  rightDist = myRobot->checkRangeDevicesCurrentPolar(-120, -60);
          //myRobot->getRobotRadius());

  //如果左边或者右边距离大于4000mm，很有可能差声波反射走了，很有可能为伪数据，
  //改用20到120度之间的声呐再测一次，提高数据可信度
  if(leftDist >4000){
    leftDist = myRobot->checkRangeDevicesCurrentPolar(20, 120) ;
      //myRobot->getRobotRadius());
  }
  if(rightDist >4000){
    rightDist = myRobot->checkRangeDevicesCurrentPolar(-120, -20) ;
      //myRobot->getRobotRadius());
  }

  //如果前方距离大于1000，也有可能数据不准，利用-70到70度之间的数据再测一次
  //提高数据可信度
  if(dist >1000){
    dist = myRobot->checkRangeDevicesCurrentPolar(-70, 70, &angle) ;
      //- myRobot->getRobotRadius());
  }
  //计算车体当前指向与目标点之间的夹角，该角度很诡异，后期需要重新认真检查判断
  angle_attractive = ArMath::fabs(ArMath::subAngle(myRobot->getPose().findAngleTo(myGoal), myRobot->getTh()));


  //车体当前指向与目标点之间的夹角，该角度很诡异，我这里简单的做处理（大于90度用180去减），上次调试已经发现该处理不对
  //，后期调试请先把该角度处理正确<优先级很高！>
  if(angle_attractive>90){
    angle_attractive = 180 -angle_attractive;
  }


  //屏蔽周围过大环境,提高抗干扰性，初始设置为1800mm
  if(leftDist>1800){
    leftDist = 1800;
  }
  if(rightDist>1800){
    rightDist = 1800;
  }
  //减去设定的边缘安全距离
  left_err = leftDist - myObsDist_side;
  right_err = rightDist - myObsDist_side;
  X = myRobot->getX();
  Y = myRobot->getY();
  myDesired.reset();
  //部分调试信息
  printf("\n\n**");
  printf("dist = %f \n", dist);
  printf("distacne between my pose to goal =  %f\n",dist_to_goal);
  printf("left_err = %f \n", left_err);
  printf("right_err = %f \n", right_err);
  printf("angle_attractive = %f \n", angle_attractive);
  printf("poseAngle = %f \n", poseAngle);
  printf("X = %f \n", X);
  printf("Y = %f \n", Y);
  printf("*******************************\n\n");

//到达目标点做相应处理，进入目标点为圆心的600mm 环开始做处理
if(dist_to_goal<1000){
    if(dist_to_goal<200){
    //进入200mm 环，判断为到达目标点
        //myRobot->clearDirectMotion();
    //因为定时处理，所以到达目标点后第一个周期先停车
        if(!STOP_CAR_FIRST){
            myRobot->stop();
            STOP_CAR_FIRST = true;
            printf("STOP_CAR_FIRST = true\n");
        }
        printf("achieved the goal\n");
    //设置全局状态：到达目标点
        myState = STATE_ACHIEVED_GOAL;
        STOP_CAR_FIRST = false;
        ACHIEVE_ANGLE = false;
        return &myDesired;
    }
  //否则先调整姿态对准目标点
    if(ArMath::fabs(poseAngle - myRobot->getTh())>5){
        if(!TURN_TO_GOAL){
            myRobot->stop();
            TURN_TO_GOAL = true;
            printf("TURN_TO_GOAL = true\n");
        }
        myRobot->setHeading(myRobot->getPose().findAngleTo(myGoal));
        printf("try to pose to the target2, angle is %f \n", myRobot->getPose().findAngleTo(myGoal));
        printf("当前位姿信息= %f\n：",myRobot->getTh());
    }
    else{
    //对准目标点之后然后一直直线开过去
        TURN_TO_GOAL = false;
        err_goal = dist_to_goal-200;
        Vl = kp3_dist*err_goal;
        Vr = kp3_dist*err_goal;
    //做个限速
        if(Vl<100){
            Vl = Vr = 100;
        }
        myRobot->setVel2(Vl, Vr);
        printf("near the goal\n");
        printf("V = %F \n", Vr);
        printf("already pose to the target \n");
    }
    return &myDesired;
}

//前方没有障碍物
//if((dist>myObsDist_front)||(minimum_Dist>myObsDist_front)){
if(dist>myObsDist_front){
    //但是两边都太窄了，过不去，后退
    if((right_err<0)&&(left_err<0)){
        myRobot->setVel2(-100, -100);
        printf("front distacne is large, but the left and right distacne is too narrow, go back \n");
    }
    else if((right_err < 0)&&(left_err>0))
    {
    //或者右窄左宽，左转
        Vl = car_val_turn - k1*rightDist;
        Vr = car_val_turn + k1*rightDist;
        myRobot->setVel2(Vl, Vr);
        printf("front distacne is large, go left\n");
    }
    else if((left_err < 0)&&(right_err>0))
    {
    //或者左宽右窄，右转
        Vl = car_val_turn + k1*leftDist;
        Vr = car_val_turn - k1*leftDist;
        myRobot->setVel2(Vl, Vr);
        printf("front distacne is large, go right\n");
    }
    else{
        //前面空荡荡，那就找目标点，机器人在目标位置的左边，右转追寻目标点
        if((((myRobot->getPose().getY() - myGoal.getY())>270)&&(ArMath::fabs(poseAngle)<=90)) \
            ||(((myRobot->getPose().getY() - myGoal.getY())<-270)&&(ArMath::fabs(poseAngle)>=90))){ //100
        //	Vl = car_val_turn + kp2*((myRobot->getPose().getY() - myGoal.getY())) + k2*angle_attractive;
        //	Vr = car_val_turn - kp2*((myRobot->getPose().getY() - myGoal.getY())) - k2*angle_attractive;
      myRobot->setHeading(poseAngle);
      myRobot->setVel(200);
        //	myRobot->setVel2(Vl, Vr);
            printf("go for the goal\n");
        }
        //前面空荡荡，那就找目标点，避障之后机器人在目标位置的右边，左转追寻目标点
        else if((((myRobot->getPose().getY() - myGoal.getY())>270)&&(ArMath::fabs(poseAngle)>90)) \
            ||(((myRobot->getPose().getY() - myGoal.getY())<-270)&&(ArMath::fabs(poseAngle)<90))){ //100
            // Vl = car_val_turn + kp2*((myRobot->getPose().getY() - myGoal.getY())) - k2*angle_attractive;
            // Vr = car_val_turn - kp2*((myRobot->getPose().getY() - myGoal.getY())) + k2*angle_attractive;
            // myRobot->setVel2(Vl, Vr);
      myRobot->setHeading(poseAngle);
      myRobot->setVel(200);
            printf("go for the goal\n");
        }
        //目标点在容错方位范围内，切换为快速直奔目标点
        else{
            //car_val_normal = 300;
            Vl = Vr = car_val_normal;
            myRobot->setVel2(Vl, Vr);
            printf("no obstacle,go head\n");
        }
    }
    return &myDesired;
}
//遇到障碍物了，在安全距离的0.8倍之外，先减速做好避寒准备
//if((dist>myObsDist_front*0.8)||(minimum_Dist>myObsDist_front*0.8)){
if(dist>myObsDist_front*0.8){
    car_val_normal = 250*dist/myObsDist_front;
    if (car_val_normal > 250){
        car_val_normal = 250;
    }
  //左右两边都是障碍物，进入死胡同了，后退
    if((right_err<0)&&(left_err<0)){
        myRobot->setVel2(-50, -50);
        printf("near obstacle, but the left and right distacne is too narrow,go back \n");
    }
  //右边小左边大，左转
    else if ((right_err < 0)&&(left_err>0))
    {
        Vl = car_val_turn - k1*rightDist;
        Vr = car_val_turn + k1*rightDist;
        myRobot->setVel2(Vl, Vr);
        printf(" near obstacle, go left\n");
    }
  //右边大左边小，右转
    else if ((left_err < 0)&&(right_err>0))
    {
        Vl = car_val_turn + k1*leftDist;
        Vr = car_val_turn - k1*leftDist;
        myRobot->setVel2(Vl, Vr);
        printf("near obstacle, go right\n");
    }
    else{
    //减速直走
        Vl = Vr = car_val_normal;
        myRobot->setVel2(Vl, Vr);
        printf("near obstacle,slow down\n");
    }
    return &myDesired;
}
//障碍物距离在0.4到0.8安全距离范围中，进行避障
// if((dist >0.4*myObsDist_front)||(minimum_Dist>myObsDist_front*0.4)){
  if(dist >0.4*myObsDist_front){
      //左转避障
      if(leftDist>rightDist){
        rightDist = myRobot->checkRangeDevicesCurrentPolar(-190, -20);
        if(rightDist>800){
            rightDist = 800;
        }
        Vl = car_val_turn - k1*rightDist ;//+ k2*angle_attractive;
        Vr = car_val_turn + k1*rightDist ;//- k2*angle_attractive;
        myRobot->setVel2(Vl, Vr);
        printf("turn left\n");
      }
      else{
        //右转避障
        leftDist = myRobot->checkRangeDevicesCurrentPolar(20, 190);
        if(leftDist>800){
            leftDist = 800;
        }
        Vl = car_val_turn + k1*leftDist;  //- k2*angle_attractive;
        Vr = car_val_turn - k1*leftDist;  //+ k2*angle_attractive;
        myRobot->setVel2(Vl, Vr);
        printf("turn right\n");
      }
  }//else if((dist <= 0.4*myObsDist_front)||(minimum_Dist>myObsDist_front)){ //小于0.4倍的安全距离，眼看就要撞上了，紧急处理
 else if(dist <= 0.4*myObsDist_front){ //小于0.4倍的安全距离，眼看就要撞上了，紧急处理
      //左边还有空间，向左边逃离
        if((left_err > right_err)&&(right_err > -100)){
            myRobot->setVel2(-70, 70);
            printf("too close to the obstacle ,try to escape and turn left\n");
        }
      //右边还有空间，向右边逃离
        else if ((right_err > left_err)&&(left_err > -100))
        {
            myRobot->setVel2(70,-70);
            printf("too close to the obstacle ,try to escape and turn right\n");
        }
        else{
        //两边都没空间了，只能倒退
            myRobot->setVel2(-70, -70);
            printf("too close to the obstacle ,go back -_-!\n");
        }
  }

  return &myDesired;
}



int main(int argc, char **argv)
{


  Aria::init();

  argc = 3;
  argv[0] = "";
  argv[1] = "-rp";
  argv[2] = "/dev/ttyUSB0";
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArAnalogGyro gyro(&robot);
  ArSonarDevice sonar;
  ArRobotConnector robotConnector(&parser, &robot);

  if(!robotConnector.connectRobot())
  {
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

 imgdb.setRobot(&robot);

  int Rc;
  pthread_t slam_thread;
  Rc = pthread_create(&slam_thread, NULL, slam_event, NULL);
  if (Rc) { printf("Create slam thread failed!\n"); exit(-1); }

  ImageList* current = NULL;
  while (!current) current = imgdb.getEdge();
    // current->person_point


  robot.addRangeDevice(&sonar);
  robot.runAsync(true);

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  robot.setAbsoluteMaxRotVel(30);

  ArActionStallRecover recover;
  ArActionBumpers bumpers;
  double minDistance=10000;
 // ArPose endPoint(point.x, point.z);

//  current->collect_x
//          current->collect_y
//          robot.getX()
//          robot.getY()
//  current = imgdb.getEdge();

//  for (int i = 0; i < current->edge.size(); i++) {
//      for (int j = 0; j < current->edge[i].size() - 1; j++) {
//          double dis = sqrt(pow(current->edge[i][j].z,2)+(pow(current->edge[i][j].x,2)));
//           if(dis<minDistance){
//              minDistance=dis;
//           }
//      }
//  }

 // 引入避障action,前方安全距离为500mm,侧边安全距离为340mm
  Avoid avoidSide("Avoid side", ArPose(0, 0), 800, 100, 1000*minDistance);



  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 95);
  robot.addAction(&avoidSide, 80);

  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 50);

  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);


  const int duration = 500000;

  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();

  while (Aria::getRunning())
  {
    current = imgdb.getEdge();
    for (int i = 0; i < current->edge.size(); i++) {
        for (int j = 0; j < current->edge[i].size() - 1; j++) {
    //        printf("i=%d\n",i);
            double dis = sqrt(pow(current->edge[i][j].z,2)+(pow(current->edge[i][j].x,2)));
       //     printf("dis=%f,mindis=%f\n",dis,minDistance);
             if(dis<minDistance){
                minDistance=dis;
   printf("min=%f\n",minDistance);
             }
        }
    }
    //引入避障action,前方安全距离为500mm,侧边安全距离为340mm
 //   Avoid avoidSide("Avoid side", ArPose(0, 0), 800, 200, minDistance);
  //  robot.addAction(&avoidSide,80);
    robot.lock();

    if (first || avoidSide.haveAchievedGoal())
    {
      first = false;
      goalNum++;
      printf("count goalNum = %d\n", goalNum);
      if (goalNum > 2){
        goalNum = 1; // start again at goal #1
      }
//   avoidSide.setGoal(ArPose(10000,0));
        if(goalNum==1){
        printf("zuobiaox=%f,zuobiaoy=%f\n",-current->person_point.z,current->person_point.x);
        avoidSide.setGoal(ArPose(1000*current->person_point.z, -1000*current->person_point.x));
        printf("goalNum == 1\n\n");
        }
        else if(goalNum==2){
            printf("zuobiaox=%f,zuobiaoy=%f\n",-current->person_point.z,current->person_point.x);
            avoidSide.setGoal(ArPose(1000*current->person_point.z, -1000*current->person_point.x));
            printf("goalNum == 2\n\n");
        }

    }
    if(start.mSecSince() >= duration) {
      ArLog::log(ArLog::Normal, "%d seconds have elapsed. Cancelling current goal, waiting 3 seconds, and exiting.", duration/1000);
      avoidSide.cancelGoal();
      robot.unlock();
      ArUtil::sleep(3000);
      break;
    }
    robot.unlock();
  ArUtil::sleep(100);
  }

  Aria::exit(0);
  return 0;
}
