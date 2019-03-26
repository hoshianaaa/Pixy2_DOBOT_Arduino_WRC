########    DOBOT基本動作API    #########
#ver3

INPUT = 1
OUTPUT = 2

#pinのモード選択
def pinMode(pin, pin_mode):
  if pin_mode == 1:
    dType.SetIOMultiplexingEx(api, pin, 3, 1)
  if pin_mode == 2:
    dType.SetIOMultiplexingEx(api, pin, 1, 1)

  else:
    pass

#ピンのデジタル値を読む
def digitalRead(pin):
  return dType.GetIODI(api, pin)[0]

#ピンのデジタル出力
def digitalWrite(pin, val):
  dType.SetIODOEx(api, pin, val, 1)


#home
def home():
  dType.SetHOMECmdEx(api, 0, 1)

#指定した座標まで移動
def PTPMovePos(pos):
  current_pose = dType.GetPose(api)
  dType.SetPTPCmdEx(api, 2, pos[0],  pos[1],  pos[2], current_pose[3], 1)

#現在の位置から差分移動
def PTPMoveDelta(d_pos):
  dType.SetPTPCmdEx(api, 7, d_pos[0],  d_pos[1],  d_pos[2], 0, 1)

def PTPMoveZ(d_z):
  dType.SetPTPCmdEx(api, 7, 0,  0,  d_z, 0, 1)

#サクションカップのon/off
def suction(on):
  dType.SetEndEffectorSuctionCupEx(api, on, 1)

#time秒とまる
def delay(time):
  dType.SetWAITCmdEx(api, time, 1)

#speed
def speed(vel, acc):
  dType.SetPTPCoordinateParamsEx(api,vel,acc,vel,acc,1)

#stepper1
def stepper1(on):
  if on == 1:
    dType.SetEMotorEx(api, 0, 1, 20000, 1)
  else:
    dType.SetEMotorEx(api, 0, 1, 0, 1)

#now time
def now():
  return dType.gettime()[0]



#######  よく使う動作  ##########################

def pickUP(d_z):
  #speed(1500, 500)
  PTPMoveZ(-d_z)
  #delay(0.5)
  suction(1)
  delay(0.2)
  PTPMoveZ(d_z)
#  delay(0.5)
  
  

##############################################








#counterの数に合ったブロックの位置に移動
def move_block(counter):
  count = counter
  if counter > 20:
    count = counter - 20
  
  pos = [156 - int((count - 1) / 4) * 25, 198 + ((count - 1) % 4) * 25, 25]
  PTPMovePos(pos)
 # delay(1)
  
  return counter + 1


def finish():
  for i in range(3):
    suction(1)
    delay(0.3)
    suction(0)
    delay(0.1)


counter = 1#何個目のブロックかを入れる default 1
home_pos_up = [278.1002, 0, 25]#homeポジション
home_pos_down = [278.1002, 0, 9.0167]#homeポジションから下がったところ
home_pos = [200, 0, 20]


PTPMoveZ(10)
#home()
speed(2000,3000)
pinMode(19,INPUT)
pinMode(14, OUTPUT)
PTPMovePos(home_pos)


while counter < 41:
  print (counter)
  digitalWrite(14, 0)
  if(digitalRead(19) == 1):
    counter = move_block(counter)
    if counter < 21:
      pickUP(50)
    if counter > 20:
      pickUP(75)
    PTPMovePos(home_pos_up)
    PTPMovePos(home_pos_down)
    suction(0)
    PTPMovePos(home_pos_up)
    PTPMovePos(home_pos)

    time = now()
    while(1):
      digitalWrite(14, 1)#stepperを動かしてるときにarduinoに信号を送る
      stepper1(1)
      if(float(now() - time) > 1.5):
        stepper1(0)
        break

while counter < 46:
  print (counter)
  digitalWrite(14, 0)
  if(digitalRead(19) == 1):

    time = now()
    while(1):
      digitalWrite(14, 1)#stepperを動かしてるときにarduinoに信号を送る
      stepper1(1)
      if(float(now() - time) > 1.5):
        stepper1(0)
        break
    counter += 1

delay(10)
finish()  

  
  
 




    
      
    
	
