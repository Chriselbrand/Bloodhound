
/*functions that will be implemented for level 3. go over with chris and have him look through. i believe this should make all test cases. is possible that may get caught in a corner
 * and just stand there and will have to implement some form of check if it stays in same location. possibly check movement which checks the x and y before go to and then checks after. as long
 * as different values continues. idk. and we will also need to make knowledge function next that will know what offset to use for calibrate function.
 */

/*look around function that check spaces and implements the coordinate in its system. map could make this easier im just not familiar with it yet and breaking it down in
individual coordinates helps me to see movement in head even though it is complex and may jump all over the place i traced it with several examples and looked promising*/
void look_around(){
  //check forward
  unsigned int AlphaR_time = AlphaR.ping_median(5);    delay(200);
  unsigned int AlphaL_time = AlphaL.ping_median(5);    delay(200);
           int A_time = (AlphaR_time + AlphaL_time)/2;
  if(A_time>0 && A_time<800){
    //save obstacle spot to be used later
    obstacle_map[x_pos][y_pos+1] = 1;
  }
  //check left
  unsigned int BravoR_time = BravoR.ping_median(5);    delay(200);
  unsigned int BravoL_time = BravoL.ping_median(5);    delay(200);
           int B_time = (BravoR_time + BravoL_time)/2;
  if(B_time>0 && B_time<800){
    obstaclex[obnum] = x_pos--;
    obstacley[obnum] = y_pos;
    obnum++;
  }
  //check backward
  unsigned int CharlieR_time = CharlieR.ping_median(5);    delay(200);
  unsigned int CharlieL_time = CharlieL.ping_median(5);    delay(200);
           int C_time = (CharlieR_time + CharlieL_time)/2;
  if(C_time>0 && C_time<800){
    obstaclex[obnum] = x_pos;
    obstacley[obnum] = y_pos--;
    obnum++;
  }
  //check right
  unsigned int DeltaR_time = DeltaR.ping_median(5);    delay(200);
  unsigned int DeltaL_time = DeltaL.ping_median(5);    delay(200);
           int D_time = (DeltaR_time + DeltaL_time)/2;
  if(D_time>0 && D_time<800){
    obstacle_map[x_pos+1][y_pos] = 1;
  }



/* function that will bring you to the location you desire kinda. if the coordinates need to be changed based on objects in the way it will act accordingly and then will go to place you want.
 *  will also possibly go to spot that is one block away from location needed then the next step will be to go to the desired location
 */
void Go_to(int x_finish, int y_finish){
  //counts if there was object in x or y direction
  int x_counter = 0;
  int y_counter = 0;
  //difference from position to desired position
  int x_difference = x_finish - x_pos;
  int y_difference = y_finish - y_pos;
  //while desired xpos not met
  while(x_difference != 0){
    //if need to go right
    if(x_difference>0)  {
      //check if object is between xpos and desired pos
      for(int i=x_finish;i>x_pos;i--;){
        if(obstacle_map[i][y_pos] = 1){
          int x_finish = i-1;
          x_counter = 1;
        }
        //go right if no objects
        else{Right(oneBlock);    look_around();}
      }
    }
    //if need to go left
    if(x_difference<0){
      //check if object is between xpos and desired pos
      for(int i=x_finish;i<x_pos;i++;){
        if(obstacle_map[i][y_pos]=1){
          int x_finish = i+1;
          x_counter = 1;
        }
        //go left if no objects
        else{Left(oneBlock)    look_around;}
    }
  }
  //update difference to see if you are where you want to be
  x_difference = x_finish - x_pos;
  }
  //if there was a block in x direction update y finish so that can move in y pos first then next goto will get you to location
  if(x_counter>0){
    if(y_pos>3){
      y_finish--;
      y_difference = y_finish-y_pos;
    }
    if(y_pos<4){
      y_finish++;
      y_difference = y_finish-y_pos;
    }
  }
  //while y is not at desired location
while(y_difference != 0){
  //if need to go forward
    if(y_difference>0)  {
      //check to see if objects inbetween location and desired location
      for(int i=y_finish;i>y_pos;i--;){
        //if obstacle update finish point so not to go to object
        if(obstacle_map[x_pos][i]==1){
          int y_finish = i-1;
          y_counter = 1;
        }
        //go forward since no blocks
        else{Forward(oneBlock); look_around();}
      }
    }
    //if need to go backward
    if(y_difference<0){
      //look for objects in way
      for(int i=y_finish;i<y_pos;i++;){
        //if objects update finish and adjust counter
        if(obstacle_map[x_pos][i]==1){
          int y_finish = i+1;
          y_counter = 1;
        }
        //go backward since no blocks
        else{Backward(oneBlock); look_around();}
      }
    }
    //update difference to see if you are where you want to be
    y_difference = y_finish - y_pos;
  }
  //if there was block which way were you going and hug object to get to position
  if(y_counter>0){
    if(x_pos==0 || x_pos==1 || x_pos==3){
      if(y_pos<5){
        Go_To(x_pos+1,y_pos+2);
      }
      else{Go_to(x_pos+1,y_pos+1);}
    }
    if(x_pos==2||x_pos==4||x_pos==5){
      if(y_pos>2{
        Go_To(x_pos+1,y_pos-2);
      }
      else{Go_to(x_pos+1,y_pos-1);}
    }
  }
}

         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
 void knowledge(){
  int offsetx_final = 6;
  int offsety_final = 6;
  int differencex_final = 0;
  int differencey_final = 0;
  for(int i=0; i<obnum; i++){
    if(y_pos == obstacley[i]){
      int x_offset = obstaclex[i];
      int offsetx_difference = x_offset - x_pos;
      int offsetx_change = abs(offsetx_difference);
      if(offsetx_change<offsetx_final){
        offsetx_final = offsetx_change;
        differencex_final = offsetx_difference;
      }
    }
    if(x_pos == obstaclex[i]){
      int y_offset = obstacley[i];
      int offsety_difference = offsety_difference - y_pos;
      int offsety_change = abs(offsety_difference);
      if(offsety_change<offsety_final){
        offsety_final = offsety_change;
        differencey_final = offsety_difference;
      }
    }
  }
  if(offsetx_final < 6){
    if(differencex_final<0){
      align_Bravo(1);
      offset_Bravo(offsetx_final);
      align_Bravo(1);
    }
    if(differencex_final>0){
      align_Delta(1);
      offset_Delta(offsetx_final);
      align_Delta(1);
    }
  }
  if(offsety_final < 6){
    if(differencey_final<0){
      align_Charlie(1);
      offset_Charlie(offsety_final);
      align_Charlie(1);
    }
    if(differencey_final>0){
      align_Alpha(1);
      offset_Alpha(offsety_final);
      align_Alpha(1);
    }
      
  }
}        
         
 
