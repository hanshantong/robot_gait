//##################################################
//# PROJECT: P1MC POS_TRAJ.CPP
//# AUTHOR : li chunjing
//##################################################
/*******************************************************************************
* Copyright (c) 2017, UBT CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of UBT nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <P1MC.h>

Motion_Ctrl::Motion_Ctrl(float time)
{
  _time = time;
  com=0;
  p=0;
  c=0;
  l=0;
  cankle1=0;
  cankle2=0;
  T=0;  
}

void Motion_Ctrl::update_time(float time)
{
  _time = time;
  //_time=0;
}

bool Motion_Ctrl::goto_zero(dxl_Actuator *dxl_actuator)
{
  // static float theta_1 = dxl_actuator->get_start_theta();
	if(dxl_actuator->get_present_theta() > 0.1)
   	{
   		dxl_actuator->traj_theta = dxl_actuator->traj_theta - 5.0/200;
      dxl_actuator->set_goal_theta(dxl_actuator->traj_theta);
   	}

    if(dxl_actuator->get_present_theta() < -0.1)
   	{
      dxl_actuator->traj_theta = dxl_actuator->traj_theta + 5.0/200;
      dxl_actuator->set_goal_theta(dxl_actuator->traj_theta);
   	}

   	if(abs(dxl_actuator->get_present_theta()) <= 0.1)
   	{
   		dxl_actuator->set_goal_position(0);
      return true;
   	}
    return false;	
}

void Motion_Ctrl::read_para()
{
  FILE *fp;
  char para[50];
  float fValue;
  if( ( fp = fopen( "./vsw.txt", "r" ) ) == NULL )
  {
    printf( "error loading config file ./vsw.txt\n" );
  }
  else
  {
    for( int i = 0; i < 7; i++ )
    {
      fscanf( fp, "%s\t%f", para,&fValue);
      if (!strcmp(para,"p"))
      {
        p=fValue;
      }  
      else if (!strcmp(para,"c"))
      {
        c=fValue;
      }       
      else if (!strcmp(para,"l"))
      {
        l=fValue;
      }
      else if (!strcmp(para,"com"))
      {
        com=fValue;
      }   
      else if (!strcmp(para,"cankle1"))
      {
        cankle1=fValue;
      } 
      else if (!strcmp(para,"cankle2"))
      {
        cankle2=fValue;
      }   
      else if (!strcmp(para,"T"))
      {
        T=fValue;
      }                                      
      
    }
    fclose( fp );
  }
  //printf("%f %f %f %f %f %f %f\n",p,c,l,com,cankle1,cankle2,T);       
}

void Motion_Ctrl::stand(dxl_Actuator *dxl_actuator, int nNum)
{
  //float stand[6]={8.2,16.9,8.7,-8.5,3.5,12};
  //float stand[6]={14.9,25.1,10.7,-15.1,5.1,20.7};
  //float stand[6]={11,22,11,35,70,35};
  //float stand[6]={14,23.5,9.5,-11,3.5,14.5};
  float stand[12]={0,0,0,0,0,0,
                   0,0,0,0,0,0};
  // get_vsw_stand(stand,6);

  for(int i=0;i<nNum;i++)
  {  
    if(stand[i] - dxl_actuator[i].get_present_theta() > 0.5)
    {
      stand_zero[i] += 5.0/200;
      dxl_actuator[i].set_goal_theta(stand_zero[i]);    
    }
    else if(stand[i] - dxl_actuator[i].get_present_theta() < -0.5)
    {
      stand_zero[i] -=5.0/200;
      dxl_actuator[i].set_goal_theta(stand_zero[i]); 
    }
    else
    {
      dxl_actuator[i].set_goal_theta(stand[i]);    
    }
  } 
}

void  Motion_Ctrl::get_vsw_stand(float *_stand, int nNum)
{
  float t,s0,s1,s2;
  t=0;
  s0=p*cos(pi*t/T);
  if (t-2*T*(int)(t/(2*T))<0.5*T) {
      s1=c;
      s2=-0.5*(l+c)*cos(pi*(t-2*T*(int)(t/(2*T)))/(0.5*T))+0.5*(l+c);
  } else if (t-2*T*(int)(t/(2*T))<T) {
      s1=0.5*c*cos(pi*(t-2*T*(int)(t/(2*T))-0.5*T)/(0.5*T))+0.5*c;
      s2=0.5*(l)*cos(pi*(t-2*T*(int)(t/(2*T))-0.5*T)/(0.5*T))+0.5*l+c;
  } else if (t-2*T*(int)(t/(2*T))<1.5*T) {
      s1=-0.5*(l+c)*cos(pi*(t-2*T*(int)(t/(2*T))-T)/(0.5*T))+0.5*(l+c);
      s2=c;
  } else {
      s1=0.5*(l)*cos(pi*(t-2*T*(int)(t/(2*T))-1.5*T)/(0.5*T))+0.5*l+c;
      s2=0.5*c*cos(pi*(t-2*T*(int)(t/(2*T))-1.5*T)/(0.5*T))+0.5*c;
  }

  _stand[0] = 0.5*s0+s1+com;
  _stand[3] = -0.5*s0+s2+0.5*com;
  _stand[1] = 2*s1-2*com;
  _stand[4] = 2*s2-com;
  _stand[2] = _stand[1] - _stand[0] + cankle1;
  _stand[5] = _stand[4] - _stand[3] + cankle2;   
}

void Motion_Ctrl::run_vsw(dxl_Actuator *dxl_actuator, int nNum)
{

/*
Slow

One round: 30.5s, speed: 1.3LegLength/s

    tStep=0.9;
    pAngle=40;
    cAngle=10;
    lAngle=20;


Fast

One round: 22.5s, speed: 1.8LegLength/s

    tStep=0.6;
    pAngle=40;
    cAngle=10;
    lAngle=20;


Faster

One round: 16.1s, speed: 2.5LegLength/s

    tStep=0.5;
    pAngle=45;
    cAngle=19;
    lAngle=25;

*/


  float t,s0,s1,s2;   
  float sHeel1,sHeel2,sKnee1,sKnee2,sAnkle1,sAnkle2;
  //com = -5.1;

  //p=20;
  //c=10;
  //l=15;

  //cankle1 = 0;
  //cankle2 = 0;
  t = _time;// + T/2;

  //t=T/2;
  if(t>=6*T)
    t=6*T;
  //if(t<8.5*T)
  //{

  //if(t>2.5*T)
    //p=5;
  //else if(t>14.5*T && t<=16.5*T)
    //p=0;
      s0=p*cos(pi*t/T);
      if (t-2*T*(int)(t/(2*T))<0.5*T) {
          s1=c;
          s2=-0.5*(l+c)*cos(pi*(t-2*T*(int)(t/(2*T)))/(0.5*T))+0.5*(l+c);
          sHeel1 = 0.5*s0+s1+com;
          sHeel2 = -0.5*s0+s2+com;
          sKnee1 = 2*s1-2*com;
          sKnee2 = 2*s2-2*com;          
      } else if (t-2*T*(int)(t/(2*T))<T) {
          s1=0.5*c*cos(pi*(t-2*T*(int)(t/(2*T))-0.5*T)/(0.5*T))+0.5*c;
          s2=0.5*(l)*cos(pi*(t-2*T*(int)(t/(2*T))-0.5*T)/(0.5*T))+0.5*l+c;
          sHeel1 = 0.5*s0+s1+com;
          sHeel2 = -0.5*s0+s2+com;
          sKnee1 = 2*s1-2*com;
          sKnee2 = 2*s2-2*com;          
      } else if (t-2*T*(int)(t/(2*T))<1.5*T) {
          s1=-0.5*(l+c)*cos(pi*(t-2*T*(int)(t/(2*T))-T)/(0.5*T))+0.5*(l+c);
          s2=c;
          sHeel1 = 0.5*s0+s1+com;
          sHeel2 = -0.5*s0+s2+com;
          sKnee1 = 2*s1-2*com;
          sKnee2 = 2*s2-2*com;          
      } else {
          s1=0.5*(l)*cos(pi*(t-2*T*(int)(t/(2*T))-1.5*T)/(0.5*T))+0.5*l+c;
          s2=0.5*c*cos(pi*(t-2*T*(int)(t/(2*T))-1.5*T)/(0.5*T))+0.5*c;
          sHeel1 = 0.5*s0+s1+com;
          sHeel2 = -0.5*s0+s2+com;
          sKnee1 = 2*s1-2*com;
          sKnee2 = 2*s2-2*com;           
      }


        sAnkle1 = sKnee1 - sHeel1 + cankle1;
        sAnkle2 = sKnee2 - sHeel2 + cankle2;      

      /*

      if(t<=T)//half T start walking
      {
        sHeel1 = -0.5*s0+s2;
        sHeel2 = -0.5*s0+s2;
        sKnee1 = 2*s2;
        sKnee2 = 2*s2;
        sAnkle1 = sKnee1 - sHeel1;
        sAnkle2 = sKnee2 - sHeel2;
      }
      else if(t<=16*T)
      {
        sHeel1 = 0.5*s0+s1+com;
        sHeel2 = -0.5*s0+s2+com;
        sKnee1 = 2*s1-com;
        sKnee2 = 2*s2-com;
        sAnkle1 = sKnee1 - sHeel1;
        sAnkle2 = sKnee2 - sHeel2;
      }
      else if(t>16*T && t<16.5*T)
      {
        sHeel1 = -0.5*s0+s2;
        sHeel2 = -0.5*s0+s2;
        sKnee1 = 2*s2;
        sKnee2 = 2*s2;
        sAnkle1 = sKnee1 - sHeel1;
        sAnkle2 = sKnee2 - sHeel2;    
      }*/
  //}

  
    //printf("%f\n",sHeel1);
  dxl_actuator[0].set_goal_theta(sHeel1);           // heel1 sagittal
  dxl_actuator[1].set_goal_theta(sKnee1);           // heel2 sagittal
  dxl_actuator[2].set_goal_theta(sAnkle1);  // knee1 sagittal
  dxl_actuator[3].set_goal_theta(sHeel2); // knee2 sagittal
  dxl_actuator[4].set_goal_theta(sKnee2);    // ankle1 lateral
  dxl_actuator[5].set_goal_theta(sAnkle2);  
}

void  Motion_Ctrl::get_inverse_kinematics(float L1, float L2, float X, float Y, float *Theta1, float *Theta2)
{
  float r=sqrt(X*X+Y*Y);
  float d=fabs((r*r-L1*L1-L2*L2)/(2*L1*L2));  
  if(d>=1)// over the workspace
  {
    *Theta2=0;
    *Theta1=atan(X/Y)+atan((L2*sin(*Theta2))/(L1+L2*cos(*Theta2)));
    printf("Over the WorkSpace!\n");
  }
  else
  {
    *Theta2=atan(sqrt(1-d*d)/d); 
    *Theta1=atan(X/Y)+atan((L2*sin(*Theta2))/(L1+L2*cos(*Theta2)));
  }
}

void  Motion_Ctrl::get_forward_kinematics(float L1, float L2, float *X, float *Y, float Theta1, float Theta2)
{
  *X=L1*sin(Theta1)+L2*sin(Theta1-Theta2);
  *Y=L1*cos(Theta1)+L2*cos(Theta1-Theta2);
}

void  Motion_Ctrl::run_tra(dxl_Actuator *dxl_actuator, int nNum)
{
  float t = _time;// + T/2;
  float L_Upper=275;
  float L_Lower=275;
  

  if(t<=0.5*T)
  {
    float x_ankle=200/T*t;
    float y_ankle=L_Upper+L_Lower-10-100*sin(2*pi/T*t);

    float theta_1=0;
    float theta_2=0;

    get_inverse_kinematics(L_Upper,L_Lower,x_ankle,y_ankle,&theta_1,&theta_2);    
    dxl_actuator[0].set_goal_theta(theta_1*180/pi);           // heel1 sagittal
    dxl_actuator[1].set_goal_theta(theta_2*180/pi);           // heel2 sagittal
    dxl_actuator[2].set_goal_theta(theta_2*180/pi-theta_1*180/pi);  // knee1 sagittal
    
    float x_hip=200/T*t;
    float y_hip=L_Upper+L_Lower-100;
    get_inverse_kinematics(L_Upper,L_Lower,x_hip,y_hip,&theta_1,&theta_2);    

    dxl_actuator[3].set_goal_theta(theta_2*180/pi-theta_1*180/pi); // knee2 sagittal
    dxl_actuator[4].set_goal_theta(theta_2*180/pi);    // ankle1 lateral
    dxl_actuator[5].set_goal_theta(theta_1*180/pi);     
    //printf("%f %f %f %f %f\n",t,x_ankle,y_ankle,theta_1*180/pi,theta_2*180/pi);
      
  }
}

void  Motion_Ctrl::update_ati(gAti_Data *_ati)
{
  for(int i=0;i<2;i++)
  {
    m_gati_data[i].Fx = _ati[i].Fx;
    m_gati_data[i].Fy = _ati[i].Fy;
    m_gati_data[i].Fz = _ati[i].Fz;
    m_gati_data[i].Tx = _ati[i].Tx;
    m_gati_data[i].Ty = _ati[i].Ty;
    m_gati_data[i].Tz = _ati[i].Tz;
  }
}

void  Motion_Ctrl::update_mti(gMti_Data _mti)
{
  m_gmti_data.eulerangles.Roll = _mti.eulerangles.Roll;
  m_gmti_data.eulerangles.Pitch = _mti.eulerangles.Pitch;
  m_gmti_data.eulerangles.Yaw = _mti.eulerangles.Yaw;

  m_gmti_data.acceleration.accX = _mti.acceleration.accX;
  m_gmti_data.acceleration.accY = _mti.acceleration.accY;
  m_gmti_data.acceleration.accZ = _mti.acceleration.accZ;

  m_gmti_data.rateofturn.gyrX = _mti.rateofturn.gyrX;
  m_gmti_data.rateofturn.gyrY = _mti.rateofturn.gyrY;
  m_gmti_data.rateofturn.gyrZ = _mti.rateofturn.gyrZ;
}

////////////////////////////////////////  ROBOT  /////////////////////////////////////////
Robot_Model::Robot_Model()
{
  g = 9.8;
  L0 = 0.109;
  L1 = 0.275;
  L2 = 0.275;
  L3_y = 0.09;
  L3_z = 0.249012;
  L3 = sqrt(L3_y*L3_y + L3_z*L3_z);
  L4 = 0.18;
  L5 = 0.275;
  L6 = 0.275;
  L7 = 0.109;

  P0 = 0.055;
  P1 = 0.124;
  P2 = 0.125;

  M0 = 1.04;
  M1 = 3.74;
  M2 = 4.4;
  M4 = 16.2;
  M_motor_100W = 0.732;
  M_motor_200W = 0.855;


  for(int i=0;i<9;i++)
  {
    pos_joint[i].x = 0.0;
    pos_joint[i].y = 0.0;
    pos_joint[i].z = 0.0;
  }

  for(int i=0;i<9;i++)
  {
    phi_joint[i].phi_x = 0.0;
    phi_joint[i].phi_y = 0.0;
    phi_joint[i].phi_z = 0.0;
  }

  for(int i=0;i<9;i++)
  {
    for(int j=0;j<3;j++)
    {
      for(int k=0;k<3;k++)
      {
        C[i].t[j][k] = 0.0;
      }
    }
  }

  pos_L[0].x = 0.0;
  pos_L[0].y = 0.0;
  pos_L[0].z = L0;

  pos_L[1].x = 0.0;
  pos_L[1].y = 0.0;
  pos_L[1].z = L1;

  pos_L[2].x = 0.0;
  pos_L[2].y = 0.0;
  pos_L[2].z = L2;

  pos_L[3].x = 0.0;
  pos_L[3].y = L3_y;
  pos_L[3].z = L3_z;

  pos_L[4].x = 0.0;
  pos_L[4].y = L4;
  pos_L[4].z = 0.0;

  pos_L[5].x = 0.0;
  pos_L[5].y = 0.0;
  pos_L[5].z = -L5;

  pos_L[6].x = 0.0;
  pos_L[6].y = 0.0;
  pos_L[6].z = -L6;

  pos_L[7].x = 0.0;
  pos_L[7].y = 0.0;
  pos_L[7].z = -L7;


  pos_P[0].x = 0.0;
  pos_P[0].y = 0.0;
  pos_P[0].z = P0;

  pos_P[1].x = 0.0;
  pos_P[1].y = 0.0;
  pos_P[1].z = P1;

  pos_P[2].x = 0.0;
  pos_P[2].y = 0.0;
  pos_P[2].z = P2;

  pos_P[3].x = 0.0;
  pos_P[3].y = L3_y;
  pos_P[3].z = L3_z;

  pos_P[4].x = 0.0;
  pos_P[4].y = L4;
  pos_P[4].z = 0.0;

  pos_P[5].x = 0.0;
  pos_P[5].y = 0.0;
  pos_P[5].z = -(L5 - P2);

  pos_P[6].x = 0.0;
  pos_P[6].y = 0.0;
  pos_P[6].z = -(L6 - P1);

  pos_P[7].x = 0.0;
  pos_P[7].y = 0.0;
  pos_P[7].z = -(L7 - P0);
}

void Robot_Model::update_phi(dxl_Actuator *dxl_actuator)
{
  phi_joint[0].phi_x = 0.0;
  phi_joint[0].phi_y = 0.0;
  phi_joint[0].phi_z = 0.0;

  phi_joint[1].phi_x = dxl_actuator[0].get_present_theta();
  phi_joint[1].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[1].phi_z = 0.0;

  phi_joint[2].phi_x = 0.0;
  phi_joint[2].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[2].phi_z = 0.0;

  phi_joint[3].phi_x = dxl_actuator[0].get_present_theta();
  phi_joint[3].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[3].phi_z = dxl_actuator[0].get_present_theta();

  phi_joint[5].phi_x = dxl_actuator[0].get_present_theta();
  phi_joint[5].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[5].phi_z = dxl_actuator[0].get_present_theta();

  phi_joint[6].phi_x = 0.0;
  phi_joint[6].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[6].phi_z = 0.0;

  phi_joint[7].phi_x = dxl_actuator[0].get_present_theta();
  phi_joint[7].phi_y = dxl_actuator[0].get_present_theta();
  phi_joint[7].phi_z = 0.0;
}

void Robot_Model::update_phi()
{
  phi_joint[0].phi_x = pi/6;
  phi_joint[0].phi_y = pi/3;
  phi_joint[0].phi_z = pi/4;

  phi_joint[1].phi_x = pi/6;
  phi_joint[1].phi_y = pi/3;
  phi_joint[1].phi_z = pi/3;

  phi_joint[2].phi_x = pi/6;
  phi_joint[2].phi_y = pi/3;
  phi_joint[2].phi_z = pi/3;

  phi_joint[3].phi_x = pi/6;
  phi_joint[3].phi_y = pi/3;
  phi_joint[3].phi_z = pi/3;

  phi_joint[5].phi_x = pi/6;
  phi_joint[5].phi_y = pi/3;
  phi_joint[5].phi_z = pi/4;

  phi_joint[6].phi_x = pi/4;
  phi_joint[6].phi_y = pi/4;
  phi_joint[6].phi_z = pi/4;

  phi_joint[7].phi_x = pi/3;
  phi_joint[7].phi_y = pi/4;
  phi_joint[7].phi_z = pi/3;
}

void Robot_Model::phi_to_matrix(float phi, int n, struct Trans_matrix *_C)
{
  if(n == 0)   //rotate around x axis
  {
    _C->t[0][0] = 1.0;
    _C->t[0][1] = 0.0;
    _C->t[0][2] = 0.0;

    _C->t[1][0] = 0.0;
    _C->t[1][1] = cos(phi);
    _C->t[1][2] = sin(phi);

    _C->t[2][0] = 0.0;
    _C->t[2][1] = -sin(phi);
    _C->t[2][2] = cos(phi);

  }

  if(n == 1)
  {
    _C->t[0][0] = cos(phi);
    _C->t[0][1] = 0.0;
    _C->t[0][2] = -sin(phi);

    _C->t[1][0] = 0.0;
    _C->t[1][1] = 1.0;
    _C->t[1][2] = 0.0;

    _C->t[2][0] = sin(phi);
    _C->t[2][1] = 0.0;
    _C->t[2][2] = cos(phi);    
  }

  if(n == 2)
  {
    _C->t[0][0] = cos(phi);
    _C->t[0][1] = sin(phi);
    _C->t[0][2] = 0.0;

    _C->t[1][0] = -sin(phi);
    _C->t[1][1] = cos(phi);
    _C->t[1][2] = 0.0;

    _C->t[2][0] = 0.0;
    _C->t[2][1] = 0.0;
    _C->t[2][2] = 1.0;    
  }
}

void Robot_Model::vector_plus_vector( struct Pos *pos1, struct Pos *pos2, struct Pos *pos_result)
{
  pos_result->x = pos1->x + pos2->x;
  pos_result->y = pos1->y + pos2->y;
  pos_result->z = pos1->z + pos2->z;
}

void Robot_Model::matrix_mul_matrix(struct Trans_matrix *_C1, struct Trans_matrix *_C2, struct Trans_matrix *_C_result)
{
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      _C_result->t[i][j] = _C1->t[i][0] * _C2->t[0][j] + _C1->t[i][1] * _C2->t[1][j] + _C1->t[i][2] * _C2->t[2][j];
    }
  }
}

void Robot_Model::matrix_mul_vector(struct Trans_matrix *_C, struct Pos *pos, struct Pos *pos_result)
{
  pos_result->x = _C->t[0][0] * pos->x + _C->t[0][1] * pos->y +  _C->t[0][2] * pos->z;
  pos_result->y = _C->t[1][0] * pos->x + _C->t[1][1] * pos->y +  _C->t[1][2] * pos->z;
  pos_result->z = _C->t[2][0] * pos->x + _C->t[2][1] * pos->y +  _C->t[2][2] * pos->z;
}

void Robot_Model::vector_to_matrix(struct Phi *phi, struct Trans_matrix *_C)
{
  struct Trans_matrix C_temp_x;
  struct Trans_matrix C_temp_y;
  struct Trans_matrix C_temp_z;
  struct Trans_matrix C_temp;

  phi_to_matrix(phi->phi_x, 0, &C_temp_x);
  phi_to_matrix(phi->phi_y, 1, &C_temp_y);
  phi_to_matrix(phi->phi_z, 2, &C_temp_z);

  matrix_mul_matrix(&C_temp_y, &C_temp_z, &C_temp);
  matrix_mul_matrix(&C_temp_x, &C_temp, _C);
}

void Robot_Model::vector_to_inv_matrix(struct Phi *phi, struct Trans_matrix *_C)
{
  struct Trans_matrix C_temp_x;
  struct Trans_matrix C_temp_y;
  struct Trans_matrix C_temp_z;
  struct Trans_matrix C_temp;

  phi_to_matrix(-phi->phi_x, 0, &C_temp_x);
  phi_to_matrix(-phi->phi_y, 1, &C_temp_y);
  phi_to_matrix(-phi->phi_z, 2, &C_temp_z);

  matrix_mul_matrix(&C_temp_y, &C_temp_x, &C_temp);
  matrix_mul_matrix(&C_temp_z, &C_temp, _C);
}

void Robot_Model::update_trans_matrix()
{
  struct Trans_matrix C_temp;

  vector_to_matrix(phi_joint+0,C+0);

  vector_to_matrix(phi_joint+1,&C_temp);
  matrix_mul_matrix(&C_temp,C+0,C+1);

  vector_to_matrix(phi_joint+2,&C_temp);
  matrix_mul_matrix(&C_temp,C+1,C+2);

  vector_to_matrix(phi_joint+3,&C_temp);
  matrix_mul_matrix(&C_temp,C+2,C+3);

  vector_to_matrix(phi_joint+5,&C_temp);
  matrix_mul_matrix(&C_temp,C+3,C+5);

  vector_to_matrix(phi_joint+6,&C_temp);
  matrix_mul_matrix(&C_temp,C+5,C+6);

  vector_to_matrix(phi_joint+7,&C_temp);
  matrix_mul_matrix(&C_temp,C+6,C+7);
}

void Robot_Model::update_inv_trans_matrix()
{
  struct Trans_matrix C_temp;
  vector_to_inv_matrix(phi_joint+0,C_inv+0);

  vector_to_inv_matrix(phi_joint+1,&C_temp);
  matrix_mul_matrix(C_inv+0,&C_temp,C_inv+1);

  vector_to_inv_matrix(phi_joint+2,&C_temp);
  matrix_mul_matrix(C_inv+1,&C_temp,C_inv+2);

  vector_to_inv_matrix(phi_joint+3,&C_temp);
  matrix_mul_matrix(C_inv+2,&C_temp,C_inv+3);

  vector_to_inv_matrix(phi_joint+5,&C_temp);
  matrix_mul_matrix(C_inv+3,&C_temp,C_inv+5);

  vector_to_inv_matrix(phi_joint+6,&C_temp);
  matrix_mul_matrix(C_inv+5,&C_temp,C_inv+6);

  vector_to_inv_matrix(phi_joint+7,&C_temp);
  matrix_mul_matrix(C_inv+6,&C_temp,C_inv+7);
}

void Robot_Model::update_joint_position()
{
  struct Pos pos_temp;
  matrix_mul_vector(C_inv+0, pos_L+0, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+0, pos_joint+1);

  matrix_mul_vector(C_inv+1, pos_L+1, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+1, pos_joint+2);

  matrix_mul_vector(C_inv+2, pos_L+2, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+2, pos_joint+3);

  matrix_mul_vector(C_inv+3, pos_L+3, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+3, pos_joint+4);

  matrix_mul_vector(C_inv+3, pos_L+4, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+3, pos_joint+5);

  matrix_mul_vector(C_inv+5, pos_L+5, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+5, pos_joint+6);

  matrix_mul_vector(C_inv+6, pos_L+6, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+6, pos_joint+7);

  matrix_mul_vector(C_inv+7, pos_L+7, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+7, pos_joint+8);
}

void Robot_Model::update_com_position()
{
  struct Pos pos_temp;
  matrix_mul_vector(C_inv+0, pos_P+0, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+0, pos_com+0);

  matrix_mul_vector(C_inv+1, pos_P+1, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+1, pos_com+1);

  matrix_mul_vector(C_inv+2, pos_P+2, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+2, pos_com+2);

  matrix_mul_vector(C_inv+3, pos_P+3, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+3, pos_com+4);

  matrix_mul_vector(C_inv+5, pos_P+5, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+5, pos_com+5);

  matrix_mul_vector(C_inv+6, pos_P+6, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+6, pos_com+6);

  matrix_mul_vector(C_inv+7, pos_P+7, &pos_temp);
  vector_plus_vector(&pos_temp, pos_joint+7, pos_com+7); 
}

void Robot_Model::calculate_gravity_compsation()
{
  float X_com, Y_com, Z_com;
  float l1, l2;
  float G1,G2;
  float X_G_moment_around_joint, Y_G_moment_around_joint, Z_G_moment_around_joint;
  struct Pos pos_temp;
  struct Pos pos_temp_1;


  X_com = (M1*(pos_com+1)->x + M2*(pos_com+2)->x + M4*(pos_com+4)->x + M2*(pos_com+5)->x + M1*(pos_com+6)->x)/(2*M1 + 2*M2 + M4);
  Y_com = (M1*(pos_com+1)->y + M2*(pos_com+2)->y + M4*(pos_com+4)->y + M2*(pos_com+5)->y + M1*(pos_com+6)->y)/(2*M1 + 2*M2 + M4);
  Z_com = (M1*(pos_com+1)->z + M2*(pos_com+2)->z + M4*(pos_com+4)->z + M2*(pos_com+5)->z + M1*(pos_com+6)->z)/(2*M1 + 2*M2 + M4);

  // printf("X_com=%f,Y_com=%f,Z_com=%f\n",X_com,Y_com,Z_com);
  // printf("X_com=%f,Y_com=%f,Z_com=%f\n",X_com,Y_com,Z_com);

  l1 = sqrt((X_com - (pos_joint+1)->x)*(X_com - (pos_joint+1)->x) + (Y_com - (pos_joint+1)->y)*(Y_com - (pos_joint+1)->y) + (Z_com - (pos_joint+1)->z)*(Z_com - (pos_joint+1)->z));
  l2 = sqrt((X_com - (pos_joint+7)->x)*(X_com - (pos_joint+7)->x) + (Y_com - (pos_joint+7)->y)*(Y_com - (pos_joint+7)->y) + (Z_com - (pos_joint+7)->z)*(Z_com - (pos_joint+7)->z));

  G1 = l2/(l1+l2)*(2*M1 + 2*M2 + M4)*g;
  G2 = l1/(l1+l2)*(2*M1 + 2*M2 + M4)*g;

  // printf("l1=%f,l2=%f\n",l1,l2);
  // printf("G1=%f,G2=%f\n",G1,G2);

  X_G_moment_around_joint = (Y_com - (pos_joint+1)->y) * (-G1);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+1)->x) * (-G1);
  Z_G_moment_around_joint = 0;

  // printf("X_G_moment_around_joint=%f,Y_G_moment_around_joint=%f\n",X_G_moment_around_joint,Y_G_moment_around_joint);

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+1, &pos_temp, &pos_temp_1);
  gravity_comp[0] = -pos_temp_1.x;
  gravity_comp[1] = -pos_temp_1.y;



  X_G_moment_around_joint = (Y_com - (pos_joint+7)->y) * (-G2);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+7)->x) * (-G2);
  Z_G_moment_around_joint = 0;

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+7, &pos_temp, &pos_temp_1);
  gravity_comp[11] = -pos_temp_1.x;
  gravity_comp[10] = -pos_temp_1.y;


  X_com = (M2*(pos_com+2)->x + M4*(pos_com+4)->x + M2*(pos_com+5)->x)/(2*M2 + M4);
  Y_com = (M2*(pos_com+2)->y + M4*(pos_com+4)->y + M2*(pos_com+5)->y)/(2*M2 + M4);
  Z_com = (M2*(pos_com+2)->z + M4*(pos_com+4)->z + M2*(pos_com+5)->z)/(2*M2 + M4);

  l1 = sqrt((X_com - (pos_joint+2)->x)*(X_com - (pos_joint+2)->x) + (Y_com - (pos_joint+2)->y)*(Y_com - (pos_joint+2)->y) + (Z_com - (pos_joint+2)->z)*(Z_com - (pos_joint+2)->z));
  l2 = sqrt((X_com - (pos_joint+6)->x)*(X_com - (pos_joint+6)->x) + (Y_com - (pos_joint+6)->y)*(Y_com - (pos_joint+6)->y) + (Z_com - (pos_joint+6)->z)*(Z_com - (pos_joint+6)->z));

  G1 = l2/(l1+l2)*(2*M2 + M4)*g;
  G2 = l1/(l1+l2)*(2*M2 + M4)*g;

  X_G_moment_around_joint = (Y_com - (pos_joint+2)->y) * (-G1);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+2)->x) * (-G1);
  Z_G_moment_around_joint = 0;

  // printf("X_G_moment_around_joint=%f,Y_G_moment_around_joint=%f\n",X_G_moment_around_joint,Y_G_moment_around_joint);

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+2, &pos_temp, &pos_temp_1);
  gravity_comp[2] = -pos_temp_1.y;

  X_G_moment_around_joint = (Y_com - (pos_joint+6)->y) * (-G2);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+6)->x) * (-G2);
  Z_G_moment_around_joint = 0;

  // printf("X_G_moment_around_joint=%f,Y_G_moment_around_joint=%f\n",X_G_moment_around_joint,Y_G_moment_around_joint);

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+6, &pos_temp, &pos_temp_1);
  gravity_comp[9] = -pos_temp_1.y;

  X_com = (pos_com+4)->x;
  Y_com = (pos_com+4)->y;
  Z_com = (pos_com+4)->z;

  l1 = sqrt((X_com - (pos_joint+3)->x)*(X_com - (pos_joint+3)->x) + (Y_com - (pos_joint+3)->y)*(Y_com - (pos_joint+3)->y) + (Z_com - (pos_joint+3)->z)*(Z_com - (pos_joint+3)->z));
  l2 = sqrt((X_com - (pos_joint+5)->x)*(X_com - (pos_joint+5)->x) + (Y_com - (pos_joint+5)->y)*(Y_com - (pos_joint+5)->y) + (Z_com - (pos_joint+5)->z)*(Z_com - (pos_joint+5)->z));

  G1 = l2/(l1+l2)*M4*g;
  G2 = l1/(l1+l2)*M4*g;

  X_G_moment_around_joint = (Y_com - (pos_joint+3)->y) * (-G1);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+3)->x) * (-G1);
  Z_G_moment_around_joint = 0;

  // printf("X_G_moment_around_joint=%f,Y_G_moment_around_joint=%f\n",X_G_moment_around_joint,Y_G_moment_around_joint);

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+3, &pos_temp, &pos_temp_1);
  gravity_comp[3] = -pos_temp_1.x;
  gravity_comp[4] = -pos_temp_1.y;
  gravity_comp[5] = -pos_temp_1.z;

  X_G_moment_around_joint = (Y_com - (pos_joint+5)->y) * (-G2);
  Y_G_moment_around_joint =  -(X_com - (pos_joint+5)->x) * (-G2);
  Z_G_moment_around_joint = 0;

  // printf("X_G_moment_around_joint=%f,Y_G_moment_around_joint=%f\n",X_G_moment_around_joint,Y_G_moment_around_joint);

  pos_temp.x = X_G_moment_around_joint;
  pos_temp.y = Y_G_moment_around_joint;
  pos_temp.z = Z_G_moment_around_joint;

  matrix_mul_vector(C+5, &pos_temp, &pos_temp_1);
  gravity_comp[8] = -pos_temp_1.x;
  gravity_comp[7] = -pos_temp_1.y;
  gravity_comp[6] = -pos_temp_1.z;
}

void Robot_Model::matrix_test()
{
  struct Trans_matrix C_temp;

  // vector_to_matrix(phi_joint+1,&C_temp);
  // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);


  // // C_temp = C[0];
  // // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);

  // C_temp = C_inv[0];
  // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);

  // // C_temp = C[2];
  // // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);

  // C_temp = C_inv[2];
  // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);

  

  matrix_mul_matrix(C+0,C_inv+0,&C_temp);
  printf("\nC00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+1,C_inv+1,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+2,C_inv+2,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+3,C_inv+3,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  // matrix_mul_matrix(C+4,C_inv+4,&C_temp);
  // printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+5,C_inv+5,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+6,C_inv+6,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
  matrix_mul_matrix(C+7,C_inv+7,&C_temp);
  printf("C00=%f C01=%f C02=%f\t  C10=%f C11=%f C12=%f\t  C20=%f C21=%f C22=%f\n",C_temp.t[0][0], C_temp.t[0][1], C_temp.t[0][2], C_temp.t[1][0], C_temp.t[1][1], C_temp.t[1][2], C_temp.t[2][0], C_temp.t[2][1], C_temp.t[2][2]);
  
}


















